"""Dead-reckoning SLAM for the RoverC, which has no range sensors.

Localization: there are no wheel encoders, so position comes from the velocity
the rover was commanded (/rover/cmd_applied from rover_driver: the input,
used for prediction), and heading comes from the IMU gyro. A small Kalman
filter (heading.py) learns the gyro's bias whenever the rover is commanded to
stand still, which keeps heading drift low.

Mapping: an occupancy grid, 5 cm cells by default:
- Free: cells the rover's footprint has passed over.
- Occupied: if the accelerometer feels a hard jolt while the rover is driving
  steadily, it's taken as a bump into something, and the cells just ahead are
  marked. Odometry then holds still until the command changes, because the
  rover is pushing against something rather than moving.
- Unknown: everything else.

Nothing corrects position error, so it grows with distance driven. Call
/rover/reset_map when the rover is back at a known start.

Subscribes  /rover/cmd_applied (geometry_msgs/TwistStamped), /rover/imu (sensor_msgs/Imu)
Publishes   /odom (nav_msgs/Odometry) and TF odom -> base_link, at the IMU rate (50 Hz)
            /map (nav_msgs/OccupancyGrid, latched, 1 Hz when changed)
            /rover/path (nav_msgs/Path, 2 Hz)
            /rover/footprint (geometry_msgs/PolygonStamped, latched)
            /diagnostics (status 'rover_slam: odometry', 1 Hz)
Services    /rover/reset_map (std_srvs/Trigger): pose back to zero, map and path cleared
"""

from array import array
import math

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from rover_slam.grid_map import FREE, GridMap, OCCUPIED
from rover_slam.heading import HeadingFilter, UpVector
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

CMD_TIMEOUT = 0.3        # s; older /rover/cmd_applied counts as stopped
IMU_TIMEOUT = 0.5        # s; then heading falls back to the commanded turn rate
DISTANCE_ERROR = 0.15    # rough position error per metre driven, for the covariance
PATH_STEP = 0.02         # m; path point spacing
PATH_MAX = 10000
UNKNOWN_VARIANCE = 1e6   # for the axes a ground rover doesn't estimate (z, roll, pitch)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RoverSlam(Node):

    def __init__(self):
        super().__init__('rover_slam')

        def param(name, default):
            return self.declare_parameter(name, default).value

        self._resolution = param('resolution', 0.05)
        map_size = param('map_size', 10.0)
        self._length = param('footprint_length', 0.14)
        self._width = param('footprint_width', 0.12)
        self._linear_scale = param('linear_scale', 1.0)
        self._lateral_scale = param('lateral_scale', 1.0)
        self._odom_frame = param('odom_frame', 'odom')
        self._base_frame = param('base_frame', 'base_link')
        self._publish_tf = param('publish_tf', True)
        self._bump_detection = param('bump_detection', True)
        self._bump_threshold = param('bump_threshold', 8.0)
        self._still_delay = param('still_delay', 0.3)

        self._map = GridMap(self._resolution, map_size)
        self._up = UpVector()
        self._heading = HeadingFilter()

        # Pose and its rough variance.
        self._x = self._y = self._yaw = 0.0
        self._var_xy = self._var_yaw = 0.0
        self._vx = self._vy = self._wz = 0.0
        self._distance = 0.0

        # The input: what the rover was told to do.
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time = -math.inf
        self._last_moving_cmd = -math.inf
        self._cmd_changed = -math.inf
        self._blocked = False  # after a bump, until the command changes

        self._last_imu_stamp = None
        self._last_imu_time = -math.inf
        self._last_fallback = None
        self._still = True

        self._map_dirty = True
        self._last_mark = None
        self._path = Path()
        self._path.header.frame_id = self._odom_frame
        self._path_dirty = False
        self._bumps = 0
        self._last_bump = -math.inf

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self._path_pub = self.create_publisher(Path, 'rover/path', 10)
        self._footprint_pub = self.create_publisher(PolygonStamped, 'rover/footprint', latched)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(TwistStamped, 'rover/cmd_applied', self._on_cmd, 10)
        self.create_subscription(Imu, 'rover/imu', self._on_imu, qos_profile_sensor_data)
        self.create_service(Trigger, 'rover/reset_map', self._on_reset)
        self.create_timer(0.05, self._fallback_tick)
        self.create_timer(1.0, self._publish_map)
        self.create_timer(0.5, self._publish_path)
        self.create_timer(1.0, self._publish_diagnostics)

        self._publish_footprint()
        self._mark()
        self.get_logger().info(
            f'Map: {self._map.cells}x{self._map.cells} cells of {self._resolution * 100:.0f} cm, '
            'starting at the centre')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- Inputs ----------

    def _on_cmd(self, msg: TwistStamped):
        t = msg.twist
        cmd = (t.linear.x, t.linear.y, t.angular.z)
        now = self._now()
        if max(abs(a - b) for a, b in zip(cmd, self._cmd)) > 0.01:
            self._cmd_changed = now
            self._blocked = False
        if any(abs(v) > 1e-6 for v in cmd):
            self._last_moving_cmd = now
        self._cmd = cmd
        self._cmd_time = now

    def _input(self, now: float):
        """The current command and whether the rover should be standing still."""
        if now - self._cmd_time > CMD_TIMEOUT:
            return (0.0, 0.0, 0.0), True  # driver gone: the rover's own timeout stops it
        return self._cmd, now - self._last_moving_cmd > self._still_delay

    def _on_imu(self, msg: Imu):
        now = self._now()
        self._last_imu_time = now
        self._last_fallback = None
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        previous, self._last_imu_stamp = self._last_imu_stamp, stamp
        if previous is None:
            return
        dt = stamp - previous
        if not 0.0 < dt < 0.2:
            return

        a = msg.linear_acceleration
        g = msg.angular_velocity
        accel = (a.x, a.y, a.z)
        self._up.update(accel)

        (vx, vy, wz_cmd), still = self._input(now)
        self._still = still
        rate = self._heading.update(dt, self._up.yaw_rate((g.x, g.y, g.z)), wz_cmd, still)
        if self._bump_detection:
            self._check_bump(now, accel, vx, vy)
        self._integrate(dt, vx, vy, rate)
        self._publish_odom(msg.header.stamp)

    def _fallback_tick(self):
        # No IMU: keep going on the commanded turn rate alone.
        now = self._now()
        if now - self._last_imu_time < IMU_TIMEOUT:
            return
        if self._last_fallback is None:
            self._last_fallback = now
            self.get_logger().warning('No IMU data: heading comes from the commands only',
                                      throttle_duration_sec=30.0)
            return
        dt, self._last_fallback = now - self._last_fallback, now
        (vx, vy, wz), _ = self._input(now)
        self._integrate(dt, vx, vy, wz)
        self._publish_odom(self.get_clock().now().to_msg())

    # ---------- Dead reckoning ----------

    def _integrate(self, dt, vx, vy, wz):
        if self._blocked:
            vx = vy = 0.0
        vx *= self._linear_scale
        vy *= self._lateral_scale
        mid = self._yaw + 0.5 * wz * dt
        c, s = math.cos(mid), math.sin(mid)
        self._x += (vx * c - vy * s) * dt
        self._y += (vx * s + vy * c) * dt
        self._yaw = _wrap(self._yaw + wz * dt)
        self._vx, self._vy, self._wz = vx, vy, wz

        step = math.hypot(vx, vy) * dt
        self._distance += step
        self._var_yaw += (self._heading.rate_var + self._heading.bias_var) * dt * dt
        self._var_xy += (DISTANCE_ERROR * step) ** 2 + self._var_yaw * step * step
        self._mark()

    def _check_bump(self, now, accel, vx, vy):
        steady = now - self._cmd_changed > 0.3  # starting and stopping jolt too
        if (self._blocked or not steady or math.hypot(vx, vy) < 0.03
                or now - self._last_bump < 1.0
                or self._up.horizontal(accel) < self._bump_threshold):
            return
        self._last_bump = now
        self._bumps += 1
        self._blocked = True

        # Wall goes just past the footprint's edge, in the direction of travel.
        travel = math.atan2(vy, vx)  # in the rover's frame
        c, s = abs(math.cos(travel)), abs(math.sin(travel))
        half_l, half_w = self._length / 2, self._width / 2
        reach = min(half_l / max(c, 1e-6), half_w / max(s, 1e-6)) + self._resolution / 2
        across = half_w if c >= s else half_l
        heading = self._yaw + travel
        hx = self._x + reach * math.cos(heading)
        hy = self._y + reach * math.sin(heading)
        if self._map.mark_obstacle(hx, hy, heading, across):
            self._map_dirty = True
        self.get_logger().info(f'Bump at ({hx:.2f}, {hy:.2f}) m')

    def _mark(self):
        pose = (self._x, self._y, self._yaw)
        last = self._last_mark
        if last is not None and math.hypot(pose[0] - last[0], pose[1] - last[1]) < PATH_STEP \
                and abs(_wrap(pose[2] - last[2])) < math.radians(5):
            return
        self._last_mark = pose
        if self._map.mark_footprint(*pose, self._length, self._width):
            self._map_dirty = True
        self._append_path()

    # ---------- Output ----------

    def _publish_odom(self, stamp):
        qz, qw = math.sin(self._yaw / 2), math.cos(self._yaw / 2)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = cov[7] = self._var_xy
        cov[14] = cov[21] = cov[28] = UNKNOWN_VARIANCE
        cov[35] = self._var_yaw
        odom.pose.covariance = cov
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._wz
        tcov = [0.0] * 36
        tcov[0] = tcov[7] = (DISTANCE_ERROR * math.hypot(self._vx, self._vy) + 0.01) ** 2
        tcov[14] = tcov[21] = tcov[28] = UNKNOWN_VARIANCE
        tcov[35] = self._heading.rate_var + self._heading.bias_var
        odom.twist.covariance = tcov
        self._odom_pub.publish(odom)

        if self._tf is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)

    def _append_path(self):
        p = PoseStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = self._odom_frame
        p.pose.position.x = self._x
        p.pose.position.y = self._y
        p.pose.orientation.z = math.sin(self._yaw / 2)
        p.pose.orientation.w = math.cos(self._yaw / 2)
        self._path.poses.append(p)
        if len(self._path.poses) > PATH_MAX:
            del self._path.poses[:len(self._path.poses) - PATH_MAX]
        self._path_dirty = True

    def _publish_path(self, force=False):
        if not (self._path_dirty or force):
            return
        self._path_dirty = False
        self._path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(self._path)

    def _publish_map(self, force=False):
        if not (self._map_dirty or force):
            return
        self._map_dirty = False
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = float(self._resolution)
        msg.info.width = msg.info.height = self._map.cells
        msg.info.origin.position.x = msg.info.origin.position.y = self._map.origin
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', self._map.data.tobytes())
        self._map_pub.publish(msg)

    def _publish_footprint(self):
        msg = PolygonStamped()
        msg.header.frame_id = self._base_frame
        hl, hw = self._length / 2, self._width / 2
        msg.polygon.points = [Point32(x=float(x), y=float(y))
                              for x, y in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]
        self._footprint_pub.publish(msg)

    def _publish_diagnostics(self):
        imu_ok = self._now() - self._last_imu_time < IMU_TIMEOUT
        status = DiagnosticStatus(name='rover_slam: odometry', hardware_id='dead_reckoning')
        status.level = DiagnosticStatus.OK if imu_ok else DiagnosticStatus.WARN
        status.message = 'gyro + commands' if imu_ok else 'no IMU: heading from commands only'
        values = {
            'x_m': f'{self._x:.3f}',
            'y_m': f'{self._y:.3f}',
            'yaw_deg': f'{math.degrees(self._yaw):.1f}',
            'distance_m': f'{self._distance:.2f}',
            'position_sigma_m': f'{math.sqrt(self._var_xy):.3f}',
            'yaw_sigma_deg': f'{math.degrees(math.sqrt(self._var_yaw)):.2f}',
            'gyro_bias_deg_s': f'{math.degrees(self._heading.bias):.3f}',
            'gyro_bias_sigma_deg_s': f'{math.degrees(math.sqrt(self._heading.bias_var)):.3f}',
            'still': self._still,
            'blocked': self._blocked,
            'bumps': self._bumps,
            'free_cells': self._map.count(FREE),
            'occupied_cells': self._map.count(OCCUPIED),
        }
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.status.append(status)
        self._diag_pub.publish(diag)

    def _on_reset(self, request, response):
        # The gyro bias is kept: it's still right.
        self._x = self._y = self._yaw = 0.0
        self._var_xy = self._var_yaw = 0.0
        self._distance = 0.0
        self._bumps = 0
        self._blocked = False
        self._map.clear()
        self._path.poses = []
        self._last_mark = None
        self._mark()
        self._publish_map(force=True)
        self._publish_path(force=True)
        self.get_logger().info('Map and pose reset')
        response.success = True
        response.message = 'map and pose reset'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RoverSlam()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
        # SIGTERM shut ROS down mid-spin and rclpy raised instead: a normal exit.
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
