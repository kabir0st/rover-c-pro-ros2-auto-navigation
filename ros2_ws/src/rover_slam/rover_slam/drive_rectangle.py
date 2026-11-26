"""Drive the rover around a rectangle, steering by the dead-reckoning pose (/odom).

    ros2 run rover_slam drive_rectangle                          # 1.0 x 0.5 m, turning at corners
    ros2 run rover_slam drive_rectangle --width 0.6 --height 0.6 --strafe   # mecanum, never turns
    ros2 run rover_slam drive_rectangle --clockwise --laps 2

It starts wherever the rover is: the first side goes straight ahead, then it
turns left (right with --clockwise). With --strafe it keeps facing forward and
slides sideways instead of turning.

Corners and straightness come from the gyro heading, so they're good. Side
lengths are only as good as the speed calibration (linear_scale and
lateral_scale in rover_slam's config): drive one, measure it, adjust.

Needs rover_driver and rover_slam running. Clear a space first; Ctrl-C stops
the rover.
"""

import argparse
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool

RATE = 20.0                       # Hz
HEADING_GAIN = 2.0                # rad/s of correction per rad of heading error
MAX_HEADING_CORRECTION = 0.5      # rad/s
TURN_GAIN = 2.0
TURN_TOLERANCE = math.radians(2.5)
SETTLE_TIME = 0.3                 # s inside the tolerance to finish a turn
SLOWDOWN_GAIN = 2.0               # m/s per m left: slows over the last few cm
POSE_TIMEOUT = 0.5                # s


class Abort(Exception):
    pass


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class RectangleDriver(Node):

    def __init__(self, args):
        super().__init__('drive_rectangle')
        self.args = args
        self.pose = None
        self.pose_time = -math.inf
        self.locked = False
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'rover/locked', self._on_locked, latched)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.pose_time = time.monotonic()

    def _on_locked(self, msg: Bool):
        self.locked = msg.data

    # ---------- Helpers ----------

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while (left := end - time.monotonic()) > 0:
            rclpy.spin_once(self, timeout_sec=left)

    def send(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(3):
            self.send()
            self.wait(1 / RATE)

    def check(self):
        if self.locked:
            raise Abort('the rover is locked (button A or /rover/set_locked)')
        if time.monotonic() - self.pose_time > POSE_TIMEOUT:
            raise Abort('lost /odom; is rover_slam running and the rover connected?')

    def wait_for_pose(self, timeout=5.0):
        end = time.monotonic() + timeout
        while self.pose is None or time.monotonic() - self.pose_time > POSE_TIMEOUT:
            if time.monotonic() > end:
                raise Abort('no /odom; start rover_driver and rover_slam first')
            rclpy.spin_once(self, timeout_sec=0.1)

    # ---------- Moves ----------

    def drive_side(self, length, direction, heading):
        """Drive `length` metres along `direction` (unit vector in the rover's frame)
        while holding `heading`."""
        x0, y0, _ = self.pose
        dx, dy = direction
        while True:
            self.check()
            x, y, yaw = self.pose
            left = length - math.hypot(x - x0, y - y0)
            if left <= 0.0:
                break
            v = max(self.args.min_speed, min(self.args.speed, SLOWDOWN_GAIN * left))
            wz = _clamp(HEADING_GAIN * _wrap(heading - yaw), MAX_HEADING_CORRECTION)
            self.send(v * dx, v * dy, wz)
            self.wait(1 / RATE)
        self.send()

    def turn_to(self, heading):
        settled_since = None
        start = time.monotonic()
        while True:
            self.check()
            error = _wrap(heading - self.pose[2])
            if abs(error) < TURN_TOLERANCE:
                self.send()
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since > SETTLE_TIME:
                    return
            else:
                settled_since = None
                rate = min(self.args.turn_speed, max(self.args.min_turn_speed, TURN_GAIN * abs(error)))
                self.send(wz=math.copysign(rate, error))
            if time.monotonic() - start > 15.0:
                self.get_logger().warning(f'Turn not settled, {math.degrees(error):.1f}° off')
                return
            self.wait(1 / RATE)

    def run(self):
        a = self.args
        self.wait_for_pose()
        x0, y0, heading = self.pose
        sides = [a.width, a.height, a.width, a.height]
        sign = -1 if a.clockwise else 1
        # Strafe mode: forward, sideways, back, sideways the other way, always facing ahead.
        directions = [(1, 0), (0, sign), (-1, 0), (0, -sign)]
        names = ['forward', 'right' if a.clockwise else 'left', 'back',
                 'left' if a.clockwise else 'right']
        self.get_logger().info(
            f'{a.width:.2f} x {a.height:.2f} m rectangle, {a.laps} lap(s), '
            f"{'strafing' if a.strafe else 'turning'} {'clockwise' if a.clockwise else 'counter-clockwise'}")

        for lap in range(a.laps):
            for i, length in enumerate(sides):
                if a.strafe:
                    self.get_logger().info(f'Lap {lap + 1}, side {i + 1}: {names[i]} {length:.2f} m')
                    self.drive_side(length, directions[i], heading)
                else:
                    self.get_logger().info(f'Lap {lap + 1}, side {i + 1}: {length:.2f} m, then turn')
                    self.drive_side(length, (1, 0), heading)
                    heading = _wrap(heading + sign * math.pi / 2)
                    self.turn_to(heading)
        self.stop()

        x, y, yaw = self.pose
        self.get_logger().info(
            f'Done. By dead reckoning it ended {math.hypot(x - x0, y - y0) * 100:.1f} cm '
            f'and {abs(math.degrees(_wrap(yaw - heading))):.1f}° from the start. '
            'Measure the real gap to check the calibration.')


def main():
    parser = argparse.ArgumentParser(
        description='Drive the rover around a rectangle using /odom.')
    parser.add_argument('--width', type=float, default=1.0, help='first side, m (default 1.0)')
    parser.add_argument('--height', type=float, default=0.5, help='second side, m (default 0.5)')
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--clockwise', action='store_true', help='turn right instead of left')
    parser.add_argument('--strafe', action='store_true',
                        help='slide sideways instead of turning (mecanum)')
    parser.add_argument('--speed', type=float, default=0.15, help='m/s (default 0.15)')
    parser.add_argument('--min-speed', type=float, default=0.06,
                        help='slowest drive speed, above the motors\' dead zone (default 0.06)')
    parser.add_argument('--turn-speed', type=float, default=1.2, help='rad/s (default 1.2)')
    parser.add_argument('--min-turn-speed', type=float, default=0.5,
                        help='slowest turn speed (default 0.5)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.width <= 0 or args.height <= 0 or args.speed <= 0 or args.laps < 1:
        parser.error('width, height, speed and laps must be positive')

    rclpy.init()
    node = RectangleDriver(args)
    code = 0
    try:
        node.run()
    except Abort as e:
        node.get_logger().error(f'Stopped: {e}')
        code = 1
    except KeyboardInterrupt:
        code = 130
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
