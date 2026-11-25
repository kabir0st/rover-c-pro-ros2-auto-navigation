"""ROS 2 driver for the RoverC Pro firmware in firmware/rover.

Sends drive commands to the rover over UDP at a steady rate (which also acts as
the rover's heartbeat) and publishes the telemetry it sends back.

Subscribes
  /cmd_vel                geometry_msgs/Twist (TwistStamped with cmd_vel_stamped:=true)
  /rover/gripper          std_msgs/Float32, gripper servo angle in degrees (10..90)
Publishes
  /rover/imu              sensor_msgs/Imu, 50 Hz, no orientation
  /rover/imu/temperature  sensor_msgs/Temperature, 1 Hz
  /rover/battery          sensor_msgs/BatteryState, 1 Hz (the stick's battery)
  /rover/locked           std_msgs/Bool, latched, published on change
  /diagnostics            diagnostic_msgs/DiagnosticArray, 1 Hz
Services
  /rover/set_locked       std_srvs/SetBool, same as pressing button A on the stick
"""

import math
import select
import socket
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rover_driver import protocol
from sensor_msgs.msg import BatteryState, Imu, Temperature
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

STANDARD_GRAVITY = 9.80665

# Name of the link status in /diagnostics. rover_web reads it.
LINK_STATUS_NAME = 'rover_driver: link'

# Telemetry older than this counts as a lost link.
LINK_TIMEOUT = 2.0

# Lock/unlock requests ride on this many command packets, in case UDP drops one.
LOCK_REQUEST_REPEATS = 5

# Re-resolve the rover's hostname this often while the link is down.
RESOLVE_INTERVAL = 5.0

_CHARGE_TO_ROS = {
    protocol.CHARGE_CHARGING: BatteryState.POWER_SUPPLY_STATUS_CHARGING,
    protocol.CHARGE_FULL: BatteryState.POWER_SUPPLY_STATUS_FULL,
    protocol.CHARGE_DISCHARGING: BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
    protocol.CHARGE_NOT_CHARGING: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
    protocol.CHARGE_DISABLED: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
    protocol.CHARGE_IDLE: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
}


def _percent(value: float) -> int:
    """Map -1..1 to the firmware's -100..100."""
    if not math.isfinite(value):
        return 0
    return int(round(max(-1.0, min(1.0, value)) * 100))


class RoverDriver(Node):

    def __init__(self):
        super().__init__('rover_driver')
        self._host = self.declare_parameter('rover_host', 'roverc.local').value
        self._port = self.declare_parameter('rover_port', protocol.ROVER_UDP_PORT).value
        send_rate = self.declare_parameter('send_rate', 20.0).value
        self._cmd_timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self._max_linear = self.declare_parameter('max_linear_speed', 0.3).value
        self._max_angular = self.declare_parameter('max_angular_speed', 2.0).value
        stamped = self.declare_parameter('cmd_vel_stamped', False).value
        self._imu_frame = self.declare_parameter('imu_frame_id', 'imu_link').value
        self._battery_frame = self.declare_parameter('battery_frame_id', 'base_link').value
        self._send_period = 1.0 / send_rate

        # Shared between ROS callbacks and the UDP thread.
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)  # vx, vy, wz from /cmd_vel
        self._cmd_time = -math.inf
        self._grip = 0
        self._lock_flags = 0
        self._lock_repeats = 0
        self._telemetry = None
        self._last_rx = -math.inf
        self._rx_count = 0
        self._lost_count = 0

        # Only touched by the UDP thread.
        self._addr = None
        self._last_resolve = -math.inf
        self._seq = 0
        self._last_tlm_seq = None
        self._published_locked = None

        # Only touched by the 1 Hz timer.
        self._connected = False

        self._imu_pub = self.create_publisher(Imu, 'rover/imu', qos_profile_sensor_data)
        self._temp_pub = self.create_publisher(Temperature, 'rover/imu/temperature', 10)
        self._battery_pub = self.create_publisher(BatteryState, 'rover/battery', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._locked_pub = self.create_publisher(Bool, 'rover/locked', latched)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        if stamped:
            self.create_subscription(
                TwistStamped, 'cmd_vel', lambda msg: self._on_cmd_vel(msg.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Float32, 'rover/gripper', self._on_gripper, 10)
        self.create_service(SetBool, 'rover/set_locked', self._on_set_locked)
        self.create_timer(1.0, self._publish_slow)

        self.get_logger().info(f'Looking for the rover at {self._host}:{self._port}')
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', 0))
        self._running = True
        self._io_thread = threading.Thread(target=self._io_loop, daemon=True)
        self._io_thread.start()

    # ---------- ROS callbacks ----------

    def _on_cmd_vel(self, msg: Twist):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.get_logger().warning('Ignoring /cmd_vel with NaN/inf', throttle_duration_sec=5.0)
            return
        with self._lock:
            self._cmd = values
            self._cmd_time = time.monotonic()

    def _on_gripper(self, msg: Float32):
        if not math.isfinite(msg.data):
            return
        with self._lock:
            self._grip = int(round(max(10.0, min(90.0, msg.data))))

    def _on_set_locked(self, request, response):
        with self._lock:
            self._lock_flags = protocol.CMD_FLAG_LOCK if request.data else protocol.CMD_FLAG_UNLOCK
            self._lock_repeats = LOCK_REQUEST_REPEATS
            connected = time.monotonic() - self._last_rx < LINK_TIMEOUT
        action = 'lock' if request.data else 'unlock'
        response.success = connected
        response.message = f'{action} sent' if connected else \
            f'{action} queued, but the rover is not connected'
        return response

    # ---------- UDP thread ----------

    def _io_loop(self):
        next_send = time.monotonic()
        while self._running:
            now = time.monotonic()
            link_down = now - self._last_rx > LINK_TIMEOUT
            if now - self._last_resolve > RESOLVE_INTERVAL and (self._addr is None or link_down):
                self._resolve()

            timeout = max(0.0, next_send - time.monotonic())
            readable, _, _ = select.select([self._sock], [], [], timeout)
            if readable:
                try:
                    data, _ = self._sock.recvfrom(512)
                except OSError:
                    data = None
                if data:
                    self._handle_telemetry(data)

            now = time.monotonic()
            if now >= next_send:
                self._send_command(now)
                next_send += self._send_period
                if next_send < now:
                    next_send = now + self._send_period

    def _resolve(self):
        self._last_resolve = time.monotonic()
        try:
            info = socket.getaddrinfo(self._host, self._port, socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as e:
            self.get_logger().warning(f'Cannot resolve {self._host}: {e}',
                                      throttle_duration_sec=30.0)
            return
        addr = info[0][4]
        if addr != self._addr:
            self.get_logger().info(f'Rover address: {addr[0]}:{addr[1]}')
        self._addr = addr

    def _send_command(self, now: float):
        if self._addr is None:
            return
        with self._lock:
            if now - self._cmd_time <= self._cmd_timeout:
                vx, vy, wz = self._cmd
            else:
                vx = vy = wz = 0.0
            flags = 0
            if self._lock_repeats > 0:
                flags = self._lock_flags
                self._lock_repeats -= 1
            grip = self._grip
        self._seq = (self._seq + 1) & 0xFFFF
        # ROS: x forward, y left, z counter-clockwise.
        # RoverC: x right, y forward, z counter-clockwise.
        cmd = protocol.Command(
            seq=self._seq,
            x=_percent(-vy / self._max_linear),
            y=_percent(vx / self._max_linear),
            z=_percent(wz / self._max_angular),
            grip=grip,
            flags=flags,
        )
        self._send(cmd)

    def _send(self, cmd: protocol.Command):
        try:
            self._sock.sendto(protocol.pack_command(cmd), self._addr)
        except OSError as e:
            self.get_logger().warning(f'Sending to the rover failed: {e}',
                                      throttle_duration_sec=10.0)

    def _handle_telemetry(self, data: bytes):
        t = protocol.unpack_telemetry(data)
        if t is None:
            return
        if self._last_tlm_seq is not None:
            gap = (t.seq - self._last_tlm_seq - 1) & 0xFFFF
            lost = gap if gap < 1000 else 0  # a big jump means the rover rebooted
        else:
            lost = 0
        self._last_tlm_seq = t.seq
        with self._lock:
            self._telemetry = t
            self._last_rx = time.monotonic()
            self._rx_count += 1
            self._lost_count += lost

        stamp = self.get_clock().now().to_msg()
        if t.imu_ok:
            self._imu_pub.publish(self._imu_msg(t, stamp))
        if t.locked != self._published_locked:
            self._published_locked = t.locked
            self._locked_pub.publish(Bool(data=t.locked))

    def _imu_msg(self, t: protocol.Telemetry, stamp) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        msg.orientation_covariance[0] = -1.0  # no orientation estimate
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            math.radians(v) for v in t.gyro)
        (msg.linear_acceleration.x, msg.linear_acceleration.y,
         msg.linear_acceleration.z) = (v * STANDARD_GRAVITY for v in t.accel)
        return msg

    # ---------- 1 Hz: battery, temperature, diagnostics ----------

    def _publish_slow(self):
        with self._lock:
            t = self._telemetry
            age = time.monotonic() - self._last_rx
            rx_count, self._rx_count = self._rx_count, 0
            lost_count, self._lost_count = self._lost_count, 0
        stamp = self.get_clock().now().to_msg()
        connected = t is not None and age < LINK_TIMEOUT

        if connected != self._connected:
            self._connected = connected
            if connected:
                self.get_logger().info('Rover connected')
            else:
                self.get_logger().warning('Lost the rover (no telemetry)')

        if connected:
            self._battery_pub.publish(self._battery_msg(t, stamp))
            if math.isfinite(t.imu_temp):
                temp = Temperature(temperature=float(t.imu_temp), variance=0.0)
                temp.header.stamp = stamp
                temp.header.frame_id = self._imu_frame
                self._temp_pub.publish(temp)

        diag = DiagnosticArray()
        diag.header.stamp = stamp
        diag.status.append(self._link_status(t, age, connected, rx_count, lost_count))
        self._diag_pub.publish(diag)

    def _battery_msg(self, t: protocol.Telemetry, stamp) -> BatteryState:
        nan = math.nan
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = self._battery_frame
        msg.voltage = t.batt_mv / 1000.0 if t.batt_mv > 0 else nan
        msg.current = t.batt_ma / 1000.0 if t.batt_ma != 0 else nan
        msg.percentage = t.batt_pct / 100.0 if 0 <= t.batt_pct <= 100 else nan
        msg.temperature = nan
        msg.charge = nan
        msg.capacity = nan
        msg.design_capacity = nan
        msg.power_supply_status = _CHARGE_TO_ROS.get(
            t.charge_state, BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = t.batt_mv > 0
        msg.location = 'm5stickc'
        return msg

    def _link_status(self, t, age, connected, rx_count, lost_count) -> DiagnosticStatus:
        status = DiagnosticStatus(name=LINK_STATUS_NAME, hardware_id=self._host)
        if not connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'no telemetry from the rover'
            if self._addr is None:
                status.message += f' (cannot resolve {self._host})'
        elif not t.rover_found:
            status.level = DiagnosticStatus.ERROR
            status.message = 'RoverC base not detected - check the stick is seated'
        elif t.locked:
            status.level = DiagnosticStatus.WARN
            status.message = 'locked (button A on the stick, or /rover/set_locked)'
        elif age > 0.5:
            status.level = DiagnosticStatus.WARN
            status.message = 'telemetry delayed'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'connected'

        values = {
            'rover_address': self._addr[0] if self._addr else '',
            'telemetry_hz': rx_count,
            'telemetry_lost': lost_count,
        }
        if t is not None:
            values.update({
                'rssi_dbm': t.rssi,
                'locked': t.locked,
                'rover_found': t.rover_found,
                'imu_ok': t.imu_ok,
                'moving': t.moving,
                'uptime_s': t.uptime_ms // 1000,
                'battery_mv': t.batt_mv,
                'battery_pct': t.batt_pct,
                'vbus_mv': t.vbus_mv,
                'grip_deg': t.grip,
            })
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return status

    # ---------- Shutdown ----------

    def close(self):
        self._running = False
        self._io_thread.join(timeout=1.0)
        if self._addr is not None:
            # Stop now rather than waiting for the rover's 0.5 s timeout.
            self._send(protocol.Command(seq=(self._seq + 1) & 0xFFFF, x=0, y=0, z=0))
        self._sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
