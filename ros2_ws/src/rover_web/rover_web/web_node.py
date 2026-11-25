"""Browser control page for the rover, served from ROS 2.

Open http://<this PC>:8080 from any device on the same network. Holding a
button publishes /cmd_vel. The page also shows the rover's battery, IMU and
link status from rover_driver's topics.

Publishes   /cmd_vel (geometry_msgs/Twist), /rover/gripper (std_msgs/Float32)
Subscribes  /rover/imu, /rover/battery, /rover/locked, /diagnostics
Calls       /rover/set_locked (std_srvs/SetBool)

HTTP endpoints (used by static/index.html)
  GET /                       the page
  GET /drive?vx=&vy=&wz=      fractions -1..1 of the max speeds; all 0 = stop
  GET /grip?angle=            gripper angle in degrees
  GET /lock?on=1|0            lock or unlock the rover
  GET /status                 JSON snapshot of the rover's state
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import socket
import threading
import time
from urllib.parse import parse_qs, urlparse

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rover_driver.driver_node import LINK_STATUS_NAME
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

IMU_FRESH = 1.0        # s
STATUS_FRESH = 3.0     # s; the driver publishes battery and diagnostics at 1 Hz
VIEWER_TIMEOUT = 3.0   # s; the page polls /status every 1 s

_LEVELS = {
    DiagnosticStatus.OK: 'ok',
    DiagnosticStatus.WARN: 'warn',
    DiagnosticStatus.ERROR: 'error',
    DiagnosticStatus.STALE: 'error',
}


def _num(value, digits=3):
    """JSON-safe float: None for NaN/inf."""
    return round(value, digits) if math.isfinite(value) else None


def _clamp(value):
    return max(-1.0, min(1.0, value))


class RoverWeb(Node):

    def __init__(self):
        super().__init__('rover_web')
        port = self.declare_parameter('port', 8080).value
        self._max_linear = self.declare_parameter('max_linear_speed', 0.3).value
        self._max_angular = self.declare_parameter('max_angular_speed', 2.0).value
        self._drive_timeout = self.declare_parameter('drive_timeout', 0.5).value
        self._stamped = self.declare_parameter('cmd_vel_stamped', False).value

        page = os.path.join(get_package_share_directory('rover_web'), 'static', 'index.html')
        with open(page, 'rb') as f:
            self.page = f.read()

        # Written by ROS callbacks and HTTP threads.
        self._lock = threading.Lock()
        self._driving = False
        self._last_drive = 0.0
        self._imu = None
        self._imu_time = -math.inf
        self._battery = None
        self._battery_time = -math.inf
        self._locked = None
        self._link = None
        self._link_time = -math.inf
        self._viewers = {}

        cmd_type = TwistStamped if self._stamped else Twist
        self._cmd_pub = self.create_publisher(cmd_type, 'cmd_vel', 10)
        self._grip_pub = self.create_publisher(Float32, 'rover/gripper', 10)
        self._lock_client = self.create_client(SetBool, 'rover/set_locked')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Imu, 'rover/imu', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'rover/battery', self._on_battery, 10)
        self.create_subscription(Bool, 'rover/locked', self._on_locked, latched)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10)
        self.create_timer(0.1, self._check_drive_timeout)

        self._server = ThreadingHTTPServer(('0.0.0.0', port), _make_handler(self))
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info(
            f'Control page: http://localhost:{port} (or http://{socket.gethostname()}.local:{port}'
            ' from another device)')

    # ---------- From the page ----------

    def drive(self, vx: float, vy: float, wz: float):
        twist = Twist()
        twist.linear.x = _clamp(vx) * self._max_linear
        twist.linear.y = _clamp(vy) * self._max_linear
        twist.angular.z = _clamp(wz) * self._max_angular
        self._publish_twist(twist)
        with self._lock:
            self._driving = (vx, vy, wz) != (0.0, 0.0, 0.0)
            self._last_drive = time.monotonic()

    def grip(self, angle: float):
        self._grip_pub.publish(Float32(data=float(angle)))

    def request_lock(self, on: bool) -> tuple[bool, str]:
        if not self._lock_client.service_is_ready():
            return False, 'rover_driver is not running'
        self._lock_client.call_async(SetBool.Request(data=on))
        return True, 'ok'

    def status(self, viewer: str) -> dict:
        now = time.monotonic()
        with self._lock:
            self._viewers[viewer] = now
            self._viewers = {ip: t for ip, t in self._viewers.items()
                             if now - t < VIEWER_TIMEOUT}
            imu = self._imu if now - self._imu_time < IMU_FRESH else None
            battery = self._battery if now - self._battery_time < STATUS_FRESH else None
            link = self._link if now - self._link_time < STATUS_FRESH else None
            locked = self._locked
            viewers = len(self._viewers)

        result = {'link': link, 'locked': locked, 'viewers': viewers,
                  'battery': None, 'imu': None}
        if battery is not None:
            result['battery'] = {
                'voltage': _num(battery.voltage, 2),
                'percent': _num(battery.percentage, 2),
                'charging': battery.power_supply_status ==
                BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            }
        if imu is not None:
            a, g = imu.linear_acceleration, imu.angular_velocity
            result['imu'] = {
                'accel': [_num(a.x, 2), _num(a.y, 2), _num(a.z, 2)],
                'gyro': [_num(g.x, 3), _num(g.y, 3), _num(g.z, 3)],
            }
        return result

    # ---------- ROS callbacks ----------

    def _publish_twist(self, twist: Twist):
        if self._stamped:
            msg = TwistStamped(twist=twist)
            msg.header.stamp = self.get_clock().now().to_msg()
            self._cmd_pub.publish(msg)
        else:
            self._cmd_pub.publish(twist)

    def _check_drive_timeout(self):
        # The page resends every 150 ms while a button is held. If it stops
        # (tab closed, phone locked), send one stop and then stay quiet so other
        # /cmd_vel publishers aren't overridden.
        with self._lock:
            timed_out = self._driving and time.monotonic() - self._last_drive > self._drive_timeout
            if timed_out:
                self._driving = False
        if timed_out:
            self._publish_twist(Twist())

    def _on_imu(self, msg: Imu):
        with self._lock:
            self._imu, self._imu_time = msg, time.monotonic()

    def _on_battery(self, msg: BatteryState):
        with self._lock:
            self._battery, self._battery_time = msg, time.monotonic()

    def _on_locked(self, msg: Bool):
        with self._lock:
            self._locked = msg.data

    def _on_diagnostics(self, msg: DiagnosticArray):
        for s in msg.status:
            if s.name != LINK_STATUS_NAME:
                continue
            values = {kv.key: kv.value for kv in s.values}
            rssi = int(values.get('rssi_dbm', '0') or 0)
            link = {
                'level': _LEVELS.get(s.level, 'error'),
                'message': s.message,
                'rssi': rssi if rssi != 0 else None,
            }
            with self._lock:
                self._link, self._link_time = link, time.monotonic()

    def close(self):
        self._server.shutdown()
        self._server.server_close()


def _make_handler(node: RoverWeb):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'  # keep-alive: the page sends a request every 150 ms

        def do_GET(self):
            url = urlparse(self.path)
            query = parse_qs(url.query)

            def number(name):
                try:
                    value = float(query.get(name, ['0'])[0])
                except ValueError:
                    return 0.0
                return value if math.isfinite(value) else 0.0

            if url.path == '/':
                self._send(200, node.page, 'text/html; charset=utf-8')
            elif url.path == '/drive':
                node.drive(number('vx'), number('vy'), number('wz'))
                self._send(200, b'ok')
            elif url.path == '/grip':
                node.grip(number('angle'))
                self._send(200, b'ok')
            elif url.path == '/lock':
                ok, message = node.request_lock(query.get('on', ['1'])[0] == '1')
                self._send(200 if ok else 503, message.encode())
            elif url.path == '/status':
                body = json.dumps(node.status(self.client_address[0])).encode()
                self._send(200, body, 'application/json')
            else:
                self._send(404, b'not found')

        def _send(self, code, body, content_type='text/plain; charset=utf-8'):
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):
            pass  # one line per request at 7 Hz is too noisy

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = RoverWeb()
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
