# ROS 2 workspace (Lyrical)

There are two packages:

- **`rover_driver`:** talks UDP to the rover firmware (`firmware/rover`). It turns
  `/cmd_vel` into motor commands and publishes the rover's IMU, battery and link status.
- **`rover_web`:** a browser control page. It publishes `/cmd_vel` and shows the
  rover's topics.

## Build

```sh
cd ros2_ws
source /opt/ros/lyrical/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Run

To start the driver and the control page:

```sh
ros2 launch rover_web web.launch.py
```

Then open `http://localhost:8080`, or `http://<this PC>:8080` from a phone on the same
WiFi. Hold an arrow to drive and release to stop.

You can also start the pieces separately:

```sh
ros2 launch rover_driver rover.launch.py                        # driver only
ros2 launch rover_driver rover.launch.py rover_host:=192.168.0.33   # if roverc.local doesn't resolve
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # keyboard; shift + keys strafes
```

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | x forward, y left, z counter-clockwise. Use `cmd_vel_stamped:=true` for `TwistStamped`. |
| `/rover/gripper` | `std_msgs/Float32` | Gripper servo angle, 10–90°. |
| `/rover/imu` | `sensor_msgs/Imu` | 50 Hz. Angular velocity in rad/s and acceleration in m/s², in the stick's axes (`imu_link`). No orientation. |
| `/rover/imu/temperature` | `sensor_msgs/Temperature` | 1 Hz. The IMU chip's own temperature, not the room's. |
| `/rover/battery` | `sensor_msgs/BatteryState` | 1 Hz, the stick's battery. NaN where the board can't measure something. |
| `/rover/locked` | `std_msgs/Bool` | Latched. True while button A or `/rover/set_locked` has locked it. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz. Status `rover_driver: link`: connection, RSSI, telemetry rate and loss, flags. |

The service `/rover/set_locked` (`std_srvs/SetBool`) locks or unlocks the rover.

## Speed scaling

The rover has no wheel encoders. `max_linear_speed` (0.3 m/s) and `max_angular_speed`
(2.0 rad/s) are the speeds assumed at 100% motor power. Measure your rover and set them
in both `rover_driver/config/driver.yaml` and `rover_web/config/web.yaml`.

## Testing without the rover

```sh
ros2 run rover_driver fake_rover                                # pretends to be the rover
ros2 launch rover_web web.launch.py rover_host:=127.0.0.1
python3 -m pytest src/rover_driver/test                         # protocol tests
```
