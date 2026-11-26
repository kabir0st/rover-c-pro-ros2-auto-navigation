# ROS 2 workspace (Lyrical)

There are three packages:

- **`rover_driver`:** talks UDP to the rover firmware (`firmware/rover`). It turns
  `/cmd_vel` into motor commands and publishes the rover's IMU, battery and link status.
- **`rover_slam`:** dead-reckoning localization and a simple 2D map, plus a script that
  drives a rectangle.
- **`rover_web`:** a browser control page with a live map. It publishes `/cmd_vel` and
  shows the rover's topics.

## Build

```sh
cd ros2_ws
source /opt/ros/lyrical/setup.zsh
colcon build --symlink-install
source install/setup.zsh
```

## Run

To start everything (driver, SLAM and the control page):

```sh
ros2 launch rover_web web.launch.py
```

Then open `http://localhost:8080`, or `http://<this PC>:8080` from a phone on the same
WiFi. Hold an arrow to drive and release to stop. The map on the right shows a 5 cm
grid, with bold lines every metre.

You can also start the pieces separately:

```sh
ros2 launch rover_driver rover.launch.py                        # driver only
ros2 launch rover_driver rover.launch.py rover_host:=192.168.0.33   # if roverc.local doesn't resolve
ros2 launch rover_slam slam.launch.py                           # SLAM only
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # keyboard; shift + keys strafes
```

## Driving a rectangle

With the stack running, clear a space and run:

```sh
ros2 run rover_slam drive_rectangle                              # 1.0 x 0.5 m, turning left at corners
ros2 run rover_slam drive_rectangle --width 0.6 --height 0.6 --strafe   # slide sideways, never turn
ros2 run rover_slam drive_rectangle --clockwise --laps 2 --speed 0.1
```

It starts wherever the rover is, with the first side straight ahead. Ctrl-C, Button A
or the page's Lock button stops it. At the end it prints how far from the start it
thinks it ended up. Compare that with what really happened.

## How the SLAM works, and its limits

The rover has no wheel encoders and no distance sensor, so this is dead reckoning, not
full SLAM:

- **Position:** comes from the velocity the rover was told to drive
  (`/rover/cmd_applied`). The driver works this out from the motor commands, including
  the RoverC's wheel mixing and clipping, and it's zero while the rover is locked or
  disconnected.
- **Heading:** comes from the IMU gyro. A small Kalman filter uses the command as its
  prediction. While the rover is told to stand still, it learns the gyro's bias. While
  it moves, it follows the bias-corrected gyro. It finds "up" from gravity, so it
  doesn't matter how the stick is mounted. Because of this, corners and straight lines
  are good.
- **Map:**
  - Cells the rover's footprint passes over are marked free.
  - A hard jolt while driving steadily counts as a bump, and marks the cells just ahead
    as occupied. This is experimental: tune `bump_threshold`.
  - Everything else stays unknown. There's no sensor to see walls before hitting them.
- **Drift:** nothing corrects position, so error grows with distance. That includes
  wheel slip, pushing the rover by hand, or driving into a wall without a detected
  bump. Press **Reset map** on the page, or call `/rover/reset_map`, when the rover is
  back at its start.

I didn't use existing ROS packages for this:

- `slam_toolbox` and cartographer need a laser range scanner.
- `robot_localization` is a general EKF. With only commands and one gyro it adds
  nothing, and it can't learn the gyro bias, which is the main source of heading
  drift here.

For real mapping, add a range sensor (for example a ToF sensor on the RoverC's Grove
port) or use the ESP32-CAM.

## Calibrating distance

Odometry trusts the commanded speed, so the distance it reports depends on how fast
the motors really are:

1. Run `drive_rectangle --width 1.0 --height 1.0`.
2. Measure the real length of a side.
3. In `rover_slam/config/slam.yaml`, set `linear_scale` to its current value × the
   real length ÷ 1.0.
4. Do the same for `lateral_scale` with `--strafe`. Strafing usually covers less
   ground.
5. Rebuild, or just restart if you built with `--symlink-install`.

## Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | x forward, y left, z counter-clockwise. Use `cmd_vel_stamped:=true` for `TwistStamped`. |
| `/rover/gripper` | `std_msgs/Float32` | Gripper servo angle, 10–90°. |
| `/rover/imu` | `sensor_msgs/Imu` | 50 Hz. Angular velocity in rad/s and acceleration in m/s², in the stick's axes (`imu_link`). No orientation. |
| `/rover/imu/temperature` | `sensor_msgs/Temperature` | 1 Hz. The IMU chip's own temperature, not the room's. |
| `/rover/battery` | `sensor_msgs/BatteryState` | 1 Hz, the stick's battery. NaN where the board can't measure something. |
| `/rover/locked` | `std_msgs/Bool` | Latched. True while button A or `/rover/set_locked` has locked it. |
| `/rover/cmd_applied` | `geometry_msgs/TwistStamped` | 20 Hz. The velocity the rover was actually told to drive. |
| `/odom` | `nav_msgs/Odometry` | 50 Hz, plus TF `odom` → `base_link`. Dead-reckoning pose. |
| `/map` | `nav_msgs/OccupancyGrid` | Latched. 5 cm cells, 10 × 10 m, centred on the start. |
| `/rover/path` | `nav_msgs/Path` | The path driven since the last reset. |
| `/rover/footprint` | `geometry_msgs/PolygonStamped` | Latched. The rover outline used for the map. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `rover_driver: link` covers connection, RSSI and telemetry rate. `rover_slam: odometry` covers pose, gyro bias, drift estimate and bumps. |

Services:

- `/rover/set_locked` (`std_srvs/SetBool`) locks or unlocks the rover.
- `/rover/reset_map` (`std_srvs/Trigger`) clears the map and puts the pose back to zero.

## Speed scaling

`max_linear_speed` (0.3 m/s) and `max_angular_speed` (2.0 rad/s) are the speeds
assumed at 100% motor power. They map `/cmd_vel` to motor power. Keep them the same in
`rover_driver/config/driver.yaml` and `rover_web/config/web.yaml`. Odometry
calibration is separate (see above).

## Testing without the rover

```sh
ros2 run rover_driver fake_rover                                # pretends to be the rover
ros2 launch rover_web web.launch.py rover_host:=127.0.0.1
python3 -m pytest src/rover_driver/test src/rover_slam/test      # unit tests
```

The fake rover has a tilted IMU and a gyro bias, like the real one. It moves exactly as
commanded, though, so it tests the code, not the calibration.
