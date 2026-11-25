# RoverC Pro firmware (ROS 2 relay)

Firmware for the M5StickC, StickC Plus or StickC Plus2 that sits on the RoverC Pro. It's
a thin relay: the ROS 2 driver in `ros2_ws/src/rover_driver` sends drive commands over
UDP, and the stick sends back IMU and battery readings. The browser control page now
lives in ROS 2 (`rover_web`).

It uses M5's own libraries: M5Unified for the stick and M5-RoverC for the rover.

## WiFi

Set your network in `include/secrets.h` (git-ignored; `secrets.example.h` is the template):

- **Your own WiFi:** set `WIFI_SSID` and `WIFI_PASS`.
- **Hotspot:** if `WIFI_SSID` is empty or the stick can't join within 15 s, it creates a
  network called `RoverC` (password `roverc123`). The rover is then `192.168.4.1`. Join
  it from the ROS PC.

Either way the rover is reachable as `roverc.local`.

## Build and flash

```sh
pio run                 # build
pio run -t upload       # flash (stick on USB)
pio device monitor      # serial log
```

## Screen

- **First line:**
  - `READY`, `DRIVING`, `LOCKED (btn A)` or `RoverC NOT FOUND`.
- **Your own WiFi:**
  - The network name and the rover's IP.
  - The battery.
  - `ROS <ip>` and the command rate, or `ROS: waiting`.
- **Hotspot:**
  - The hotspot name and IP.
  - The battery.
  - Every joined device, with the ROS PC tagged `ros`.

## Safety

- **Auto-stop:** the rover stops if no command arrives for 0.5 s. The ROS driver sends
  commands at a steady 20 Hz, even when they're all zero.
- **Lock:** button A locks the rover, which stops it and ignores commands until you
  press A again. ROS can do the same with `/rover/set_locked`.

## Protocol

The packet layouts are in `include/protocol.h`, mirrored by
`ros2_ws/src/rover_driver/rover_driver/protocol.py`. Both use UDP port 4210 and
little-endian packed structs.

- **Command (PC → rover, 10 bytes, 20 Hz):**
  - Speeds `x`/`y`/`z` from −100 to 100, on `M5_RoverC::setSpeed`'s axes.
  - The gripper angle.
  - Lock and unlock flags.
- **Telemetry (rover → PC, 50 bytes, 50 Hz):**
  - Accelerometer (g) and gyro (deg/s).
  - IMU temperature.
  - Battery voltage, current, percentage and charge state.
  - 5 V input voltage and WiFi RSSI.
  - Lock, rover-found and moving flags.

On a Plus2 the board can't measure battery current, charge state or the 5 V input, so
those are sent as "unknown".
