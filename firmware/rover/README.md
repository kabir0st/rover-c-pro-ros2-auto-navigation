# RoverC Pro firmware (browser control)

Drive an M5Stack RoverC Pro from a web browser. Runs on an M5StickC, StickC Plus or
StickC Plus2. It uses M5's own libraries: M5Unified for the stick and M5-RoverC for the
rover.

## WiFi

Edit `include/secrets.h` before flashing:

- **Hotspot (default):** leave `WIFI_SSID` empty. The rover creates a WiFi network
  called `RoverC` with the password `roverc123`. Join it from your phone or laptop and
  open `http://192.168.4.1`.
- **Your own WiFi:** set `WIFI_SSID` and `WIFI_PASS`. The stick's screen shows its IP,
  or you can open `http://roverc.local`. If it can't join within 15 s, it starts the
  hotspot instead.

## Build and flash

```sh
pio run                 # build
pio run -t upload       # flash (stick on USB)
pio device monitor      # serial log, prints the URL
```

On Linux the serial port needs permission once. Either install PlatformIO's udev rules:

```sh
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

or add yourself to the `dialout` group with `sudo usermod -aG dialout $USER`, then log
out and back in.

## Using it

- **Driving:** hold an arrow to move, and release to stop. The two round arrows turn on
  the spot. The slider sets the speed.
- **Gripper:** set its position with the gripper slider (10–90°).
- **Lock:** button A (the big front button on the stick) stops the rover and ignores the
  browser until you press it again.
- **Auto-stop:** the rover stops by itself if it gets no command for 0.5 s, for example
  when the tab closes or WiFi drops.

**First test with the wheels off the ground.** If a direction is mirrored, flip the sign
of that button's `data-x`, `data-y` or `data-z` in `src/page.h`.
