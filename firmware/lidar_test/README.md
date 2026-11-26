# LD06 lidar test

A standalone test for an LDROBOT LD06 lidar wired to the M5StickC. You don't need LDROBOT's
USB debug/speed-control board, because the stick takes its place. The rover firmware isn't
involved, but flashing this replaces it on the stick, so reflash `firmware/rover` when you're
done.

## Wiring

Wire the LD06 to the stick's own Grove port (GND / 5V / G32 / G33), not to the RoverC's
I2C Grove ports or its servo ports.

| LD06 pin | Stick Grove |
|---|---|
| 1 Tx | G33 (white) |
| 2 PWM | not connected |
| 3 GND | GND (black) |
| 4 P5V | 5V (red) |

- **Pin 1:** find it from the marking on the LD06's socket, not from wire colours. Swapping
  5V and GND can kill it.
- **PWM:** leave it unconnected. The LD06 then holds its own speed of about 10 Hz.
- **Signal level:** Tx is 3.3 V, so it goes straight to G33 with no level shifter.
- **Power:** the LD06 needs 4.5–5.5 V, about 300 mA at start-up and 180 mA running. The
  stick's Grove 5V supplies this when the stick is on USB or the RoverC is switched on. Never
  use the 3V3 pin.
- **Connectors:** the LD06 has a 1.5 mm ZH socket and Grove is 2.0 mm, so you need an adapter
  lead or a spliced cable.

The servo ports won't work for the data. Their signal pin is a PWM output from the RoverC's
STM32, which only takes motor and servo commands over I2C, so it can't receive the lidar's
serial data.

## Test

1. **Spin test:** power it. The rotor should spin up within a second or two and then turn
   steadily.
2. **Flash the test:** run `pio run -t upload` in this folder. The screen shows:
   - `LD06 OK`, about 10 Hz scan rate, about 375 packets/s, 0 CRC errors, and the distance
     straight ahead. Put a box at a measured distance in front of the lidar to check that
     reading.
   - `NO DATA` if no bytes arrive. Check power, GND and Tx to G33.
   - `BAD DATA` if bytes arrive but no packet passes its checksum.
3. **View it on the PC:** the stick also passes the lidar's data through to its USB port at
   230400 baud.

   ```sh
   python3 ld06_view.py              # live plot, front at the top
   python3 ld06_view.py --text       # one line per revolution
   python3 ld06_view.py --selftest   # check the parser without hardware
   ```

   It picks the first `/dev/ttyUSB*` or `/dev/ttyACM*`, or you can pass the port. For a raw
   look at the bytes, run `stty -F /dev/ttyUSB0 230400 raw && xxd /dev/ttyUSB0 | head`. You
   should see `54 2c` repeating.
4. **Restore the rover:** run `cd ../rover && pio run -t upload`.
