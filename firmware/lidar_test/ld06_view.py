#!/usr/bin/env python3
"""Live view of an LD06 lidar on a serial port.

The port is either the M5StickC running firmware/lidar_test (it bridges the lidar
to USB) or any 3.3 V USB-serial adapter wired to the LD06's Tx pin.

    python3 ld06_view.py                      # first /dev/ttyUSB* or /dev/ttyACM*, live plot
    python3 ld06_view.py /dev/ttyACM0 --text  # one line per revolution, no plot
    python3 ld06_view.py --selftest           # check the parser without hardware

Needs pyserial, and matplotlib for the plot.
"""

import argparse
import glob
import math
import struct
import sys
import time

BAUD = 230400
HEADER = b'\x54\x2c'  # header byte, then VerLen: packet type 1, 12 points
POINTS = 12
PACKET_SIZE = 47
# header, verlen, speed (deg/s), start angle (0.01 deg), 12 x (distance mm, intensity),
# end angle (0.01 deg), timestamp (ms), crc8
_PACKET = struct.Struct('<BBHH' + 'HB' * POINTS + 'HHB')
assert _PACKET.size == PACKET_SIZE


def _crc_table():
    """CRC-8, polynomial 0x4D, initial value 0, MSB first: the LD06's checksum."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


_CRC_TABLE = _crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


def decode_packet(raw: bytes):
    """One checked packet -> (speed in deg/s, [(angle deg, distance mm, intensity), ...]).

    Angles go clockwise from the sensor's front, as the LD06 reports them.
    """
    f = _PACKET.unpack(raw)
    speed, start, end = f[2], f[3], f[4 + 2 * POINTS]
    span = (end - start) % 36000
    points = []
    for i in range(POINTS):
        angle = (start + span * i / (POINTS - 1)) % 36000 / 100.0
        points.append((angle, f[4 + 2 * i], f[5 + 2 * i]))
    return speed, points


def encode_packet(speed: int, start: int, end: int, distances, intensities, timestamp=0) -> bytes:
    """Build a packet like the LD06 sends. Angles in 0.01 deg. For tests."""
    fields = [0x54, 0x2C, speed, start]
    for d, q in zip(distances, intensities):
        fields += [d, q]
    fields += [end, timestamp, 0]
    raw = bytearray(_PACKET.pack(*fields))
    raw[-1] = crc8(raw[:-1])
    return bytes(raw)


class Parser:
    """Turns the serial byte stream into packets, and packets into full revolutions."""

    def __init__(self):
        self.packets = 0
        self.crc_errors = 0
        self.speed = 0  # deg/s, from the last packet
        self._buf = bytearray()
        self._points = []
        self._last_angle = None
        self._first = True  # the first revolution is partial; drop it

    def feed(self, data: bytes) -> list:
        """Add bytes. Returns the revolutions completed by them, each a list of points."""
        self._buf += data
        revolutions = []
        while True:
            start = self._buf.find(HEADER)
            if start < 0:
                keep = 1 if self._buf.endswith(HEADER[:1]) else 0  # header split across reads
                del self._buf[:len(self._buf) - keep]
                break
            del self._buf[:start]
            if len(self._buf) < PACKET_SIZE:
                break
            raw = bytes(self._buf[:PACKET_SIZE])
            if crc8(raw[:-1]) != raw[-1]:
                self.crc_errors += 1
                del self._buf[:1]  # not a real packet start: look further on
                continue
            del self._buf[:PACKET_SIZE]
            self.packets += 1
            self.speed, points = decode_packet(raw)
            revolutions += self._add_points(points)
        return revolutions

    def _add_points(self, points) -> list:
        done = []
        for p in points:
            if self._last_angle is not None and p[0] < self._last_angle - 180:  # wrapped past 360
                if not self._first:
                    done.append(self._points)
                self._first = False
                self._points = []
            self._points.append(p)
            self._last_angle = p[0]
        return done


def summary(rev, parser: Parser) -> str:
    valid = [p for p in rev if p[1] > 0]
    line = f'{parser.speed / 360:5.1f} Hz  {len(rev):3d} points  {len(valid):3d} valid'
    if valid:
        front = min(valid, key=lambda p: min(p[0], 360 - p[0]))
        near = min(valid, key=lambda p: p[1])
        line += (f'  front {front[1]:5d} mm @ {front[0]:5.1f} deg'
                 f'  nearest {near[1]:5d} mm @ {near[0]:5.1f} deg')
    return line + f'  crc errors {parser.crc_errors}'


def open_port(port: str):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = BAUD
    ser.timeout = 0.05
    # Leave DTR/RTS released: on the M5StickC they are wired to reset and boot mode.
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def find_port() -> str:
    ports = sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))
    if not ports:
        sys.exit('No serial port found. Plug in the stick, or pass the port name.')
    return ports[0]


def run_text(ser, parser: Parser):
    last_data = time.monotonic()
    warned = False
    while True:
        data = ser.read(ser.in_waiting or 1)
        now = time.monotonic()
        if data:
            last_data, warned = now, False
        elif now - last_data > 2.0 and not warned:
            print(f'No data from {ser.port}. Check the LD06 power and its Tx wire.', flush=True)
            warned = True
        for rev in parser.feed(data):
            print(summary(rev, parser), flush=True)


def run_plot(ser, parser: Parser, max_range: float):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection='polar')
    ax.set_theta_zero_location('N')  # sensor front at the top
    ax.set_theta_direction(-1)       # LD06 angles go clockwise
    ax.set_xticks([math.radians(a) for a in range(0, 360, 45)])
    ax.set_xticklabels(['front', '45', '90', '135', 'back', '225', '270', '315'])
    dots = ax.scatter([], [], s=4, c=[], cmap='viridis', vmin=0, vmax=255)
    fig.colorbar(dots, ax=ax, shrink=0.6, label='intensity')
    title = ax.set_title(f'Waiting for data on {ser.port}')
    state = {'last_data': time.monotonic()}

    def update(_frame):
        data = ser.read(ser.in_waiting or 1)
        now = time.monotonic()
        if data:
            state['last_data'] = now
        revs = parser.feed(data)
        if not revs:
            if now - state['last_data'] > 2.0:
                title.set_text(f'No data from {ser.port}: check LD06 power and Tx')
            return dots, title
        rev = [p for p in revs[-1] if p[1] > 0]
        if rev:
            theta = [math.radians(p[0]) for p in rev]
            r = [p[1] / 1000.0 for p in rev]
            dots.set_offsets(list(zip(theta, r)))
            dots.set_array([p[2] for p in rev])
            ax.set_rmax(max_range or max(1.0, max(r) * 1.05))
        title.set_text(f'{parser.speed / 360:.1f} Hz, {len(revs[-1])} points '
                       f'({len(rev)} valid), {parser.crc_errors} CRC errors  [m]')
        return dots, title

    # Keep a reference, or the animation is garbage-collected and stops.
    fig._ld06_animation = FuncAnimation(fig, update, interval=30, cache_frame_data=False)
    plt.show()


def selftest():
    distances = list(range(1000, 1000 + POINTS))
    raw = encode_packet(3600, 35500, 500, distances, [200] * POINTS)  # 355 deg -> 5 deg
    assert raw[-1] == crc8(raw[:-1])
    speed, points = decode_packet(raw)
    assert speed == 3600
    assert [p[1] for p in points] == distances
    assert abs(points[0][0] - 355.0) < 1e-9 and abs(points[-1][0] - 5.0) < 1e-9
    assert abs(points[1][0] - (355.0 + 10 / 11)) < 1e-9

    # One flipped byte is rejected; garbage before and between packets is skipped.
    bad = bytearray(raw)
    bad[10] ^= 0x01
    p = Parser()
    p.feed(b'\x00\x54\x13garbage' + bytes(bad) + raw[:20])
    p.feed(raw[20:] + raw)
    assert p.crc_errors == 1 and p.packets == 2, (p.crc_errors, p.packets)

    # Three revolutions of 1 deg steps: the first is dropped (partial), the second comes
    # back whole when the third starts.
    stream = b''.join(
        encode_packet(3600, k * 1200, k * 1200 + 1100, [500] * POINTS, [100] * POINTS)
        for _ in range(3) for k in range(30))
    p = Parser()
    revs = []
    for i in range(0, len(stream), 61):  # odd chunk size: packets split across reads
        revs += p.feed(stream[i:i + 61])
    assert p.crc_errors == 0 and p.packets == 90
    assert len(revs) == 1 and len(revs[0]) == 360, [len(r) for r in revs]
    assert [round(a) for a, _, _ in revs[0]] == list(range(360))
    print('selftest OK')


def main():
    ap = argparse.ArgumentParser(description='Live view of an LD06 lidar on a serial port.')
    ap.add_argument('port', nargs='?', help='serial port (default: first ttyUSB/ttyACM)')
    ap.add_argument('--text', action='store_true', help='print one line per revolution')
    ap.add_argument('--range', type=float, default=0.0,
                    help='plot radius in metres (default: fit each scan)')
    ap.add_argument('--selftest', action='store_true', help='test the parser and exit')
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    port = args.port or find_port()
    ser = open_port(port)
    print(f'Reading LD06 data on {port} at {BAUD} baud. Ctrl-C to stop.', flush=True)
    parser = Parser()
    try:
        if args.text:
            run_text(ser, parser)
        else:
            run_plot(ser, parser, args.range)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
