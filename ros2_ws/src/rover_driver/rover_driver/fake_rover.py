"""Pretend to be the rover, for testing the ROS side without hardware.

Listens on UDP 4210 like the firmware, prints the commands it gets, and sends
telemetry back at 50 Hz: an IMU that turns when told to (mounted slightly
tilted, with a small gyro bias and noise like a real one), and a battery that
slowly drains.

    ros2 run rover_driver fake_rover
    ros2 launch rover_driver rover.launch.py rover_host:=127.0.0.1
"""

import argparse
import math
import random
import socket
import time

from rover_driver import protocol

COMMAND_TIMEOUT = 0.5
PEER_TIMEOUT = 2.0
TELEMETRY_PERIOD = 0.02

MAX_TURN_DEG_S = math.degrees(2.0)  # turn rate at 100 %, matches rover_driver's default
GYRO_BIAS_DEG_S = (0.3, -0.2, 0.6)
GYRO_NOISE_DEG_S = 0.2
ACCEL_NOISE_G = 0.005
_tilt = (0.0, -0.03, 1.0)
UP = tuple(v / math.sqrt(sum(c * c for c in _tilt)) for v in _tilt)  # the stick's "up" axis


def main():
    parser = argparse.ArgumentParser(description='Fake RoverC for testing rover_driver.')
    parser.add_argument('--port', type=int, default=protocol.ROVER_UDP_PORT)
    args, _ = parser.parse_known_args()  # ignore --ros-args when started from a launch file

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.settimeout(0.002)
    print(f'Fake rover listening on UDP {args.port}', flush=True)

    start = time.monotonic()
    peer = None
    last_cmd_time = -1e9
    last_tx = 0.0
    seq = 0
    cmd_seq = 0
    locked = False
    grip = 0
    speeds = (0, 0, 0)
    shown = None

    try:
        while True:
            try:
                data, src = sock.recvfrom(64)
                cmd = protocol.unpack_command(data)
            except socket.timeout:
                cmd = None
            now = time.monotonic()

            if cmd is not None:
                peer, last_cmd_time, cmd_seq = src, now, cmd.seq
                if cmd.flags & protocol.CMD_FLAG_LOCK:
                    locked = True
                if cmd.flags & protocol.CMD_FLAG_UNLOCK:
                    locked = False
                if not locked and cmd.grip:
                    grip = cmd.grip
                speeds = (0, 0, 0) if locked else (cmd.x, cmd.y, cmd.z)

            if now - last_cmd_time > COMMAND_TIMEOUT:
                speeds = (0, 0, 0)

            state = (speeds, grip, locked)
            if state != shown:
                shown = state
                x, y, z = speeds
                print(f'x={x:4d} y={y:4d} z={z:4d}  grip={grip}  locked={locked}', flush=True)

            if peer and now - last_cmd_time < PEER_TIMEOUT and now - last_tx >= TELEMETRY_PERIOD:
                last_tx = now
                seq += 1
                elapsed = now - start
                moving = speeds != (0, 0, 0)
                flags = protocol.TLM_FLAG_ROVER_FOUND | protocol.TLM_FLAG_IMU_OK
                flags |= protocol.TLM_FLAG_LOCKED if locked else 0
                flags |= protocol.TLM_FLAG_MOVING if moving else 0
                turn = protocol.effective_speeds(*speeds)[2] / 100 * MAX_TURN_DEG_S
                gyro = tuple(turn * u + b + random.gauss(0, GYRO_NOISE_DEG_S)
                             for u, b in zip(UP, GYRO_BIAS_DEG_S))
                accel = tuple(u + random.gauss(0, ACCEL_NOISE_G) for u in UP)
                t = protocol.Telemetry(
                    flags=flags, seq=seq, cmd_seq=cmd_seq, uptime_ms=int(elapsed * 1000),
                    accel=accel, gyro=gyro,
                    imu_temp=35.0, batt_mv=int(4100 - elapsed), batt_ma=-120, vbus_mv=0,
                    batt_pct=max(0, 90 - int(elapsed / 60)),
                    charge_state=protocol.CHARGE_DISCHARGING, rssi=-50, grip=grip)
                sock.sendto(protocol.pack_telemetry(t), peer)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
