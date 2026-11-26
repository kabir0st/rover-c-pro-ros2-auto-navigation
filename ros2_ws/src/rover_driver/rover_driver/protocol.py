"""UDP packet layout shared with the rover firmware.

Mirrors firmware/rover/include/protocol.h; change both together.
Little-endian, packed.
"""

from dataclasses import dataclass
import struct

ROVER_UDP_PORT = 4210
PROTOCOL_VERSION = 1

CMD_FLAG_LOCK = 0x01
CMD_FLAG_UNLOCK = 0x02

TLM_FLAG_LOCKED = 0x01
TLM_FLAG_ROVER_FOUND = 0x02
TLM_FLAG_MOVING = 0x04
TLM_FLAG_IMU_OK = 0x08

# M5Unified m5::charge_state_t
CHARGE_CHARGING = 1
CHARGE_NOT_CHARGING = 2
CHARGE_FULL = 3
CHARGE_DISABLED = 4
CHARGE_DISCHARGING = 5
CHARGE_IDLE = 6

_COMMAND = struct.Struct('<2sBBHbbbB')
_TELEMETRY = struct.Struct('<2sBBHHI3f3ffhhhbbbB')
COMMAND_SIZE = _COMMAND.size
TELEMETRY_SIZE = _TELEMETRY.size


@dataclass
class Command:
    """PC -> rover. Speeds are -100..100 on M5_RoverC::setSpeed's axes."""

    seq: int
    x: int  # strafe, + = right
    y: int  # forward, + = forward
    z: int  # turn, + = left (counter-clockwise)
    grip: int = 0  # gripper angle 10..90, 0 = leave as is
    flags: int = 0  # CMD_FLAG_*


@dataclass
class Telemetry:
    """Rover -> PC. IMU values are in the stick's own axes."""

    flags: int
    seq: int
    cmd_seq: int
    uptime_ms: int
    accel: tuple  # g
    gyro: tuple  # deg/s
    imu_temp: float  # deg C, NaN if unknown
    batt_mv: int  # <= 0 if unknown
    batt_ma: int  # + charging / - discharging, 0 if not measurable
    vbus_mv: int  # -1 if not measurable
    batt_pct: int  # -1 if unknown
    charge_state: int  # CHARGE_*
    rssi: int  # dBm, 0 if unknown
    grip: int  # 0 = never set

    @property
    def locked(self) -> bool:
        return bool(self.flags & TLM_FLAG_LOCKED)

    @property
    def rover_found(self) -> bool:
        return bool(self.flags & TLM_FLAG_ROVER_FOUND)

    @property
    def moving(self) -> bool:
        return bool(self.flags & TLM_FLAG_MOVING)

    @property
    def imu_ok(self) -> bool:
        return bool(self.flags & TLM_FLAG_IMU_OK)


def wheel_speeds(x: int, y: int, z: int) -> tuple:
    """Wheel powers the rover sets for a command: M5_RoverC::setSpeed, line by line.

    Order: front-left, front-right, rear-left, rear-right.
    """
    if z != 0:
        x = int(x * (100 - abs(z)) / 100)
        y = int(y * (100 - abs(z)) / 100)

    def clamp(v):
        return max(-100, min(100, v))

    return clamp(y + x - z), clamp(y - x + z), clamp(y - x - z), clamp(y + x + z)


def effective_speeds(x: int, y: int, z: int) -> tuple:
    """What the rover really does for a command, as (x, y, z) in -100..100.

    Goes through the wheel mixing above, so it includes the library's turn
    scaling and the per-wheel clipping (a full-power diagonal only drives two
    wheels, for example).
    """
    fl, fr, rl, rr = wheel_speeds(x, y, z)
    return (fl - fr - rl + rr) / 4, (fl + fr + rl + rr) / 4, (-fl + fr - rl + rr) / 4


def pack_command(cmd: Command) -> bytes:
    return _COMMAND.pack(b'RC', PROTOCOL_VERSION, cmd.flags, cmd.seq & 0xFFFF,
                         cmd.x, cmd.y, cmd.z, cmd.grip)


def unpack_command(data: bytes) -> Command | None:
    if len(data) != COMMAND_SIZE:
        return None
    magic, version, flags, seq, x, y, z, grip = _COMMAND.unpack(data)
    if magic != b'RC' or version != PROTOCOL_VERSION:
        return None
    return Command(seq=seq, x=x, y=y, z=z, grip=grip, flags=flags)


def pack_telemetry(t: Telemetry) -> bytes:
    return _TELEMETRY.pack(b'RT', PROTOCOL_VERSION, t.flags, t.seq & 0xFFFF, t.cmd_seq & 0xFFFF,
                           t.uptime_ms, *t.accel, *t.gyro, t.imu_temp, t.batt_mv, t.batt_ma,
                           t.vbus_mv, t.batt_pct, t.charge_state, t.rssi, t.grip)


def unpack_telemetry(data: bytes) -> Telemetry | None:
    if len(data) != TELEMETRY_SIZE:
        return None
    f = _TELEMETRY.unpack(data)
    if f[0] != b'RT' or f[1] != PROTOCOL_VERSION:
        return None
    return Telemetry(flags=f[2], seq=f[3], cmd_seq=f[4], uptime_ms=f[5],
                     accel=f[6:9], gyro=f[9:12], imu_temp=f[12],
                     batt_mv=f[13], batt_ma=f[14], vbus_mv=f[15], batt_pct=f[16],
                     charge_state=f[17], rssi=f[18], grip=f[19])
