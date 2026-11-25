import math

from rover_driver import protocol


def test_sizes_match_firmware():
    # Must match the static_asserts in firmware/rover/include/protocol.h.
    assert protocol.COMMAND_SIZE == 10
    assert protocol.TELEMETRY_SIZE == 50


def test_command_round_trip():
    cmd = protocol.Command(seq=65535, x=-100, y=100, z=-7, grip=45, flags=protocol.CMD_FLAG_LOCK)
    data = protocol.pack_command(cmd)
    assert data[:2] == b'RC'
    assert protocol.unpack_command(data) == cmd


def test_telemetry_round_trip():
    t = protocol.Telemetry(
        flags=protocol.TLM_FLAG_LOCKED | protocol.TLM_FLAG_IMU_OK, seq=3, cmd_seq=9,
        uptime_ms=123456, accel=(0.0, 0.5, 1.0), gyro=(-1.5, 0.0, 90.0), imu_temp=math.nan,
        batt_mv=4012, batt_ma=-150, vbus_mv=-1, batt_pct=-1, charge_state=5, rssi=-61, grip=0)
    back = protocol.unpack_telemetry(protocol.pack_telemetry(t))
    assert back.locked and back.imu_ok and not back.rover_found and not back.moving
    assert back.accel == t.accel and back.gyro == t.gyro
    assert math.isnan(back.imu_temp)
    assert (back.batt_mv, back.batt_ma, back.vbus_mv, back.batt_pct, back.rssi) == \
        (4012, -150, -1, -1, -61)


def test_rejects_wrong_packets():
    good = protocol.pack_command(protocol.Command(seq=1, x=0, y=0, z=0))
    assert protocol.unpack_command(good[:-1]) is None
    assert protocol.unpack_command(b'XX' + good[2:]) is None
    assert protocol.unpack_telemetry(good) is None
