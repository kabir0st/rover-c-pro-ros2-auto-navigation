import math
import random

from rover_slam.grid_map import FREE, GridMap, OCCUPIED, UNKNOWN
from rover_slam.heading import HeadingFilter, UpVector

DT = 0.02  # 50 Hz, like the IMU


def test_up_vector_finds_tilted_up_axis():
    up = UpVector()
    tilt = (0.0, -0.3, 9.7)
    for _ in range(200):
        up.update(tilt)
    n = math.sqrt(sum(t * t for t in tilt))
    assert all(abs(u - t / n) < 1e-6 for u, t in zip(up.up, tilt))
    # A turn about that axis reads as pure yaw rate.
    gyro = tuple(0.5 * t / n for t in tilt)
    assert abs(up.yaw_rate(gyro) - 0.5) < 1e-6
    assert up.horizontal(tilt) < 1e-6
    up.update((50.0, 0.0, 0.0))  # a bump is ignored
    assert abs(up.up[2] - 9.7 / n) < 1e-6


def test_heading_filter_learns_bias_at_rest_and_removes_it_when_moving():
    random.seed(1)
    bias = 0.01  # rad/s, about 0.6 deg/s
    f = HeadingFilter()
    for _ in range(int(3 / DT)):  # 3 s standing still
        rate = f.update(DT, bias + random.gauss(0, 0.003), 0.0, still=True)
        assert abs(rate) < 0.01
    assert abs(f.bias - bias) < 0.002

    # Turning at 1 rad/s, motors only manage 0.8: the filter follows the gyro.
    heading = 0.0
    for _ in range(int(2 / DT)):
        heading += f.update(DT, 0.8 + bias + random.gauss(0, 0.003), 1.0, still=False) * DT
    assert abs(heading - 1.6) < 0.02
    assert abs(f.bias - bias) < 0.002  # moving doesn't disturb the learned bias


def test_heading_filter_follows_rover_moved_by_hand():
    f = HeadingFilter()
    heading = 0.0
    for _ in range(int(1 / DT)):
        heading += f.update(DT, 1.0, 0.0, still=True) * DT
    assert heading > 0.9  # not absorbed as bias
    assert abs(f.bias) < 0.01


def test_footprint_marks_cells_under_the_rover():
    m = GridMap(0.05, 2.0)
    assert m.cells == 40 and m.count(UNKNOWN) == 1600
    assert m.mark_footprint(0.0, 0.0, 0.0, 0.14, 0.12)
    assert m.data[m.index(0.0, 0.0)] == FREE
    assert m.data[m.index(0.3, 0.0)] == UNKNOWN
    assert 4 <= m.count(FREE) <= 12
    assert not m.mark_footprint(0.0, 0.0, 0.0, 0.14, 0.12)  # nothing new
    # Off the map edge: ignored, no crash.
    assert not m.mark_footprint(5.0, 5.0, 0.3, 0.14, 0.12)


def test_obstacles_survive_footprints():
    m = GridMap(0.05, 2.0)
    assert m.mark_obstacle(0.1, 0.0, 0.0, 0.06)
    assert m.data[m.index(0.1, 0.0)] == OCCUPIED
    m.mark_footprint(0.1, 0.0, 0.0, 0.14, 0.12)
    assert m.data[m.index(0.1, 0.0)] == OCCUPIED
