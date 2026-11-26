"""Heading estimation from the drive command and the IMU.

Plain Python (no ROS) so it can be unit-tested.
"""

import math

STANDARD_GRAVITY = 9.80665


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


class UpVector:
    """Tracks which way is up in the IMU's own axes, from the accelerometer.

    At rest an accelerometer reads +1 g pointing up. Projecting the gyro onto
    that axis gives the yaw rate however the stick happens to be mounted, so
    the IMU's axes never need configuring.
    """

    def __init__(self, smoothing: float = 0.05):
        self.smoothing = smoothing
        self.up = None

    def update(self, accel):
        n = _norm(accel)
        if not 0.8 * STANDARD_GRAVITY < n < 1.2 * STANDARD_GRAVITY:
            return  # a bump or hard acceleration, not a clean gravity reading
        unit = [a / n for a in accel]
        if self.up is None:
            self.up = unit
            return
        k = self.smoothing
        mixed = [(1 - k) * u + k * a for u, a in zip(self.up, unit)]
        m = _norm(mixed)
        self.up = [c / m for c in mixed]

    def yaw_rate(self, gyro) -> float:
        """Rotation rate about the up axis, counter-clockwise positive."""
        if self.up is None:
            return gyro[2]
        return sum(g * u for g, u in zip(gyro, self.up))

    def horizontal(self, accel) -> float:
        """Size of the acceleration in the ground plane (gravity removed)."""
        if self.up is None:
            return 0.0
        along = sum(a * u for a, u in zip(accel, self.up))
        return _norm([a - along * u for a, u in zip(accel, self.up)])


class HeadingFilter:
    """Kalman filter over [yaw rate, gyro bias].

    Predict: the yaw rate is what was commanded (the input). Standing still,
    that's trusted to within `still_sigma`. Moving, it gets a wide margin,
    because the motors never turn exactly the commanded amount.
    Update: the gyro measures yaw rate + bias + noise.

    The bias is only observable at rest: while moving, a steady gap between
    command and gyro could be motor error just as well as bias. So the bias is
    learned while the rover is told to stand still, and held fixed while it
    moves (a "consider" state). Then the yaw rate is the command and the
    bias-corrected gyro blended by their variances, which in practice is
    mostly the gyro. Bias is what makes a cheap gyro's heading drift.
    """

    def __init__(self, gyro_noise=0.02, bias_walk=0.0005, initial_bias_sigma=0.05,
                 still_sigma=0.002, moving_sigma=0.3, moving_sigma_per_rad_s=0.5,
                 still_gate=0.2):
        self.gyro_noise = gyro_noise            # rad/s, per sample
        self.bias_walk = bias_walk              # rad/s per sqrt(s)
        self.still_sigma = still_sigma          # rad/s
        self.moving_sigma = moving_sigma        # rad/s
        self.moving_sigma_per_rad_s = moving_sigma_per_rad_s
        self.still_gate = still_gate            # rad/s: "still" but turning this fast = picked up
        self.bias = 0.0
        self.bias_var = initial_bias_sigma ** 2
        self.rate = 0.0
        self.rate_var = 0.0

    def update(self, dt: float, gyro_rate: float, cmd_rate: float, still: bool) -> float:
        """Feed one gyro sample. Returns the estimated yaw rate (rad/s)."""
        if still and abs(gyro_rate - self.bias) > self.still_gate:
            still = False  # commanded still, yet clearly turning: someone moved it by hand
        self.bias_var += self.bias_walk ** 2 * dt
        noise_var = self.gyro_noise ** 2

        if still:
            # Prior: rate = 0. The gyro reading is mostly bias.
            prior_var = self.still_sigma ** 2
            s = prior_var + self.bias_var + noise_var
            innovation = gyro_rate - self.bias
            self.rate = prior_var / s * innovation
            self.bias += self.bias_var / s * innovation
            self.rate_var = prior_var - prior_var ** 2 / s
            self.bias_var -= self.bias_var ** 2 / s
        else:
            # Prior: rate = command. Bias held; the corrected gyro measures the rate.
            prior_var = (self.moving_sigma + self.moving_sigma_per_rad_s * abs(cmd_rate)) ** 2
            measured = gyro_rate - self.bias
            measured_var = noise_var + self.bias_var
            total = prior_var + measured_var
            self.rate = (cmd_rate * measured_var + measured * prior_var) / total
            self.rate_var = prior_var * measured_var / total
        return self.rate
