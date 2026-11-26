"""Square occupancy grid centred on the start point.

Plain Python + numpy (no ROS) so it can be unit-tested. Values follow
nav_msgs/OccupancyGrid: -1 unknown, 0 free, 100 occupied. data[row, col] with
row = y and col = x, which is the order OccupancyGrid wants.
"""

import math

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class GridMap:

    def __init__(self, resolution: float, size: float):
        self.resolution = resolution
        self.cells = int(round(size / resolution))
        self.origin = -self.cells * resolution / 2.0  # same for x and y
        self.data = np.full((self.cells, self.cells), UNKNOWN, dtype=np.int8)

    def clear(self):
        self.data.fill(UNKNOWN)

    def index(self, x: float, y: float):
        col = math.floor((x - self.origin) / self.resolution)
        row = math.floor((y - self.origin) / self.resolution)
        if 0 <= row < self.cells and 0 <= col < self.cells:
            return row, col
        return None

    def mark_footprint(self, x, y, yaw, length, width) -> bool:
        """Mark unknown cells under the rover as free. Returns True if any changed.

        Occupied cells stay occupied: they're where the rover bumped into
        something, and dead-reckoning drift shouldn't wipe them out.
        """
        res, n = self.resolution, self.cells
        reach = 0.5 * math.hypot(length, width)
        c0 = max(0, math.floor((x - reach - self.origin) / res))
        c1 = min(n - 1, math.floor((x + reach - self.origin) / res))
        r0 = max(0, math.floor((y - reach - self.origin) / res))
        r1 = min(n - 1, math.floor((y + reach - self.origin) / res))
        if c0 > c1 or r0 > r1:
            return False

        cx = self.origin + (np.arange(c0, c1 + 1) + 0.5) * res - x
        cy = self.origin + (np.arange(r0, r1 + 1) + 0.5) * res - y
        dx, dy = np.meshgrid(cx, cy)
        c, s = math.cos(yaw), math.sin(yaw)
        forward = dx * c + dy * s
        left = -dx * s + dy * c
        inside = (np.abs(forward) <= length / 2) & (np.abs(left) <= width / 2)

        block = self.data[r0:r1 + 1, c0:c1 + 1]
        newly_free = inside & (block == UNKNOWN)
        block[newly_free] = FREE
        return bool(newly_free.any())

    def mark_obstacle(self, x, y, direction, half_width) -> bool:
        """Mark a short wall across `direction`, centred on (x, y)."""
        px, py = -math.sin(direction), math.cos(direction)
        steps = max(1, math.ceil(half_width / (self.resolution / 2)))
        changed = False
        for i in range(-steps, steps + 1):
            t = half_width * i / steps
            idx = self.index(x + px * t, y + py * t)
            if idx is not None and self.data[idx] != OCCUPIED:
                self.data[idx] = OCCUPIED
                changed = True
        return changed

    def count(self, value: int) -> int:
        return int(np.count_nonzero(self.data == value))
