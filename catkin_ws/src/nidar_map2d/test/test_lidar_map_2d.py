#!/usr/bin/env python3
"""Unit tests for the LiDAR 2D mapper's pure functions (scripts/lidar_map_2d.py).

Run (sourced ROS environment, no simulator):
    python3 -m unittest discover -s catkin_ws/src/nidar_map2d/test -v
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import lidar_map_2d as lm  # noqa: E402


def room_scan(sensor, half=2.0, z_wall=1.4, n=720, floor_ring=1.5):
    """A sensor inside a closed square room |x|,|y| <= half: wall returns at z_wall all around,
    plus a ring of floor returns (z = 0) like a downward VLP-16 beam."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    d = np.stack([np.cos(a), np.sin(a)], 1)
    # distance from sensor to the square along each direction
    with np.errstate(divide='ignore'):
        tx = np.where(d[:, 0] > 0, (half - sensor[0]) / d[:, 0], (-half - sensor[0]) / d[:, 0])
        ty = np.where(d[:, 1] > 0, (half - sensor[1]) / d[:, 1], (-half - sensor[1]) / d[:, 1])
    t = np.minimum(np.abs(tx), np.abs(ty))
    walls = np.column_stack([sensor[0] + d[:, 0] * t, sensor[1] + d[:, 1] * t, np.full(n, z_wall)])
    floor = np.column_stack([sensor[0] + d[:, 0] * floor_ring, sensor[1] + d[:, 1] * floor_ring,
                             np.zeros(n)])
    return np.vstack([walls, floor])


class Mapper(unittest.TestCase):
    def setUp(self):
        self.g = lm.GridSpec(-3.0, -3.0, 120, 120, 0.05)
        self.L = np.zeros(self.g.width * self.g.height, np.float32)

    def state_at(self, x, y):
        occ = lm.to_occupancy(self.L).reshape(self.g.height, self.g.width)
        return occ[int((y - self.g.y0) / self.g.res), int((x - self.g.x0) / self.g.res)]

    def wall_near(self, x, y):
        """A wall exactly on a cell boundary may land in either neighbouring cell."""
        return any(self.state_at(x + dx, y + dy) == 100
                   for dx in (-0.05, 0, 0.05) for dy in (-0.05, 0, 0.05))

    def test_room_walls_free_interior_unknown_outside(self):
        sensor = np.array([0.3, -0.4, 1.5])
        lm.integrate_scan(self.L, self.g, sensor, room_scan(sensor), 0.3, 2.2, 12.0,
                          rng=np.random.RandomState(0))
        self.assertTrue(self.wall_near(2.0, 0.0))          # east wall
        self.assertTrue(self.wall_near(-1.0, 2.0))         # north wall
        self.assertEqual(self.state_at(0.0, 0.0), 0)       # floor inside
        self.assertEqual(self.state_at(1.2, -1.2), 0)
        self.assertEqual(self.state_at(2.6, 0.0), -1)      # behind the wall: never seen

    def test_floor_returns_do_not_make_walls(self):
        sensor = np.array([0.0, 0.0, 1.5])
        lm.integrate_scan(self.L, self.g, sensor, room_scan(sensor), 0.3, 2.2, 12.0,
                          rng=np.random.RandomState(0))
        self.assertEqual(self.state_at(1.5, 0.0), 0)       # where the floor ring landed

    def test_spurious_return_is_cleared_by_later_rays(self):
        sensor = np.array([0.0, 0.0, 1.5])
        ghost = np.array([[1.0, 0.0, 1.4]])
        lm.integrate_scan(self.L, self.g, sensor, ghost, 0.3, 2.2, 12.0)
        self.assertEqual(self.state_at(1.0, 0.0), 100)
        for _ in range(5):   # the room's east wall is behind the ghost; rays pass through it
            lm.integrate_scan(self.L, self.g, sensor, room_scan(sensor), 0.3, 2.2, 12.0,
                              rng=np.random.RandomState(1))
        self.assertEqual(self.state_at(1.0, 0.0), 0)
        self.assertTrue(self.wall_near(2.0, 0.0))

    def test_walls_survive_repeated_grazing_rays(self):
        sensor = np.array([0.0, 0.0, 1.5])
        for k in range(10):
            s = sensor + np.array([0.1 * k - 0.5, 0.05 * k, 0.0])
            lm.integrate_scan(self.L, self.g, s, room_scan(s), 0.3, 2.2, 12.0,
                              rng=np.random.RandomState(k))
        occ = lm.to_occupancy(self.L).reshape(self.g.height, self.g.width)
        east = occ[:, int((2.0 - self.g.x0) / self.g.res)]
        rows = slice(int((-1.5 - self.g.y0) / self.g.res), int((1.5 - self.g.y0) / self.g.res))
        self.assertGreater((east[rows] == 100).mean(), 0.9)

    def test_range_cap(self):
        sensor = np.array([0.0, 0.0, 1.5])
        far = np.array([[2.5, 0.0, 1.4]])
        lm.integrate_scan(self.L, self.g, sensor, far, 0.3, 2.2, max_range=2.0)
        self.assertEqual(self.state_at(2.5, 0.0), -1)
        self.assertEqual(self.state_at(1.0, 0.0), -1)


if __name__ == '__main__':
    unittest.main()
