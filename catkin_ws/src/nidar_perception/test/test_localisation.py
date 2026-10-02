#!/usr/bin/env python3
"""Unit tests for the survivor localisation geometry (phase plan 4.3).

Run (needs a sourced ROS environment, no simulator):
    python3 -m unittest discover -s catkin_ws/src/nidar_perception/test -v

The tests import the geometry functions straight out of scripts/survivor_detector.py.
"""
import json
import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import survivor_detector as sd  # noqa: E402

K = np.array([[375.0, 0.0, 320.0], [0.0, 375.0, 240.0], [0.0, 0.0, 1.0]])


def gazebo_project(point_cam, K):
    """Independent forward projection: Gazebo camera frame (x fwd, y left, z up) -> pixel."""
    x, y, z = point_cam
    return K[0, 2] - K[0, 0] * y / x, K[1, 2] - K[1, 1] * z / x


class PixelRay(unittest.TestCase):
    def test_principal_point_is_straight_ahead(self):
        np.testing.assert_allclose(sd.pixel_ray_camera_link(320, 240, K), [1, 0, 0], atol=1e-12)

    def test_right_of_centre_is_negative_y(self):
        # camera_link +Y is LEFT, so something on the right of the image has y < 0
        self.assertLess(sd.pixel_ray_camera_link(420, 240, K)[1], 0)
        self.assertGreater(sd.pixel_ray_camera_link(220, 240, K)[1], 0)

    def test_below_centre_is_negative_z(self):
        # camera_link +Z is UP, so something lower in the image has z < 0
        self.assertLess(sd.pixel_ray_camera_link(320, 340, K)[2], 0)
        self.assertGreater(sd.pixel_ray_camera_link(320, 140, K)[2], 0)

    def test_unit_length(self):
        for u, v in ((0, 0), (639, 479), (100, 400)):
            self.assertAlmostEqual(np.linalg.norm(sd.pixel_ray_camera_link(u, v, K)), 1.0)

    def test_round_trip_with_independent_projection(self):
        for target in ([3.0, 0.5, -0.8], [5.0, -1.2, -1.1], [2.0, 0.0, -0.3]):
            u, v = gazebo_project(target, K)
            ray = sd.pixel_ray_camera_link(u, v, K)
            expected = np.array(target) / np.linalg.norm(target)
            np.testing.assert_allclose(ray, expected, atol=1e-9)


class PlaneIntersection(unittest.TestCase):
    def test_hits_plane_at_known_point(self):
        origin = np.array([0.0, 0.0, 1.5])
        target = np.array([3.0, 1.0, 0.35])
        d = (target - origin) / np.linalg.norm(target - origin)
        np.testing.assert_allclose(sd.intersect_plane(origin, d, 0.35, 8.0), target, atol=1e-9)

    def test_near_horizontal_ray_rejected(self):
        self.assertIsNone(sd.intersect_plane(np.array([0, 0, 1.5]), np.array([1.0, 0, -0.01]), 0.35, 80.0))

    def test_ray_pointing_up_rejected(self):
        self.assertIsNone(sd.intersect_plane(np.array([0, 0, 1.5]), np.array([0.7, 0, 0.7]), 0.35, 8.0))

    def test_plane_above_camera_rejected(self):
        self.assertIsNone(sd.intersect_plane(np.array([0, 0, 0.2]), np.array([0.8, 0, -0.6]), 0.35, 8.0))

    def test_range_cap(self):
        origin = np.array([0.0, 0.0, 1.5])
        d = np.array([math.cos(0.1), 0, -math.sin(0.1)])      # shallow: hit is ~11 m away
        self.assertIsNone(sd.intersect_plane(origin, d, 0.35, 8.0))
        self.assertIsNotNone(sd.intersect_plane(origin, d, 0.35, 20.0))


class BoxGate(unittest.TestCase):
    def test_interior_box_ok(self):
        self.assertEqual(sd.box_is_usable((100, 100, 200, 220), 640, 480, 6, 20), (True, None))

    def test_each_border_rejected(self):
        for box in ((0, 100, 90, 200), (100, 2, 200, 90), (500, 100, 640, 200), (100, 300, 200, 480)):
            self.assertEqual(sd.box_is_usable(box, 640, 480, 6, 20), (False, 'border'), box)

    def test_speck_rejected(self):
        self.assertEqual(sd.box_is_usable((100, 100, 112, 160), 640, 480, 6, 20), (False, 'small'))


class DuplicateMerge(unittest.TestCase):
    """The 2026-10-02 flight tagged survivor_4 three times and survivor_5 twice."""

    @staticmethod
    def track(tid, xy, n, published):
        t = sd.Track(tid, [xy[0], xy[1], 0.0], 1.0, 0.8)
        t.n_obs = n
        t.published = published
        return t

    def test_nearby_confirmed_track_is_merged(self):
        a = self.track(4, (5.81, -2.41), 12, True)
        b = self.track(6, (5.16, -3.14), 3, False)           # 0.98 m from a
        got = sd.merge_into_published([a, b], b, 1.5)
        self.assertIs(got, a)
        self.assertEqual(a.n_obs, 15)
        # observation-weighted: moves a little, toward b
        self.assertLess(5.16, a.position[0])
        self.assertLess(a.position[0], 5.81)
        self.assertGreater(a.position[0], 5.6)

    def test_distinct_survivor_not_merged(self):
        a = self.track(1, (4.0, -4.0), 10, True)
        b = self.track(2, (6.0, -2.0), 3, False)             # 2.83 m: the sim's closest pair
        self.assertIsNone(sd.merge_into_published([a, b], b, 1.5))
        self.assertEqual(a.n_obs, 10)

    def test_only_published_tracks_absorb(self):
        a = self.track(1, (0.0, 0.0), 5, False)
        b = self.track(2, (0.5, 0.0), 3, False)
        self.assertIsNone(sd.merge_into_published([a, b], b, 1.5))

    def test_picks_the_nearest_published_track(self):
        a = self.track(1, (0.0, 0.0), 5, True)
        c = self.track(3, (1.2, 0.0), 5, True)
        b = self.track(2, (0.9, 0.0), 3, False)
        self.assertIs(sd.merge_into_published([a, c, b], b, 1.5), c)


class GroundTruthReplay(unittest.TestCase):
    """Real YOLO boxes recorded in Gazebo, localised with the production functions."""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(HERE, 'data', 'gazebo_detections.json')) as f:
            cls.rows = json.load(f)['detections']

    def localise(self, r):
        b = r['bbox']
        ok, _ = sd.box_is_usable(b, 640, 480, 6, 20)
        if not ok:
            return None
        Kr = np.array(r['K']).reshape(3, 3)
        T = np.array(r['Tsensor'])
        d = T[:3, :3] @ sd.pixel_ray_camera_link(0.5 * (b[0] + b[2]), 0.5 * (b[1] + b[3]), Kr)
        hit = sd.intersect_plane(T[:3, 3], d, 0.35, 8.0)
        return None if hit is None else hit[:2]

    def test_accuracy_on_usable_detections(self):
        errs = []
        for r in self.rows:
            p = self.localise(r)
            if p is not None:
                errs.append(np.linalg.norm(p - np.array(r['true'])))
        self.assertGreaterEqual(len(errs), 15, 'too few usable detections in the fixture')
        errs = np.array(errs)
        self.assertLess(np.median(errs), 0.6, 'median error %.2f m' % np.median(errs))
        # competition grid cells are 2 m: everything must land well inside the right cell
        self.assertLess(errs.max(), 1.4, 'worst error %.2f m' % errs.max())

    def test_gate_removes_the_bad_detections(self):
        # every detection the gates let through must be within one cell; the ones they drop
        # include all the grossly wrong ones
        bad_through = 0
        for r in self.rows:
            b = r['bbox']
            p = self.localise(r)
            if p is not None and np.linalg.norm(p - np.array(r['true'])) >= 1.4:
                bad_through += 1
        self.assertEqual(bad_through, 0)

    def test_old_axis_mapping_would_fail(self):
        """Guard against reverting to [oz, +ox, +oy]: it must be demonstrably worse on this data."""
        errs_old = []
        for r in self.rows:
            b = r['bbox']
            if not sd.box_is_usable(b, 640, 480, 6, 20)[0]:
                continue
            Kr = np.array(r['K']).reshape(3, 3)
            T = np.array(r['Tsensor'])
            u, v = 0.5 * (b[0] + b[2]), 0.5 * (b[1] + b[3])
            o = np.array([(u - Kr[0, 2]) / Kr[0, 0], (v - Kr[1, 2]) / Kr[1, 1], 1.0])
            o /= np.linalg.norm(o)
            d = T[:3, :3] @ np.array([o[2], o[0], o[1]])            # the old, wrong mapping
            hit = sd.intersect_plane(T[:3, 3], d, 0.35, 8.0)
            if hit is not None:
                errs_old.append(np.linalg.norm(hit[:2] - np.array(r['true'])))
        self.assertTrue(len(errs_old) == 0 or np.median(errs_old) > 1.4)


if __name__ == '__main__':
    unittest.main()
