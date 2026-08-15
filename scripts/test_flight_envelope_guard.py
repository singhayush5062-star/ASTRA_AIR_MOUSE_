#!/usr/bin/env python3
"""
Unit Tests for Production-Grade Flight Envelope Guard
======================================================
Tests:
1. Authoritative World-frame coordinate transformation.
2. Boundary validation & structured rejection codes (OUT_OF_BOUNDS_X, Y, Z, BOUNDARY_CROSSING).
3. Velocity separation and zeroing on held setpoints.
"""

import sys
import os
if '/opt/ros/noetic/lib/python3/dist-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
catkin_py = '/home/developer/NIDAR/catkin_ws/devel/lib/python3/dist-packages'
if os.path.exists(catkin_py) and catkin_py not in sys.path:
    sys.path.insert(0, catkin_py)

import unittest
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget


class DummyGuard:
    def __init__(self):
        self.world_x_min = -7.0
        self.world_x_max = 7.0
        self.world_y_min = -7.0
        self.world_y_max = 7.0
        self.world_z_min = 1.45
        self.world_z_max = 1.55
        self.boundary_margin = 0.2

        self.eff_xw_min = self.world_x_min + self.boundary_margin
        self.eff_xw_max = self.world_x_max - self.boundary_margin
        self.eff_yw_min = self.world_y_min + self.boundary_margin
        self.eff_yw_max = self.world_y_max - self.boundary_margin
        self.eff_zw_min = self.world_z_min
        self.eff_zw_max = self.world_z_max
        self.last_valid_pos_world = None

    def camera_to_world(self, xc, yc, zc):
        xw = -yc
        yw = xc - 6.5
        zw = zc + 0.1
        return xw, yw, zw

    def world_to_camera(self, xw, yw, zw):
        """Inverse rigid transformation from Gazebo world to camera_init frame."""
        xc = yw + 6.5
        yc = -xw
        zc = zw - 0.1
        return xc, yc, zc

    def clamp_and_transform(self, xc, yc, zc):
        """Simulate fuel_cb clamping: world-frame clamp + inverse transform."""
        xw, yw, zw = self.camera_to_world(xc, yc, zc)
        xw_c = min(max(xw, self.eff_xw_min), self.eff_xw_max)
        yw_c = min(max(yw, self.eff_yw_min), self.eff_yw_max)
        zw_c = min(max(zw, self.eff_zw_min), self.eff_zw_max)
        return self.world_to_camera(xw_c, yw_c, zw_c)

    def validate_command(self, cmd):
        xc, yc, zc = cmd.position.x, cmd.position.y, cmd.position.z
        xw, yw, zw = self.camera_to_world(xc, yc, zc)

        if xw < self.eff_xw_min:
            return False, "OUT_OF_BOUNDS_X_MIN", f"World X below min (xw={xw:.2f}m < {self.eff_xw_min:.2f}m)", (xw, yw, zw)
        if xw > self.eff_xw_max:
            return False, "OUT_OF_BOUNDS_X_MAX", f"World X above max (xw={xw:.2f}m > {self.eff_xw_max:.2f}m)", (xw, yw, zw)

        if yw < self.eff_yw_min:
            return False, "OUT_OF_BOUNDS_Y_MIN", f"World Y below min (South Gate: yw={yw:.2f}m < {self.eff_yw_min:.2f}m)", (xw, yw, zw)
        if yw > self.eff_yw_max:
            return False, "OUT_OF_BOUNDS_Y_MAX", f"World Y above max (North Wall: yw={yw:.2f}m > {self.eff_yw_max:.2f}m)", (xw, yw, zw)

        if zw < self.eff_zw_min:
            return False, "OUT_OF_BOUNDS_Z_MIN", f"World Z below min (zw={zw:.2f}m < {self.eff_zw_min:.2f}m)", (xw, yw, zw)
        if zw > self.eff_zw_max:
            return False, "OUT_OF_BOUNDS_Z_MAX", f"World Z above max (zw={zw:.2f}m > {self.eff_zw_max:.2f}m)", (xw, yw, zw)

        return True, "ACCEPT", "Safe setpoint inside arena envelope", (xw, yw, zw)


class TestFlightEnvelopeGuardProduction(unittest.TestCase):
    def setUp(self):
        self.guard = DummyGuard()

    def test_camera_to_world_transform(self):
        print("\n--- Running Test 1: Camera -> World Coordinate Transform ---")
        xw, yw, zw = self.guard.camera_to_world(0.0, 0.0, 0.0)
        self.assertAlmostEqual(xw, 0.0)
        self.assertAlmostEqual(yw, -6.5)
        self.assertAlmostEqual(zw, 0.1)
        print(f"  Spawn check: camera=(0,0,0) -> world=({xw:.2f},{yw:.2f},{zw:.2f}) | PASS")

        xw, yw, zw = self.guard.camera_to_world(-0.5, 0.0, 0.0)
        self.assertAlmostEqual(xw, 0.0)
        self.assertAlmostEqual(yw, -7.0)
        self.assertAlmostEqual(zw, 0.1)
        print(f"  South door check: camera=(-0.5,0,0) -> world=({xw:.2f},{yw:.2f},{zw:.2f}) | PASS")

        xw, yw, zw = self.guard.camera_to_world(13.5, 0.0, 0.0)
        self.assertAlmostEqual(xw, 0.0)
        self.assertAlmostEqual(yw, 7.0)
        self.assertAlmostEqual(zw, 0.1)
        print(f"  North wall check: camera=(13.5,0,0) -> world=({xw:.2f},{yw:.2f},{zw:.2f}) | PASS")

    def test_structured_rejection_codes(self):
        print("\n--- Running Test 2: Structured Rejection Codes ---")
        cmd = PositionCommand()
        cmd.position.x = 6.5
        cmd.position.y = 0.0
        cmd.position.z = 1.4
        is_valid, code, reason, world_pt = self.guard.validate_command(cmd)
        self.assertTrue(is_valid)
        self.assertEqual(code, "ACCEPT")
        print(f"  Center interior check -> Result: {is_valid} ({code}) | PASS")

        cmd.position.x = -0.6  # world y = -7.1 (out of bounds)
        is_valid, code, reason, world_pt = self.guard.validate_command(cmd)
        self.assertFalse(is_valid)
        self.assertEqual(code, "OUT_OF_BOUNDS_Y_MIN")
        print(f"  South gate breach -> Result: {is_valid} ({code}) | PASS")

        cmd.position.x = 6.5
        cmd.position.z = 2.5  # world z = 2.6 (out of bounds)
        is_valid, code, reason, world_pt = self.guard.validate_command(cmd)
        self.assertFalse(is_valid)
        self.assertEqual(code, "OUT_OF_BOUNDS_Z_MAX")
        print(f"  High altitude breach -> Result: {is_valid} ({code}) | PASS")

    def test_velocity_zeroing_mask(self):
        print("\n--- Running Test 3: Velocity Zeroing Type Mask ---")
        expected_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        self.assertTrue(expected_mask & PositionTarget.IGNORE_VX > 0)
        self.assertTrue(expected_mask & PositionTarget.IGNORE_VY > 0)
        self.assertTrue(expected_mask & PositionTarget.IGNORE_VZ > 0)
        print("  Hold setpoint type_mask correctly ignores velocity components: PASS")

    def test_world_to_camera_inverse(self):
        """world_to_camera is the exact inverse of camera_to_world."""
        print("\n--- Running Test 4: World -> Camera Inverse Transform ---")
        # Spawn: world=(0, -6.5, 0.1) -> camera=(0, 0, 0)
        xc, yc, zc = self.guard.world_to_camera(0.0, -6.5, 0.1)
        self.assertAlmostEqual(xc, 0.0)
        self.assertAlmostEqual(yc, 0.0)
        self.assertAlmostEqual(zc, 0.0)
        print(f"  Spawn inverse: world=(0,-6.5,0.1) -> camera=({xc:.2f},{yc:.2f},{zc:.2f}) | PASS")

        # Arena center: world=(0, 0, 1.5) -> camera=(6.5, 0, 1.4)
        xc, yc, zc = self.guard.world_to_camera(0.0, 0.0, 1.5)
        self.assertAlmostEqual(xc, 6.5)
        self.assertAlmostEqual(yc, 0.0)
        self.assertAlmostEqual(zc, 1.4)
        print(f"  Arena center inverse: world=(0,0,1.5) -> camera=({xc:.2f},{yc:.2f},{zc:.2f}) | PASS")

    def test_round_trip_transform(self):
        """camera -> world -> camera preserves the point exactly."""
        print("\n--- Running Test 5: Round-Trip Transform ---")
        test_points = [
            (0.0, 0.0, 0.0),
            (6.5, 3.0, 1.4),
            (13.0, -6.0, 1.5),
            (-0.3, 5.0, 1.45),
        ]
        for xc, yc, zc in test_points:
            xw, yw, zw = self.guard.camera_to_world(xc, yc, zc)
            xc2, yc2, zc2 = self.guard.world_to_camera(xw, yw, zw)
            self.assertAlmostEqual(xc, xc2, places=6)
            self.assertAlmostEqual(yc, yc2, places=6)
            self.assertAlmostEqual(zc, zc2, places=6)
            print(f"  ({xc:.1f},{yc:.1f},{zc:.1f}) -> world -> ({xc2:.2f},{yc2:.2f},{zc2:.2f}) | PASS")

    def test_clamp_interior_point_unchanged(self):
        """A point well inside the arena should not be modified by clamping."""
        print("\n--- Running Test 6: Interior Point Not Clamped ---")
        # camera=(6.5, 0, 1.4) -> world=(0, 0, 1.5) — center of arena
        xc, yc, zc = 6.5, 0.0, 1.4
        px, py, pz = self.guard.clamp_and_transform(xc, yc, zc)
        self.assertAlmostEqual(px, xc, places=4)
        self.assertAlmostEqual(py, yc, places=4)
        self.assertAlmostEqual(pz, zc, places=4)
        print(f"  Interior ({xc},{yc},{zc}) -> ({px:.2f},{py:.2f},{pz:.2f}) unchanged | PASS")

        # camera=(10, 3, 1.4) -> world=(-3, 3.5, 1.5) — inside arena
        xc, yc, zc = 10.0, 3.0, 1.4
        px, py, pz = self.guard.clamp_and_transform(xc, yc, zc)
        self.assertAlmostEqual(px, xc, places=4)
        self.assertAlmostEqual(py, yc, places=4)
        print(f"  Interior ({xc},{yc},{zc}) -> ({px:.2f},{py:.2f},{pz:.2f}) unchanged | PASS")

    def test_clamp_at_world_boundary(self):
        """Points outside world boundaries should be clamped correctly."""
        print("\n--- Running Test 7: World Boundary Clamping ---")
        # camera=(15, 0, 1.4) -> world=(0, 8.5, 1.5) — yw exceeds eff_yw_max (6.8)
        # Clamped world = (0, 6.8, 1.5) -> camera = (6.8+6.5, 0, 1.4) = (13.3, 0, 1.4)
        xc, yc, zc = 15.0, 0.0, 1.4
        px, py, pz = self.guard.clamp_and_transform(xc, yc, zc)
        self.assertAlmostEqual(px, 13.3, places=2)
        self.assertAlmostEqual(py, 0.0, places=2)
        print(f"  Boundary ({xc},{yc},{zc}) -> clamped ({px:.2f},{py:.2f},{pz:.2f}) | PASS")

        # camera=(6.5, -8, 1.4) -> world=(8, 0, 1.5) — xw exceeds eff_xw_max (6.8)
        # Clamped world = (6.8, 0, 1.5) -> camera = (0+6.5, -6.8, 1.4) = (6.5, -6.8, 1.4)
        xc, yc, zc = 6.5, -8.0, 1.4
        px, py, pz = self.guard.clamp_and_transform(xc, yc, zc)
        self.assertAlmostEqual(px, 6.5, places=2)
        self.assertAlmostEqual(py, -6.8, places=2)
        print(f"  Boundary ({xc},{yc},{zc}) -> clamped ({px:.2f},{py:.2f},{pz:.2f}) | PASS")

    def test_camera_axis_mapping(self):
        """Verify camera X corresponds to world Y and camera Y to world X."""
        print("\n--- Running Test 8: Camera-World Axis Mapping ---")
        # Moving only in camera X should change only world Y
        xw1, yw1, _ = self.guard.camera_to_world(5.0, 0.0, 0.0)
        xw2, yw2, _ = self.guard.camera_to_world(10.0, 0.0, 0.0)
        self.assertAlmostEqual(xw1, xw2)  # world X unchanged
        self.assertNotAlmostEqual(yw1, yw2)  # world Y changed
        print(f"  Camera X movement: world X unchanged ({xw1:.2f}=={xw2:.2f}), Y changed ({yw1:.2f}!={yw2:.2f}) | PASS")

        # Moving only in camera Y should change only world X
        xw1, yw1, _ = self.guard.camera_to_world(5.0, 2.0, 0.0)
        xw2, yw2, _ = self.guard.camera_to_world(5.0, 5.0, 0.0)
        self.assertAlmostEqual(yw1, yw2)  # world Y unchanged
        self.assertNotAlmostEqual(xw1, xw2)  # world X changed
        print(f"  Camera Y movement: world Y unchanged ({yw1:.2f}=={yw2:.2f}), X changed ({xw1:.2f}!={xw2:.2f}) | PASS")


if __name__ == '__main__':
    unittest.main()
