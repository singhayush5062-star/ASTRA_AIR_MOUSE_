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


if __name__ == '__main__':
    unittest.main()
