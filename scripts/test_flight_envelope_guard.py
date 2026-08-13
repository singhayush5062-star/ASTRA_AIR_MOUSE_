#!/usr/bin/env python3
"""
Unit and Integration Test Suite for Flight Envelope Guard
========================================================
Tests:
- Test 1: Z Limit validation (1.45 <= Z <= 1.55)
- Test 2: Arena XY boundary validation (-7.0 <= X <= 7.0, -7.0 <= Y <= 7.0)
- Test 3: South Gate crossing detection (Y_MIN boundary crossing from y >= -7.0 to y < -7.0)
"""

import unittest
import math
import sys
import os

# Import FlightEnvelopeGuard class from scripts directory
sys.path.insert(0, '/home/developer/NIDAR/scripts')
from flight_envelope_guard import FlightEnvelopeGuard

class MockPoint:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class MockPositionCommand:
    def __init__(self, x, y, z, yaw=0.0):
        self.position = MockPoint(x, y, z)
        self.velocity = MockPoint(0.0, 0.0, 0.0)
        self.yaw = yaw

class TestFlightEnvelopeGuard(unittest.TestCase):
    def setUp(self):
        # Instantiate guard with default test parameters
        self.guard = FlightEnvelopeGuard.__new__(FlightEnvelopeGuard)
        self.guard.x_min = -7.0
        self.guard.x_max = 7.0
        self.guard.y_min = -7.0
        self.guard.y_max = 7.0
        self.guard.z_min = 1.45
        self.guard.z_max = 1.55
        self.guard.boundary_margin = 0.0

        self.guard.eff_x_min = self.guard.x_min
        self.guard.eff_x_max = self.guard.x_max
        self.guard.eff_y_min = self.guard.y_min
        self.guard.eff_y_max = self.guard.y_max
        self.guard.eff_z_min = self.guard.z_min
        self.guard.eff_z_max = self.guard.z_max

        self.guard.last_valid_pos = None

    def test_z_limits(self):
        print("\n--- Running Test 1: Z Limit Validation ---")
        z_test_cases = [
            (1.50, True, "Z=1.50 nominal"),
            (1.45, True, "Z=1.45 min bound"),
            (1.55, True, "Z=1.55 max bound"),
            (1.40, False, "Z=1.40 below min"),
            (1.60, False, "Z=1.60 above max"),
            (2.00, False, "Z=2.00 way above max"),
            (0.50, False, "Z=0.50 way below min"),
        ]

        for z, expected_valid, label in z_test_cases:
            cmd = MockPositionCommand(0.0, 0.0, z)
            is_valid, reason = self.guard.validate_command(cmd)
            print(f"  Testing {label:25s} -> z={z:.2f}m | Result: {is_valid} ('{reason}')")
            self.assertEqual(is_valid, expected_valid, f"Failed for {label}")

    def test_arena_xy_boundaries(self):
        print("\n--- Running Test 2: Arena XY Boundary Validation ---")
        xy_test_cases = [
            (0.0, 0.0, 1.50, True, "Inside center"),
            (-6.9, 0.0, 1.50, True, "Inside X_MIN"),
            (6.9, 0.0, 1.50, True, "Inside X_MAX"),
            (0.0, -6.9, 1.50, True, "Inside Y_MIN"),
            (0.0, 6.9, 1.50, True, "Inside Y_MAX"),
            (-7.1, 0.0, 1.50, False, "Outside X_MIN"),
            (7.1, 0.0, 1.50, False, "Outside X_MAX"),
            (0.0, 7.1, 1.50, False, "Outside Y_MAX"),
            (0.0, -7.1, 1.50, False, "Outside Y_MIN"),
        ]

        for x, y, z, expected_valid, label in xy_test_cases:
            cmd = MockPositionCommand(x, y, z)
            is_valid, reason = self.guard.validate_command(cmd)
            print(f"  Testing {label:25s} -> ({x:.1f}, {y:.1f}, {z:.1f}) | Result: {is_valid} ('{reason}')")
            self.assertEqual(is_valid, expected_valid, f"Failed for {label}")

    def test_gate_crossing(self):
        print("\n--- Running Test 3: Gate Boundary Crossing Validation ---")
        gate_test_sequence = [
            (0.0, -6.8, 1.50, True, "Inside gate approach"),
            (0.0, -6.9, 1.50, True, "Nearing boundary"),
            (0.0, -7.0, 1.50, True, "On boundary Y=-7.0"),
            (0.0, -7.1, 1.50, False, "Attempting gate escape Y=-7.1"),
        ]

        for x, y, z, expected_valid, label in gate_test_sequence:
            cmd = MockPositionCommand(x, y, z)
            is_valid, reason = self.guard.validate_command(cmd)
            print(f"  Testing {label:35s} -> y={y:.2f}m | Result: {is_valid} ('{reason}')")
            self.assertEqual(is_valid, expected_valid, f"Failed for {label}")
            if is_valid:
                self.guard.last_valid_pos = (x, y, z)

if __name__ == '__main__':
    unittest.main()
