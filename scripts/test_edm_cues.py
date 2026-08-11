#!/usr/bin/env python3
"""
Unit Test Suite for Entry Detection Module (EDM)
------------------------------------------------
Tests multi-cue detection algorithms, door opening width identification (1.0m nominal),
opening traversal tracking, mandatory cue hard-gating, and confidence score fusion.
"""

import unittest
import numpy as np
import math
import sys
import os

# Add scripts directory to module search path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import EDM detector class
from entry_detection_module import MultiCueEntryDetector, MissionState


class MockRosParam:
    @staticmethod
    def get_param(name, default):
        return default


# Mock rospy params if rospy master is not running during unit tests
import rospy
rospy.get_param = MockRosParam.get_param


class TestMultiCueEntryDetector(unittest.TestCase):

    def setUp(self):
        self.detector = MultiCueEntryDetector()

    def test_default_weights_and_thresholds(self):
        """Verify cue weights sum to 1.0 and nominal door parameters are correct."""
        total_weight = (
            self.detector.w1 + self.detector.w2 + self.detector.w3 +
            self.detector.w4 + self.detector.w5
        )
        self.assertAlmostEqual(total_weight, 1.0, places=4)
        self.assertEqual(self.detector.door_min_width, 0.50)
        self.assertEqual(self.detector.door_max_width, 1.20)
        self.assertEqual(self.detector.target_door_width, 1.00)

    def test_opening_crossed_mandatory_gating(self):
        """Verify confidence score is hard-gated below 0.45 if opening is not crossed."""
        # Setup localization and clearance to perfect values
        self.detector.cue_scores['cue3_stable_localization'] = 1.0
        self.detector.cue_scores['cue4_obstacle_clearance'] = 1.0
        self.detector.cue_scores['cue2_free_space'] = 1.0
        self.detector.cue_scores['cue5_obstacle_density'] = 1.0

        # Opening NOT crossed
        self.detector.opening_crossed = False

        conf, mandatory_ok, confirmed = self.detector.evaluate_confidence()
        self.assertFalse(mandatory_ok)
        self.assertFalse(confirmed)
        self.assertLessEqual(conf, 0.45)

    def test_successful_entry_confirmation(self):
        """Verify successful entry confirmation after opening crossed & 10 stable cycles."""
        self.detector.cue_scores['cue3_stable_localization'] = 1.0
        self.detector.cue_scores['cue4_obstacle_clearance'] = 1.0
        self.detector.cue_scores['cue2_free_space'] = 0.8
        self.detector.cue_scores['cue5_obstacle_density'] = 0.8

        self.detector.opening_crossed = True
        self.detector.cue_scores['cue1_opening_crossed'] = 1.0

        # Cycle 1 to 9: Should not be confirmed yet
        for i in range(1, 10):
            conf, mandatory_ok, confirmed = self.detector.evaluate_confidence()
            self.assertTrue(mandatory_ok)
            self.assertFalse(confirmed, f"Failed at cycle {i}")
            self.assertGreaterEqual(conf, 0.70)

        # Cycle 10: Should trigger confirmation!
        conf, mandatory_ok, confirmed = self.detector.evaluate_confidence()
        self.assertTrue(mandatory_ok)
        self.assertTrue(confirmed)

    def test_unstable_localization_resets_hysteresis(self):
        """Verify unstable localization immediately resets consecutive stable cycles."""
        self.detector.cue_scores['cue1_opening_crossed'] = 1.0
        self.detector.cue_scores['cue2_free_space'] = 1.0
        self.detector.cue_scores['cue3_stable_localization'] = 1.0
        self.detector.cue_scores['cue4_obstacle_clearance'] = 1.0
        self.detector.cue_scores['cue5_obstacle_density'] = 1.0
        self.detector.opening_crossed = True

        # Build 5 stable cycles
        for _ in range(5):
            self.detector.evaluate_confidence()

        self.assertEqual(self.detector.consecutive_stable_cycles, 5)

        # Localization drops
        self.detector.odom_rate_hz = 5.0  # Below 12Hz threshold
        self.detector.evaluate_confidence(loc_healthy=False)

        # Stability counter must reset to 0
        self.assertEqual(self.detector.consecutive_stable_cycles, 0)


if __name__ == '__main__':
    unittest.main()
