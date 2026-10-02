#!/usr/bin/env python3
"""Unit tests for the stack -> UI mission mapping (app/services/mission_view.py).

Run (no ROS, no backend dependencies needed):
    python3 -m unittest discover -s catkin_ws/src/nidar_gcs/backend/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from app.services.mission_view import (  # noqa: E402
    PHASE_ORDER, derive_mission_state, grid_label, mission_phases,
)


def derive(sim="RUNNING", edm=None, armed=False, complete=False, aborted=False, n=0, since=None):
    return derive_mission_state(sim, edm, armed, complete, aborted, n, since)


class GridLabel(unittest.TestCase):
    def test_matches_grid_visualizer_convention(self):
        # grid_visualizer.py: world (0, 0) is cell (3, 3) == "D4"; row letter from grid_y
        self.assertEqual(grid_label(3, 3), "D4")
        self.assertEqual(grid_label(0, 0), "A1")
        self.assertEqual(grid_label(6, 0), "A7")
        self.assertEqual(grid_label(0, 6), "G1")

    def test_off_grid(self):
        self.assertEqual(grid_label(-1, 2), "OFF-GRID")


class DeriveMissionState(unittest.TestCase):
    def test_stopped_is_idle(self):
        self.assertEqual(derive(sim="STOPPED", edm="EXPLORATION", armed=True), "IDLE")

    def test_waiting_for_stack_is_init(self):
        self.assertEqual(derive(sim="STARTING"), "INIT")
        # the mission manager starts in TAKEOFF long before the vehicle arms
        self.assertEqual(derive(edm="TAKEOFF", armed=False), "INIT")

    def test_flight_sequence(self):
        self.assertEqual(derive(edm="TAKEOFF", armed=True), "TAKEOFF")
        self.assertEqual(derive(edm="ENTRY_SEARCH", armed=True), "LOCALIZATION")
        self.assertEqual(derive(edm="ENTRY_CONFIRMATION", armed=True), "LOCALIZATION")
        self.assertEqual(derive(edm="EXPLORATION", armed=True), "EXPLORE")
        self.assertEqual(derive(edm="RETURN", armed=True), "RETURN")
        self.assertEqual(derive(edm="DESCEND", armed=True), "LAND")
        self.assertEqual(derive(edm="LAND", armed=True), "LAND")
        self.assertEqual(derive(edm="LAND", complete=True), "COMPLETE")

    def test_survivor_flash_then_continue(self):
        self.assertEqual(derive(edm="EXPLORATION", armed=True, n=1, since=1.0), "SURVIVOR_DETECTED")
        self.assertEqual(derive(edm="EXPLORATION", armed=True, n=1, since=30.0), "CONTINUE_EXPLORE")

    def test_abort_and_error_win(self):
        self.assertEqual(derive(edm="EXPLORATION", armed=True, aborted=True), "ABORT")
        self.assertEqual(derive(sim="ERROR"), "FAILSAFE")


class Phases(unittest.TestCase):
    def statuses(self, *a, **k):
        return {p["state"]: p["status"] for p in mission_phases(*a, **k)}

    def test_order_matches_frontend_store(self):
        # frontend/src/store/index.ts MISSION_PHASES
        self.assertEqual(PHASE_ORDER, ["INIT", "TAKEOFF", "LOCALIZATION", "EXPLORE",
                                       "SURVIVOR_DETECTED", "CONTINUE_EXPLORE", "RETURN", "LAND"])

    def test_explore_midway(self):
        s = self.statuses("EXPLORE", 0)
        self.assertEqual(s["LOCALIZATION"], "COMPLETE")
        self.assertEqual(s["EXPLORE"], "ACTIVE")
        self.assertEqual(s["RETURN"], "PENDING")

    def test_survivor_phases_not_claimed_when_none_found(self):
        s = self.statuses("RETURN", 0)
        self.assertEqual(s["SURVIVOR_DETECTED"], "PENDING")
        self.assertEqual(s["CONTINUE_EXPLORE"], "PENDING")
        self.assertEqual(self.statuses("RETURN", 3)["SURVIVOR_DETECTED"], "COMPLETE")

    def test_complete_and_idle(self):
        self.assertTrue(all(v == "COMPLETE" for v in self.statuses("COMPLETE", 6).values()))
        self.assertTrue(all(v == "PENDING" for v in self.statuses("IDLE", 0).values()))

    def test_abort_marks_furthest_phase_failed(self):
        s = self.statuses("ABORT", 0, furthest="EXPLORE")
        self.assertEqual(s["EXPLORE"], "FAILED")
        self.assertEqual(s["TAKEOFF"], "COMPLETE")


if __name__ == "__main__":
    unittest.main()
