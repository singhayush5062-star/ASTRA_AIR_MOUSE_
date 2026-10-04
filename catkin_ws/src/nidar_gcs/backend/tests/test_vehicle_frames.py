"""PX4 mode decoding and the vehicle -> arena world frame chain (app/services/vehicle_frames.py)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.services.vehicle_frames import (  # noqa: E402
    PX4_MODE_IDS, WorldFrame, mode_name, px4_mode_name,
)


def custom(main, sub=0):
    return (main << 16) | (sub << 24)


class Px4ModeTest(unittest.TestCase):
    def test_names_match_mavros(self):
        self.assertEqual(px4_mode_name(custom(6)), "OFFBOARD")
        self.assertEqual(px4_mode_name(custom(4, 6)), "AUTO.LAND")
        self.assertEqual(px4_mode_name(custom(4, 2)), "AUTO.TAKEOFF")
        self.assertEqual(px4_mode_name(custom(4, 5)), "AUTO.RTL")
        self.assertEqual(px4_mode_name(custom(3)), "POSCTL")
        self.assertEqual(px4_mode_name(custom(2)), "ALTCTL")

    def test_ids_round_trip(self):
        for name, (main, sub) in PX4_MODE_IDS.items():
            self.assertEqual(px4_mode_name(custom(main, sub)), name)

    def test_custom_mode_flag_and_ardupilot_fallback(self):
        self.assertEqual(mode_name(12, 0, custom(6)), "--")
        self.assertEqual(mode_name(12, 1, custom(6)), "OFFBOARD")
        self.assertEqual(mode_name(3, 1, 4), "GUIDED")


class WorldFrameTest(unittest.TestCase):
    def setUp(self):
        # the generated defaults: pad (0, -9.5), spawn z 0.26, facing +Y (north)
        self.f = WorldFrame(0.0, -9.5, 0.26, math.pi / 2)

    def test_matches_guard_camera_to_world(self):
        # flight_envelope_guard.py: xw = spawn_x - yc, yw = spawn_y + xc, zw = spawn_z + zc
        for xc, yc, zc in ((0, 0, 0), (1.0, 0.0, 1.24), (2.5, -1.0, 1.24), (-3.0, 4.0, 0.5)):
            # camera_init -> PX4 NED: x_ned = yc, y_ned = xc, z_ned = -zc
            xw, yw, zw = self.f.position_from_ned(yc, xc, -zc)
            self.assertAlmostEqual(xw, 0.0 - yc)
            self.assertAlmostEqual(yw, -9.5 + xc)
            self.assertAlmostEqual(zw, 0.26 + zc)

    def test_heading(self):
        # On the pad facing the door: camera_init +x == world +y; PX4 NED yaw for camera_init +x
        # is pi/2 (east in NED == +x_cinit). World yaw must be +pi/2 (north).
        _, _, yaw = self.f.attitude_from_ned(0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(yaw, math.pi / 2)
        roll, pitch, _ = self.f.attitude_from_ned(0.1, 0.2, 0.0)
        self.assertAlmostEqual(roll, 0.1)
        self.assertAlmostEqual(pitch, -0.2)

    def test_velocity_rotates_like_position(self):
        vx, vy, vz = self.f.velocity_from_ned(0.0, 1.0, -0.5)   # +x_cinit, climbing
        self.assertAlmostEqual(vx, 0.0, places=9)
        self.assertAlmostEqual(vy, 1.0)
        self.assertAlmostEqual(vz, 0.5)

    def test_from_mission_config(self):
        f = WorldFrame.from_mission_config({"launch_pad": {"center": {"x": 1.0, "y": -9.0},
                                                           "thickness": 0.03,
                                                           "spawn_yaw": 1.5707963},
                                            "vehicle": {"belly_clearance": 0.23}})
        self.assertAlmostEqual(f.spawn_z, 0.26)
        self.assertEqual((f.pad_x, f.pad_y), (1.0, -9.0))


if __name__ == "__main__":
    unittest.main()
