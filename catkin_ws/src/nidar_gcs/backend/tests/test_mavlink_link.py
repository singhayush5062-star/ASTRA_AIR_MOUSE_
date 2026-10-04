"""MavlinkLink against a fake PX4 vehicle over real UDP sockets.

Run (GCS backend deps installed, no ROS needed):
    cd catkin_ws/src/nidar_gcs/backend && python3 -m unittest discover -s tests -v
"""
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pymavlink  # noqa: F401
    HAVE_PYMAVLINK = True
except ImportError:
    HAVE_PYMAVLINK = False

from app.services.serial_ports import describe_open_error, list_serial_ports  # noqa: E402

if HAVE_PYMAVLINK:
    from app.services.mavlink_service import LinkConfig, MavlinkLink  # noqa: E402
    from fake_vehicle import AUTO_LAND, FakeVehicle, free_udp_port  # noqa: E402


@unittest.skipUnless(HAVE_PYMAVLINK, "pymavlink not installed")
class MavlinkLinkUdpTest(unittest.TestCase):
    def setUp(self):
        self.port = free_udp_port()
        self.texts = []
        self.link = MavlinkLink(on_statustext=lambda sev, txt, s, c: self.texts.append((sev, txt, c)))
        self.vehicle = None

    def tearDown(self):
        self.link.close()
        if self.vehicle:
            self.vehicle.stop()

    def connect(self, companion=True):
        self.vehicle = FakeVehicle(self.port, companion=companion).start()
        ok, msg = self.link.open(LinkConfig("udp", host="127.0.0.1", udp_port=self.port), wait_s=3.0)
        self.assertTrue(ok, msg)
        return msg

    def test_connect_reports_px4_and_mode(self):
        msg = self.connect()
        self.assertIn("PX4", msg)
        self.assertIn("OFFBOARD", msg)
        self.assertTrue(self.link.connected)
        time.sleep(0.4)
        t = self.link.snapshot()
        self.assertEqual(t["mode"], "OFFBOARD")
        self.assertTrue(t["armed"])
        self.assertAlmostEqual(t["voltage"], 15.8, places=2)
        self.assertEqual(t["remaining"], 87.0)
        for got, want in zip(t["local_ned"], (2.0, 1.0, -1.24)):
            self.assertAlmostEqual(got, want, places=5)  # float32 on the wire
        self.assertAlmostEqual(t["attitude"][0], 0.1, places=5)
        self.assertEqual(t["companion_compid"], 240)
        self.assertEqual(self.link.peer_ip(), "127.0.0.1")

    def test_set_mode_land_is_acked_and_applied(self):
        self.connect()
        ok, text = self.link.set_px4_mode("AUTO.LAND")
        self.assertTrue(ok, text)
        time.sleep(0.3)
        self.assertEqual(self.vehicle.custom_mode, AUTO_LAND)
        self.assertEqual(self.link.snapshot()["mode"], "AUTO.LAND")

    def test_companion_command_acked_by_companion_not_autopilot(self):
        self.connect()
        ok, text = self.link.companion_command(2, compid=240)
        self.assertTrue(ok, text)
        sent = [c for c in self.vehicle.commands if c[0] == 31010]
        self.assertEqual(sent[0][1], 240)
        self.assertEqual(sent[0][2], 2.0)

    def test_companion_command_without_companion_explains_forwarding(self):
        self.connect(companion=False)
        ok, text = self.link.companion_command(1, compid=240, timeout_s=0.3)
        self.assertFalse(ok)
        self.assertIn("MAV_1_FORWARD", text)

    def test_statustext_is_delivered(self):
        self.connect()
        self.vehicle.statustext("NIDAR STATE EXPLORATION")
        deadline = time.time() + 2
        while time.time() < deadline and not self.texts:
            time.sleep(0.05)
        self.assertEqual(self.texts[0][1], "NIDAR STATE EXPLORATION")
        self.assertEqual(self.texts[0][2], 240)

    def test_no_vehicle_fails_with_reason(self):
        ok, msg = self.link.open(LinkConfig("udp", host="127.0.0.1", udp_port=self.port), wait_s=0.5)
        self.assertFalse(ok)
        self.assertIn("gcs_url", msg)
        self.assertFalse(self.link.is_open)

    def test_force_keeps_link_open_and_picks_up_late_vehicle(self):
        ok, msg = self.link.open(LinkConfig("udp", host="127.0.0.1", udp_port=self.port),
                                 wait_s=0.3, force=True)
        self.assertTrue(ok)
        self.assertFalse(self.link.connected)
        self.vehicle = FakeVehicle(self.port).start()
        deadline = time.time() + 3
        while time.time() < deadline and not self.link.connected:
            time.sleep(0.05)
        self.assertTrue(self.link.connected)


@unittest.skipUnless(HAVE_PYMAVLINK, "pymavlink not installed")
class SerialErrorsTest(unittest.TestCase):
    def test_missing_serial_port_explains(self):
        link = MavlinkLink()
        ok, msg = link.open(LinkConfig("serial", serial_port="/dev/ttyNOPE0", baud_rate=57600),
                            wait_s=0.2)
        self.assertFalse(ok)
        self.assertIn("does not exist", msg)

    def test_empty_serial_port(self):
        ok, msg = MavlinkLink().open(LinkConfig("serial", serial_port=""), wait_s=0.2)
        self.assertFalse(ok)
        self.assertIn("Scan Ports", msg)


class SerialPortTextTest(unittest.TestCase):
    def test_permission_and_busy_messages(self):
        e = PermissionError(13, "Permission denied")
        self.assertIn("dialout", describe_open_error("/dev/ttyACM0", e))
        busy = OSError(16, "Device or resource busy")
        self.assertIn("ModemManager", describe_open_error("/dev/ttyACM0", busy))

    def test_list_ports_shape(self):
        for p in list_serial_ports():
            self.assertIn("port", p)
            self.assertIn("description", p)
            self.assertIn("hwid", p)


if __name__ == "__main__":
    unittest.main()
