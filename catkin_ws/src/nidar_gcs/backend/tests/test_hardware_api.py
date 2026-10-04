"""Hardware REST/WebSocket API end to end: the real FastAPI app, a fake PX4 vehicle over UDP.

Covers what the Hardware dashboard does: scan ports, CONNECT (UDP like Wi-Fi via the Jetson),
live telemetry over /api/ws/hardware in the arena world frame, LAND / ABORT reaching PX4 as
AUTO.LAND, TAKEOFF / RTL reaching the onboard commander (MAV_CMD_USER_1), the companion's
STATUSTEXT mission reports, and a failed CONNECT that explains itself.

Run: cd catkin_ws/src/nidar_gcs/backend && python3 -m unittest discover -s tests -v
"""
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import pymavlink  # noqa: F401
    from fastapi.testclient import TestClient
    HAVE_DEPS = True
except ImportError:
    HAVE_DEPS = False

if HAVE_DEPS:
    from app.main import app, hw  # noqa: E402
    from app.services.hardware_service import parse_nidar_text  # noqa: E402
    from fake_vehicle import AUTO_LAND, FakeVehicle, free_udp_port  # noqa: E402


def wait_for(pred, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.05)
    return False


@unittest.skipUnless(HAVE_DEPS, "fastapi/pymavlink not installed")
class HardwareApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.pop("ROS_MASTER_URI", None)
        cls.client = TestClient(app)
        cls.client.__enter__()   # run the lifespan (hardware service loop)

    @classmethod
    def tearDownClass(cls):
        cls.client.__exit__(None, None, None)

    def setUp(self):
        self.port = free_udp_port()
        self.vehicle = FakeVehicle(self.port).start()

    def tearDown(self):
        self.client.post("/api/hardware/disconnect")
        self.vehicle.stop()

    def connect(self):
        r = self.client.post("/api/hardware/connect", json={
            "connection_type": "udp", "host": "127.0.0.1", "udp_port": self.port,
            "drone_name": "Test Drone"})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertTrue(r.json()["connected"])
        return r.json()

    def test_ports_endpoint(self):
        r = self.client.get("/api/hardware/ports")
        self.assertEqual(r.status_code, 200)
        self.assertIn("ports", r.json())

    def test_connect_status_and_world_frame(self):
        self.assertIn("PX4", self.connect()["message"])
        self.assertTrue(wait_for(lambda: self.client.get("/api/hardware/status").json()
                                 ["drone"]["position"]["x"] is not None))
        st = self.client.get("/api/hardware/status").json()
        self.assertEqual(st["status"], "CONNECTED")
        d = st["drone"]
        self.assertEqual(d["id"], "Test Drone")
        self.assertEqual(d["mode"], "OFFBOARD")
        self.assertEqual(d["connectionStatus"], "CONNECTED")
        # fake NED (2, 1, -1.24) -> camera_init (1, 2, 1.24) -> world: pad (0, -9.5) + Rz(90)
        self.assertAlmostEqual(d["position"]["x"], -2.0, places=2)
        self.assertAlmostEqual(d["position"]["y"], -8.5, places=2)
        self.assertAlmostEqual(d["position"]["z"], 1.5, places=2)
        self.assertAlmostEqual(d["battery"]["percentage"], 87.0)
        self.assertEqual(st["health"]["px4"]["status"], "CONNECTED")
        self.assertEqual(st["health"]["fastlio2"]["status"], "UNKNOWN")  # no ROS link

    def test_websocket_streams_telemetry(self):
        self.connect()
        with self.client.websocket_connect("/api/ws/hardware") as ws:
            msg = ws.receive_json()
            self.assertEqual(msg["type"], "telemetry")
            self.assertEqual(msg["payload"]["drone"]["mode"], "OFFBOARD")
            self.assertIn("mission_phases", msg["payload"])

    def test_land_command_reaches_px4(self):
        self.connect()
        r = self.client.post("/api/hardware/command", json={"command": "land"})
        self.assertEqual(r.status_code, 200, r.text)
        self.assertEqual(r.json()["via"], "MAVLink")
        self.assertTrue(wait_for(lambda: self.vehicle.custom_mode == AUTO_LAND))

    def test_abort(self):
        self.connect()
        r = self.client.post("/api/hardware/abort")
        self.assertEqual(r.status_code, 200, r.text)
        self.assertTrue(wait_for(lambda: self.vehicle.custom_mode == AUTO_LAND))
        self.assertTrue(wait_for(lambda: self.client.get("/api/hardware/status").json()
                                 ["mission_state"] == "ABORT"))

    def test_takeoff_and_rtl_go_to_companion(self):
        self.connect()
        for cmd, code in (("takeoff", 1.0), ("rtl", 2.0)):
            r = self.client.post("/api/hardware/command", json={"command": cmd})
            self.assertEqual(r.status_code, 200, r.text)
            self.assertIn((31010, 240, code), [c[:3] for c in self.vehicle.commands])

    def test_companion_statustext_drives_mission_and_survivors(self):
        self.connect()
        self.vehicle.statustext("NIDAR ST EXPLORATION")
        self.vehicle.statustext("NIDAR SV 3 C4 91.5 -1.20 2.40")
        ok = wait_for(lambda: self.client.get("/api/hardware/status").json()["survivors"])
        self.assertTrue(ok)
        st = self.client.get("/api/hardware/status").json()
        self.assertEqual(st["survivors"][0]["gridLabel"], "C4")
        self.assertIn(st["mission_state"], ("SURVIVOR_DETECTED", "CONTINUE_EXPLORE"))

    def test_command_when_disconnected(self):
        r = self.client.post("/api/hardware/command", json={"command": "takeoff"})
        self.assertEqual(r.status_code, 409)

    def test_connect_failure_explains(self):
        self.vehicle.stop()
        port = free_udp_port()
        r = self.client.post("/api/hardware/connect", json={
            "connection_type": "udp", "host": "127.0.0.1", "udp_port": port})
        self.assertEqual(r.status_code, 503)
        self.assertIn("gcs_url", r.json()["error"])
        self.vehicle = FakeVehicle(self.port)  # tearDown stops it

    def test_events_websocket(self):
        self.connect()
        with self.client.websocket_connect("/api/ws/hardware/events") as ws:
            msg = ws.receive_json()
            self.assertEqual(msg["type"], "event")


@unittest.skipUnless(HAVE_DEPS, "fastapi/pymavlink not installed")
class StatusTextProtocolTest(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(parse_nidar_text("NIDAR ST RETURN"), {"kind": "state", "state": "RETURN"})
        sv = parse_nidar_text("NIDAR SV 2 B4 88 1.5 -2.0")
        self.assertEqual((sv["id"], sv["grid"], sv["x"]), (2, "B4", 1.5))
        self.assertEqual(parse_nidar_text("NIDAR RDY 0 FAST-LIO not tracking"),
                         {"kind": "ready", "ready": False, "reason": "FAST-LIO not tracking"})
        self.assertIsNone(parse_nidar_text("Preflight Fail: ekf2 missing data"))
        self.assertIsNone(parse_nidar_text("NIDAR SV x"))


if __name__ == "__main__":
    unittest.main()
