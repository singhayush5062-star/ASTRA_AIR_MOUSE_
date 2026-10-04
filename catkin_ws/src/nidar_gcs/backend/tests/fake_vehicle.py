"""A minimal MAVLink vehicle for tests: a PX4 autopilot (component 1) plus, optionally, the
onboard mission commander (component 240, MAVROS's default id) on one UDP socket.

It sends what the GCS reads (HEARTBEAT, ATTITUDE, LOCAL_POSITION_NED, SYS_STATUS, STATUSTEXT) and
answers COMMAND_LONG the way PX4 and nidar_hardware/mission_commander.py do.
"""
import os
import socket
import threading
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

OFFBOARD = 6 << 16                 # PX4 custom_mode: main 6
AUTO_LAND = (4 << 16) | (6 << 24)  # PX4 custom_mode: main 4 sub 6


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakeVehicle(object):
    def __init__(self, gcs_port: int, companion: bool = True) -> None:
        self.conn = mavutil.mavlink_connection("udpout:127.0.0.1:%d" % gcs_port,
                                               source_system=1, source_component=1)
        self.mav_ap = self.conn.mav
        # Second encoder sharing the socket: messages from component 240 (the companion).
        self.mav_cc = mavutil.mavlink.MAVLink(self.conn, srcSystem=1, srcComponent=240)
        self.companion = companion
        self.custom_mode = OFFBOARD
        self.armed = True
        self.commands = []
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._tx, daemon=True),
                         threading.Thread(target=self._rx, daemon=True)]

    def start(self):
        for t in self._threads:
            t.start()
        return self

    def stop(self):
        self._stop.set()
        for t in self._threads:
            if t.is_alive():
                t.join(timeout=2)
        self.conn.close()

    def _tx(self):
        while not self._stop.is_set():
            base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            self.mav_ap.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                       mavutil.mavlink.MAV_AUTOPILOT_PX4, base, self.custom_mode, 4)
            if self.companion:
                self.mav_cc.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 4)
            self.mav_ap.attitude_send(0, 0.1, -0.2, 0.0, 0, 0, 0)
            # NED (north 2, east 1, 1.24 m up) == camera_init (1, 2, 1.24)
            self.mav_ap.local_position_ned_send(0, 2.0, 1.0, -1.24, 0.5, 0.0, -0.1)
            self.mav_ap.sys_status_send(0, 0, 0, 500, 15800, 1200, 87, 0, 0, 0, 0, 0, 0)
            time.sleep(0.1)

    def statustext(self, text, severity=6, from_companion=True):
        mav = self.mav_cc if from_companion else self.mav_ap
        mav.statustext_send(severity, text.encode())

    def _rx(self):
        while not self._stop.is_set():
            msg = self.conn.recv_match(type="COMMAND_LONG", blocking=True, timeout=0.2)
            if msg is None:
                continue
            self.commands.append((msg.command, msg.target_component, msg.param1, msg.param2,
                                  msg.param3))
            if msg.target_component in (0, 1):
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    self.custom_mode = (int(msg.param2) << 16) | (int(msg.param3) << 24)
                    res = mavutil.mavlink.MAV_RESULT_ACCEPTED
                elif msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    res = mavutil.mavlink.MAV_RESULT_ACCEPTED
                else:
                    res = mavutil.mavlink.MAV_RESULT_UNSUPPORTED
                self.mav_ap.command_ack_send(msg.command, res)
            elif msg.target_component == 240 and self.companion:
                if msg.command == mavutil.mavlink.MAV_CMD_USER_1:
                    ok = int(msg.param1) in (1, 2, 3)
                    self.mav_cc.command_ack_send(
                        msg.command, mavutil.mavlink.MAV_RESULT_ACCEPTED if ok
                        else mavutil.mavlink.MAV_RESULT_DENIED)
