"""
NIDAR AirMouse GCS — MAVLink link to the real vehicle
====================================================
One MAVLink connection to the flight controller over whatever reaches it from the GCS laptop:

  serial   RC data link (T12 ground unit), SiK telemetry radio, or the FC's own USB port
  udp      Wi-Fi: MAVROS on the Jetson forwards MAVLink to this laptop (gcs_url) -- we listen
  tcp      a MAVLink TCP server (mavlink-router, SITL)

A connection is only reported as up once an AUTOPILOT heartbeat has actually been received;
otherwise open() fails with the reason (port permissions, port busy, no data, data but no valid
MAVLink -> wrong baud, ...). Once up, a reader thread keeps the telemetry snapshot current, the
GCS sends its own 1 Hz HEARTBEAT (PX4 uses it for its data-link-loss logic), a serial link that
drops (cable pulled, radio rebooted) is reopened automatically, and commands are sent as
COMMAND_LONG with ACK + retries.

Everything here is blocking and thread based (pymavlink is); HardwareService calls it from an
executor so the FastAPI event loop never waits on a radio.
"""

from __future__ import annotations

import os
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

# Speak MAVLink 2 (PX4's default on every link). Must be set before pymavlink is imported.
os.environ.setdefault("MAVLINK20", "1")

try:
    from pymavlink import mavutil
except ImportError:  # reported by open(); the rest of the GCS keeps working
    mavutil = None

from app.services.serial_ports import describe_open_error
from app.services.vehicle_frames import (
    MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MAV_MODE_FLAG_SAFETY_ARMED,
    PX4_MODE_IDS, mode_name,
)

HEARTBEAT_TIMEOUT_S = 3.0      # autopilot heartbeat older than this = link down
RECONNECT_PERIOD_S = 2.0

# MAVLink enums used here (numeric so this module imports without pymavlink for the tests of
# everything else).
MAV_TYPE_GCS = 6
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_USER_1 = 31010
MAV_RESULT = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
              4: "FAILED", 5: "IN_PROGRESS", 6: "CANCELLED"}
MSG_ID_LOCAL_POSITION_NED = 32
MSG_ID_EXTENDED_SYS_STATE = 245

# Components that are not the autopilot but may heartbeat on the same link: MAVROS/companion
# (240 by default, 191 onboard computer), cameras, gimbals, other GCSs.
AUTOPILOT_COMPID = 1


class LinkConfig(object):
    def __init__(self, connection_type: str = "serial", serial_port: Optional[str] = None,
                 baud_rate: int = 57600, host: str = "0.0.0.0", udp_port: int = 14550,
                 tcp_port: int = 5760) -> None:
        self.connection_type = (connection_type or "serial").lower()
        self.serial_port = (serial_port or "").strip()
        self.baud_rate = int(baud_rate or 57600)
        self.host = (host or "").strip()
        self.udp_port = int(udp_port or 14550)
        self.tcp_port = int(tcp_port or 5760)

    def url(self) -> str:
        if self.connection_type == "serial":
            return self.serial_port
        if self.connection_type == "udp":
            # Listen: MAVROS (gcs_url) / PX4 SITL / mavlink-router send to us, we reply to them.
            return "udpin:%s:%d" % (self.host or "0.0.0.0", self.udp_port)
        if self.connection_type == "tcp":
            return "tcp:%s:%d" % (self.host or "127.0.0.1", self.tcp_port)
        raise ValueError("unsupported connection type %r" % self.connection_type)

    def label(self) -> str:
        if self.connection_type == "serial":
            return "%s @ %d baud" % (self.serial_port, self.baud_rate)
        if self.connection_type == "udp":
            return "UDP %s:%d" % (self.host or "0.0.0.0", self.udp_port)
        return "TCP %s:%d" % (self.host or "127.0.0.1", self.tcp_port)


class MavlinkLink(object):
    def __init__(self, gcs_sysid: int = 255, gcs_compid: int = 190,
                 on_statustext: Optional[Callable[[int, str, int, int], None]] = None,
                 on_link_event: Optional[Callable[[str, str], None]] = None) -> None:
        self.gcs_sysid = gcs_sysid
        self.gcs_compid = gcs_compid
        self.on_statustext = on_statustext
        self.on_link_event = on_link_event
        self.cfg: Optional[LinkConfig] = None
        self._mav = None
        self._lock = threading.Lock()          # telemetry
        self._tx_lock = threading.Lock()       # pymavlink sends
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []
        self._acks: List[Dict[str, Any]] = []
        self._text_chunks: Dict[Tuple[int, int, int], List[str]] = {}
        self._reset_telemetry()

    # -- state ---------------------------------------------------------------------------------

    def _reset_telemetry(self) -> None:
        self.t: Dict[str, Any] = {
            "sysid": None, "compid": None, "autopilot": None, "vehicle_type": None,
            "mode": "--", "base_mode": 0, "custom_mode": 0, "armed": False, "system_status": None,
            "last_heartbeat": 0.0, "attitude": None, "attitude_time": 0.0,
            "local_ned": None, "local_vel_ned": None, "local_time": 0.0,
            "voltage": None, "current": None, "remaining": None,
            "range_m": None, "range_time": 0.0, "landed_state": None,
            "sensors_present": 0, "sensors_enabled": 0, "sensors_health": 0,
            "radio": None, "companion_heartbeat": 0.0, "companion_compid": None,
            "packets": 0, "link_error": None,
        }

    @property
    def is_open(self) -> bool:
        return self._mav is not None and not self._stop.is_set()

    def heartbeat_age(self) -> float:
        hb = self.t.get("last_heartbeat") or 0.0
        return time.time() - hb if hb else float("inf")

    @property
    def connected(self) -> bool:
        return self.is_open and self.heartbeat_age() < HEARTBEAT_TIMEOUT_S

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self.t)

    def peer_ip(self) -> Optional[str]:
        """Source IP of the last UDP packet (the Jetson, when MAVROS forwards over Wi-Fi)."""
        # pymavlink's udpin server keeps every sender in clients_last_alive (last_address is
        # only filled in broadcast mode); the most recently heard one is the vehicle side.
        alive = getattr(self._mav, "clients_last_alive", None) or {}
        if alive:
            addr = max(alive.items(), key=lambda kv: kv[1])[0]
            if addr and addr[0] not in ("0.0.0.0", ""):
                return str(addr[0])
        if self.cfg is not None and self.cfg.connection_type == "tcp":
            return self.cfg.host
        return None

    # -- open / close ----------------------------------------------------------------------------

    def _connect(self, cfg: LinkConfig):
        url = cfg.url()
        if cfg.connection_type == "serial":
            if not url:
                raise RuntimeError("No serial port selected. Press Scan Ports and pick the radio / FC.")
            if os.name != "nt" and not os.path.exists(url):
                raise FileNotFoundError(2, "No such file or directory", url)
        return mavutil.mavlink_connection(url, baud=cfg.baud_rate, source_system=self.gcs_sysid,
                                          source_component=self.gcs_compid, autoreconnect=True,
                                          robust_parsing=True, retries=0)

    def open(self, cfg: LinkConfig, wait_s: float = 6.0, force: bool = False) -> Tuple[bool, str]:
        """Open the link and wait up to wait_s for an autopilot heartbeat.

        force=True keeps the link open even without a heartbeat (bench tests: radio powered but
        the vehicle not yet), so it is reported as soon as one arrives.
        """
        self.close()
        if mavutil is None:
            return False, ("pymavlink is not installed in the GCS backend. Re-run "
                           "catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh.")
        self.cfg = cfg
        self._stop.clear()
        with self._lock:
            self._reset_telemetry()
        try:
            self._mav = self._connect(cfg)
        except Exception as e:  # serial.SerialException, socket errors, ...
            self._mav = None
            return False, self._open_error(cfg, e)

        self._threads = [threading.Thread(target=self._reader, name="mavlink-rx", daemon=True),
                         threading.Thread(target=self._heartbeat_tx, name="mavlink-hb", daemon=True)]
        for th in self._threads:
            th.start()

        deadline = time.time() + max(0.5, wait_s)
        while time.time() < deadline:
            if self.t["last_heartbeat"]:
                hb = self.snapshot()
                return True, ("Connected to %s (system %s, %s) over %s"
                              % ("PX4" if hb["autopilot"] == 12 else "autopilot %s" % hb["autopilot"],
                                 hb["sysid"], hb["mode"], cfg.label()))
            time.sleep(0.05)

        reason = self._no_heartbeat_reason(cfg)
        if force:
            return True, "Link open without a heartbeat yet (bench mode): " + reason
        self.close()
        return False, reason

    def _open_error(self, cfg: LinkConfig, e: BaseException) -> str:
        if cfg.connection_type == "serial":
            return describe_open_error(cfg.serial_port or "(no port)", e)
        if cfg.connection_type == "udp" and ("in use" in str(e).lower() or getattr(e, "errno", 0) == 98):
            return ("UDP port %d is already in use on this computer (QGroundControl or another GCS "
                    "open?). Close it, or pick another port and point MAVROS gcs_url at it."
                    % cfg.udp_port)
        if cfg.connection_type == "tcp":
            return "TCP connection to %s:%d failed: %s" % (cfg.host, cfg.tcp_port, e)
        return "Could not open %s: %s" % (cfg.label(), e)

    def _no_heartbeat_reason(self, cfg: LinkConfig) -> str:
        mav = getattr(self._mav, "mav", None)
        nbytes = getattr(mav, "total_bytes_received", 0) if mav else 0
        npkts = getattr(mav, "total_packets_received", 0) if mav else 0
        nerr = getattr(mav, "total_receive_errors", 0) if mav else 0
        comp = self.t.get("companion_heartbeat")
        if npkts and comp:
            return ("Only a companion computer heartbeat on %s (MAVROS/Jetson), none from the "
                    "flight controller. Check the FC is powered and MAVROS reaches it." % cfg.label())
        if npkts:
            return ("%d MAVLink packets on %s but no autopilot HEARTBEAT. Is the FC streaming on "
                    "this port (MAV_x_CONFIG / MAV_x_MODE)?" % (npkts, cfg.label()))
        if nbytes:
            return ("Received %d bytes on %s but no valid MAVLink (%d parse errors) -- the baud "
                    "rate does not match the radio / FC TELEM port (SER_TELx_BAUD), or the port "
                    "speaks another protocol." % (nbytes, cfg.label(), nerr))
        if cfg.connection_type == "udp":
            return ("No MAVLink on UDP %s:%d. On the Jetson, MAVROS gcs_url must send to this "
                    "laptop's IP and port %d (nidar_config/config/hardware.yaml: fcu.gcs_url); both "
                    "on the same network; firewall open (sudo ufw allow %d/udp)."
                    % (cfg.host or "0.0.0.0", cfg.udp_port, cfg.udp_port, cfg.udp_port))
        if cfg.connection_type == "serial":
            return ("No data at all on %s. Check the air unit / radio is powered and bound, the "
                    "data cable TX/RX are not swapped, and the FC TELEM port is enabled for MAVLink."
                    % cfg.label())
        return "No MAVLink heartbeat on %s." % cfg.label()

    def close(self) -> None:
        self._stop.set()
        mav, self._mav = self._mav, None
        for th in self._threads:
            if th.is_alive() and th is not threading.current_thread():
                th.join(timeout=1.5)
        self._threads = []
        if mav is not None:
            try:
                mav.close()
            except Exception:
                pass
        for waiter in self._acks:
            waiter["event"].set()
        self._acks = []

    # -- threads -------------------------------------------------------------------------------

    def _heartbeat_tx(self) -> None:
        while not self._stop.wait(1.0):
            mav = self._mav
            if mav is None:
                continue
            try:
                with self._tx_lock:
                    mav.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass

    def _reader(self) -> None:
        while not self._stop.is_set():
            mav = self._mav
            if mav is None:
                return
            try:
                msg = mav.recv_match(blocking=True, timeout=0.5)
            except Exception as e:  # cable pulled / radio reset: reopen until close()
                self._link_lost(e)
                continue
            if msg is None:
                continue
            try:
                self._handle(msg)
            except Exception:
                pass  # one malformed message must never stop telemetry

    def _link_lost(self, e: BaseException) -> None:
        with self._lock:
            self.t["link_error"] = str(e)
        if self.on_link_event:
            self.on_link_event("WARN", "MAVLINK LINK LOST (%s) — RECONNECTING" % e)
        old = self._mav
        try:
            old.close()
        except Exception:
            pass
        while not self._stop.wait(RECONNECT_PERIOD_S):
            try:
                self._mav = self._connect(self.cfg)
                with self._lock:
                    self.t["link_error"] = None
                if self.on_link_event:
                    self.on_link_event("SUCCESS", "MAVLINK LINK REOPENED (%s)" % self.cfg.label())
                return
            except Exception:
                continue

    # -- message handling ----------------------------------------------------------------------

    def _handle(self, msg) -> None:
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            return
        src_sys, src_comp = msg.get_srcSystem(), msg.get_srcComponent()
        now = time.time()
        with self._lock:
            t = self.t
            t["packets"] += 1
            if mtype == "HEARTBEAT":
                if msg.type == MAV_TYPE_GCS:
                    return  # another GCS on the same link
                if msg.autopilot == MAV_AUTOPILOT_INVALID:
                    t["companion_heartbeat"] = now
                    t["companion_compid"] = src_comp
                    return
                first = not t["last_heartbeat"]
                if t["sysid"] is None:
                    t["sysid"], t["compid"] = src_sys, src_comp
                if (src_sys, src_comp) != (t["sysid"], t["compid"]):
                    return  # a second autopilot: keep following the first
                t["last_heartbeat"] = now
                t["autopilot"], t["vehicle_type"] = msg.autopilot, msg.type
                t["base_mode"], t["custom_mode"] = msg.base_mode, msg.custom_mode
                t["armed"] = bool(msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED)
                t["system_status"] = msg.system_status
                t["mode"] = mode_name(msg.autopilot, msg.base_mode, msg.custom_mode)
                if first:
                    threading.Thread(target=self._request_streams, daemon=True).start()
                return
            if src_sys != t["sysid"]:
                if mtype == "STATUSTEXT" or mtype == "COMMAND_ACK":
                    pass  # companions report with the vehicle's sysid, but accept any
                else:
                    return
            if mtype == "ATTITUDE":
                t["attitude"] = (msg.roll, msg.pitch, msg.yaw)
                t["attitude_time"] = now
            elif mtype == "LOCAL_POSITION_NED":
                t["local_ned"] = (msg.x, msg.y, msg.z)
                t["local_vel_ned"] = (msg.vx, msg.vy, msg.vz)
                t["local_time"] = now
            elif mtype == "SYS_STATUS":
                if msg.voltage_battery not in (0, 65535):
                    t["voltage"] = msg.voltage_battery / 1000.0
                if msg.current_battery >= 0:
                    t["current"] = msg.current_battery / 100.0
                if msg.battery_remaining >= 0:
                    t["remaining"] = float(msg.battery_remaining)
                t["sensors_present"] = msg.onboard_control_sensors_present
                t["sensors_enabled"] = msg.onboard_control_sensors_enabled
                t["sensors_health"] = msg.onboard_control_sensors_health
            elif mtype == "BATTERY_STATUS" and msg.id == 0:
                cells = [v for v in msg.voltages if v not in (0, 65535)]
                if cells:
                    t["voltage"] = sum(cells) / 1000.0
                if msg.current_battery >= 0:
                    t["current"] = msg.current_battery / 100.0
                if msg.battery_remaining >= 0:
                    t["remaining"] = float(msg.battery_remaining)
            elif mtype == "DISTANCE_SENSOR":
                t["range_m"] = msg.current_distance / 100.0
                t["range_time"] = now
            elif mtype == "EXTENDED_SYS_STATE":
                t["landed_state"] = msg.landed_state
            elif mtype == "RADIO_STATUS":
                t["radio"] = {"rssi": msg.rssi, "remrssi": msg.remrssi, "noise": msg.noise,
                              "remnoise": msg.remnoise, "rxerrors": msg.rxerrors, "time": now}
        if mtype == "COMMAND_ACK":
            self._on_ack(msg, src_sys, src_comp)
        elif mtype == "STATUSTEXT":
            self._on_statustext(msg, src_sys, src_comp)

    def _on_statustext(self, msg, src_sys: int, src_comp: int) -> None:
        text = msg.text if isinstance(msg.text, str) else msg.text.decode("utf-8", "replace")
        text = text.rstrip("\x00").strip()
        chunk_id = getattr(msg, "id", 0) or 0
        if chunk_id:
            # MAVLink 2 long statustext: chunks of 50 chars, the last one shorter.
            key = (src_sys, src_comp, chunk_id)
            parts = self._text_chunks.setdefault(key, [])
            parts.append(text)
            if len(msg.text) >= 50 and len(parts) < 20:
                return
            text = "".join(self._text_chunks.pop(key, [text]))
        if self.on_statustext and text:
            self.on_statustext(int(msg.severity), text, src_sys, src_comp)

    def _on_ack(self, msg, src_sys: int, src_comp: int) -> None:
        for waiter in list(self._acks):
            if waiter["command"] != msg.command:
                continue
            want = waiter["from_compid"]
            if want == "companion" and src_comp == (self.t.get("compid") or AUTOPILOT_COMPID):
                continue  # the autopilot answering a command meant for the companion
            if isinstance(want, int) and want and src_comp != want:
                continue
            waiter["result"] = int(msg.result)
            waiter["from"] = (src_sys, src_comp)
            waiter["event"].set()

    # -- commands ------------------------------------------------------------------------------

    def _send(self, fn: str, *args) -> bool:
        mav = self._mav
        if mav is None:
            return False
        with self._tx_lock:
            getattr(mav.mav, fn)(*args)
        return True

    def _request_streams(self) -> None:
        """Ask for the local position at 5 Hz and the landed state at 1 Hz on this link (PX4's
        default rates on a radio link are lower). Best effort: a radio-mode link may refuse."""
        for msg_id, hz in ((MSG_ID_LOCAL_POSITION_NED, 5.0), (MSG_ID_EXTENDED_SYS_STATE, 1.0)):
            try:
                self._send("command_long_send", self.t["sysid"], self.t["compid"],
                           MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 1e6 / hz, 0, 0, 0, 0, 0)
            except Exception:
                pass

    def command_long(self, command: int, params: List[float], target_compid: Optional[int] = None,
                     ack_from: Any = None, timeout_s: float = 1.5,
                     retries: int = 3) -> Tuple[bool, Optional[int], str]:
        """COMMAND_LONG with ACK. Returns (accepted, MAV_RESULT or None, text)."""
        if not self.connected or self.t.get("sysid") is None:
            return False, None, "MAVLink link is not connected"
        target_sys = self.t["sysid"]
        target_comp = self.t["compid"] if target_compid is None else target_compid
        p = (list(params) + [0.0] * 7)[:7]
        waiter = {"command": command, "from_compid": ack_from, "event": threading.Event(),
                  "result": None, "from": None}
        self._acks.append(waiter)
        try:
            for confirmation in range(max(1, retries)):
                if not self._send("command_long_send", target_sys, target_comp, command,
                                  confirmation, *p):
                    return False, None, "MAVLink link closed"
                if waiter["event"].wait(timeout_s) and waiter["result"] is not None:
                    res = waiter["result"]
                    return res in (0, 5), res, MAV_RESULT.get(res, str(res))
            return False, None, "no COMMAND_ACK after %d tries" % max(1, retries)
        finally:
            if waiter in self._acks:
                self._acks.remove(waiter)

    def set_px4_mode(self, name: str) -> Tuple[bool, str]:
        if name not in PX4_MODE_IDS:
            return False, "unknown PX4 mode %s" % name
        main, sub = PX4_MODE_IDS[name]
        ok, _res, text = self.command_long(MAV_CMD_DO_SET_MODE,
                                           [MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, main, sub],
                                           ack_from=AUTOPILOT_COMPID)
        return ok, "%s %s" % (name, text)

    def companion_command(self, action_code: int, compid: int,
                          timeout_s: float = 2.0) -> Tuple[bool, str]:
        """MAV_CMD_USER_1 for the onboard mission commander (nidar_hardware/mission_commander.py).

        PX4 forwards it from this link to the companion's link only when MAVLink forwarding is
        enabled on both (MAV_0_FORWARD / MAV_1_FORWARD = 1, see hardware/DEPLOYMENT.md)."""
        ok, res, text = self.command_long(MAV_CMD_USER_1, [float(action_code)],
                                          target_compid=compid, ack_from="companion",
                                          timeout_s=timeout_s, retries=2)
        if res is None:
            return False, ("no answer from the onboard commander over MAVLink (%s). Needs the ROS "
                           "link (Wi-Fi) or MAV_0_FORWARD=1 and MAV_1_FORWARD=1 on the FC." % text)
        return ok, text


def local_ip_towards(host: str, port: int = 11311) -> Optional[str]:
    """This machine's IP on the interface that routes to `host` (what ROS_IP must be)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((host, port))
            return s.getsockname()[0]
        finally:
            s.close()
    except OSError:
        return None
