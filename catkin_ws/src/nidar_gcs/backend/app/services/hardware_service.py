"""HardwareService — the data source and command path behind the Hardware dashboard.

The real drone reaches the GCS laptop over up to two links at once:

  MAVLink   T12 RC data link / SiK radio / FC USB (serial), or Wi-Fi (UDP from MAVROS on the
            Jetson). Flight-controller telemetry -- mode, armed, attitude, local position,
            battery -- and PX4 commands. Works with nothing but a radio.
  ROS       the Jetson's ROS master over Wi-Fi, read by the same bridge process the simulation
            uses (scripts/gcs_ros_bridge.py, "hardware" profile: light topics only). Adds what
            only exists onboard: the world-frame pose, /map_2d, /survivors, the mission state,
            the onboard camera with detections, per-subsystem health and the onboard mission
            commander (TAKEOFF/RTL with pre-flight checks).

With only a radio link, the onboard commander also reports mission state and survivors as short
STATUSTEXT messages (parse_nidar_text below; sender: nidar_hardware/scripts/mission_commander.py),
so the dashboard still follows the mission.

Commands (TAKEOFF / RTL / LAND / ABORT) go over ROS when it is up, otherwise over MAVLink:
LAND/ABORT straight to PX4 (AUTO.LAND), TAKEOFF/RTL as MAV_CMD_USER_1 to the onboard commander.
TAKEOFF is never sent to PX4 directly: arming an autonomous vehicle whose autonomy stack has not
been checked is exactly what the commander's pre-flight checks exist to prevent. RTL is the
mission's own return (breadcrumb retrace out of the arena door), never PX4 AUTO.RTL, which would
fly a straight line through the arena walls.
"""

from __future__ import annotations

import asyncio
import importlib.util
import math
import re
import time
from typing import Any, Dict, List, Optional, Tuple
from urllib.parse import urlparse

from app.core.config import settings
from app.models.mission import AutonomyInfo, MissionEvent
from app.models.survivor import Survivor, SurvivorPosition
from app.models.telemetry import (
    Attitude, Battery, ConnectionStatus, DroneState, SubsystemHealth, SubsystemStatus, Vec3,
)
from app.services.mavlink_service import (
    HEARTBEAT_TIMEOUT_S, LinkConfig, MavlinkLink, local_ip_towards,
)
from app.services.mission_view import (
    PHASE_ORDER, derive_mission_state, grid_label, mission_phases,
)
from app.services.ros_link import RosLink
from app.services.sim_service import CHATTY_DEDUP_S, CHATTY_NODES, LOG_DEDUP_S, WARN_NODES, \
    _load_mission_config
from app.services.vehicle_frames import WorldFrame, optional_round

# Companion -> GCS STATUSTEXT vocabulary (kept short: one 50-char MAVLink chunk).
#   NIDAR ST <EDM state>                       mission state change
#   NIDAR SV <id> <grid> <conf%> <x> <y>       survivor confirmed (world frame, metres)
#   NIDAR RDY 1 | NIDAR RDY 0 <reason>         ready for TAKEOFF / why not
NIDAR_RE = re.compile(r"^NIDAR\s+(ST|SV|RDY)\s*(.*)$")

# MAVLink companion command codes (MAV_CMD_USER_1 param1), shared with mission_commander.py.
COMPANION_ACTIONS = {"takeoff": 1, "rtl": 2, "land": 3}

# SYS_STATUS sensor bits used for the IMU row when only MAVLink is available.
SENSOR_GYRO_ACCEL = 0x01 | 0x02

STATUSTEXT_LEVEL = {0: "ERROR", 1: "ERROR", 2: "ERROR", 3: "ERROR", 4: "WARN", 5: "INFO",
                    6: "INFO", 7: "INFO"}


def parse_nidar_text(text: str) -> Optional[Dict[str, Any]]:
    m = NIDAR_RE.match(text.strip())
    if not m:
        return None
    kind, rest = m.group(1), m.group(2).split()
    try:
        if kind == "ST" and rest:
            return {"kind": "state", "state": rest[0].upper()}
        if kind == "SV" and len(rest) >= 5:
            return {"kind": "survivor", "id": int(rest[0]), "grid": rest[1].upper(),
                    "confidence": float(rest[2]), "x": float(rest[3]), "y": float(rest[4])}
        if kind == "RDY" and rest:
            return {"kind": "ready", "ready": rest[0] == "1", "reason": " ".join(rest[1:])}
    except ValueError:
        return None
    return None


def _now_ms() -> float:
    return time.time() * 1000.0


_ROS_AVAILABLE: Optional[bool] = None


def _ros_available() -> bool:
    """The bridge needs a ROS Noetic python environment (start_gcs.sh sources it)."""
    global _ROS_AVAILABLE
    if _ROS_AVAILABLE is None:
        _ROS_AVAILABLE = importlib.util.find_spec("rosgraph") is not None
    return _ROS_AVAILABLE


class HardwareService:
    def __init__(self) -> None:
        cfg = _load_mission_config()
        self.frame = WorldFrame.from_mission_config(cfg)
        active = cfg.get("arena", {}).get("active", "arena")
        arena = cfg.get("arenas", {}).get(active, {})
        self.pad = cfg.get("launch_pad", {}).get("center", {"x": 0.0, "y": -9.5})
        self.entry = arena.get("entry", {}).get("center", {"x": 0.0, "y": -7.5})

        self.link = MavlinkLink(gcs_sysid=settings.GCS_SYSID, gcs_compid=settings.GCS_COMPID,
                                on_statustext=self._on_statustext,
                                on_link_event=lambda lvl, txt: self.add_event(txt, lvl))
        self.ros = RosLink()
        self.ros.on_log = self._on_log
        self.ros.on_survivors = self._on_ros_survivors
        self.ros.on_status = self._on_ros_status
        self.ros_master: Optional[str] = None

        self.state = "DISCONNECTED"      # operator-facing: DISCONNECTED|CONNECTING|CONNECTED|ERROR
        self.error: Optional[str] = None
        self.request: Optional[Dict[str, Any]] = None
        self.config: Dict[str, Any] = {}
        self.events: List[MissionEvent] = []
        self.event_seq = 0
        self._log_seen: Dict[str, float] = {}
        self._busy = False
        self._ros_warned = False
        self._reset_flight()

    # -- bookkeeping ------------------------------------------------------------------------------

    def _reset_flight(self) -> None:
        self.survivors: Dict[int, Survivor] = {}
        self.mavlink_edm: Optional[str] = None
        self.mavlink_edm_time = 0.0
        self.onboard_ready: Optional[Tuple[bool, str]] = None
        self.arm_time: Optional[float] = None
        self.final_timer: Optional[int] = None
        self.was_armed = False
        self.complete = False
        self.aborted = False
        self.last_new_survivor: Optional[float] = None
        self.furthest_phase: Optional[str] = None
        self.prev: Dict[str, Any] = {}

    def add_event(self, message: str, level: str = "INFO") -> None:
        self.event_seq += 1
        self.events.append(MissionEvent(id="hw-%d" % self.event_seq, timestamp=_now_ms(),
                                        message=message, level=level))
        self.events = self.events[-300:]

    def events_since(self, seq: int) -> List[MissionEvent]:
        return [e for e in self.events if int(e.id.split("-")[1]) > seq]

    @property
    def connected(self) -> bool:
        return self.state == "CONNECTED"

    def _ros_state(self) -> Dict[str, Any]:
        return self.ros.state if self.ros.connected else {}

    def _armed(self) -> bool:
        if self.link.connected:
            return bool(self.link.t.get("armed"))
        mav = self._ros_state().get("mavros") or {}
        return bool(mav.get("armed"))

    def _fcu_connected(self) -> bool:
        return self.link.connected or bool((self._ros_state().get("mavros") or {}).get("connected"))

    # -- connect / disconnect ------------------------------------------------------------------

    async def connect(self, req: Dict[str, Any]) -> Tuple[bool, str]:
        if self._busy:
            return False, "a connection attempt is already in progress"
        self._busy = True
        try:
            await self._disconnect_links()
            self.request = dict(req)
            ctype = str(req.get("connection_type") or "serial").lower()
            self.config = {k: req.get(k) for k in ("drone_name", "sys_id", "home_lat", "home_lon",
                                                     "connection_type", "serial_port", "baud_rate",
                                                     "host", "udp_port", "tcp_port")}
            self.state, self.error = "CONNECTING", None
            self._reset_flight()

            if ctype == "simulator":
                # The NIDAR SITL on this machine: PX4's GCS MAVLink port + the local ROS master.
                cfg = LinkConfig("udp", host="0.0.0.0", udp_port=settings.SITL_MAVLINK_UDP_PORT)
                master = "http://localhost:11311"
            else:
                cfg = LinkConfig(ctype, serial_port=req.get("serial_port"),
                                 baud_rate=int(req.get("baud_rate") or 57600),
                                 host=(req.get("host") or ("0.0.0.0" if ctype == "udp" else "127.0.0.1")),
                                 udp_port=int(req.get("udp_port") or 14550),
                                 tcp_port=int(req.get("tcp_port") or 5760))
                master = None
            self.add_event("CONNECTING — %s" % cfg.label(), "INFO")

            loop = asyncio.get_event_loop()
            ok, msg = await loop.run_in_executor(
                None, self.link.open, cfg, settings.MAVLINK_CONNECT_TIMEOUT_S,
                bool(req.get("force_connect")))
            if not ok:
                self.state, self.error = "ERROR", msg
                self.add_event("CONNECTION FAILED — %s" % msg, "ERROR")
                return False, msg

            self.state = "CONNECTED"
            # Attaching to a drone that is already flying (GCS restarted mid-mission) is not a
            # new flight: only a later disarmed -> armed edge starts one.
            if self._armed():
                self.was_armed, self.arm_time = True, time.time()
            self.add_event(msg.upper() if len(msg) < 120 else msg, "SUCCESS")
            self._start_ros(master)
            return True, msg
        finally:
            self._busy = False

    def _master_uri(self) -> Optional[str]:
        want = (settings.DRONE_ROS_MASTER_URI or "").strip()
        if not want:
            return None
        if want.lower() != "auto":
            return want
        ip = self.link.peer_ip()
        if ip and ip not in ("127.0.0.1", "localhost"):
            return "http://%s:11311" % ip
        if ip:  # SITL / mavlink-router on this machine
            return "http://localhost:11311"
        return None

    def _start_ros(self, master: Optional[str] = None) -> None:
        master = master or self._master_uri()
        if not master or self.ros.running:
            return
        if not _ros_available():
            if not self._ros_warned:
                self._ros_warned = True
                self.add_event("ROS LINK UNAVAILABLE — this laptop has no ROS Noetic environment; "
                               "map, survivors and onboard camera need it (run the GCS inside the "
                               "dev container). MAVLink telemetry and LAND still work.", "WARN")
            return
        host = urlparse(master).hostname or "localhost"
        local = host in ("localhost", "127.0.0.1")
        env = {"ROS_MASTER_URI": master,
               "NIDAR_GCS_PROFILE": "sim" if local else "hardware",
               "NIDAR_GCS_CAMERA_TOPIC": settings.DRONE_CAMERA_TOPIC}
        if not local:
            ros_ip = settings.DRONE_ROS_IP or local_ip_towards(host)
            if ros_ip:
                env["ROS_IP"] = ros_ip
            # The simulated camera is mounted upside down; the real one is not.
            env["NIDAR_GCS_CAMERA_FLIP"] = "0"
        self.ros.env_overrides = env
        self.ros_master = master
        self.ros.start()
        self.add_event("ROS LINK — WAITING FOR MASTER %s%s"
                       % (master, " (ROS_IP %s)" % env["ROS_IP"] if "ROS_IP" in env else ""), "INFO")

    async def _disconnect_links(self) -> None:
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.link.close)
        if self.ros.running:
            await self.ros.stop()
        self.ros.state, self.ros.map, self.ros.survivors = {}, None, []
        self.ros_master = None

    async def disconnect(self) -> None:
        await self._disconnect_links()
        self.state, self.error = "DISCONNECTED", None
        self.add_event("DRONE DISCONNECTED BY OPERATOR", "WARN")

    async def shutdown(self) -> None:
        await self._disconnect_links()

    # -- link callbacks ----------------------------------------------------------------------

    def _on_ros_status(self, status: str) -> None:
        if status == "connected":
            self.add_event("ROS LINK ONLINE — %s (map, survivors, mission, camera)"
                           % (self.ros_master or "master"), "SUCCESS")
        elif status == "stopped" and self.connected:
            self.add_event("ROS LINK LOST — %s" % (self.ros_master or "master"), "WARN")

    def _on_log(self, msg: Dict[str, Any]) -> None:
        text = msg.get("msg", "")
        if "[EDM] State Transition" in text:
            return
        level = {"WARN": "WARN", "ERROR": "ERROR", "FATAL": "ERROR"}.get(msg.get("level"), "INFO")
        node = msg.get("node", "").lstrip("/")
        if level == "WARN" and not (node.startswith(WARN_NODES + ("mission_commander",))
                                    or "FCU:" in text):
            return
        now = time.time()
        if node.startswith(CHATTY_NODES):
            level, key, window = "WARN", node + "|chatty", CHATTY_DEDUP_S
        else:
            key, window = node + "|" + re.sub(r"[0-9.\-]+", "#", text)[:90], LOG_DEDUP_S
        if now - self._log_seen.get(key, 0.0) < window:
            return
        self._log_seen[key] = now
        self.add_event("%s: %s" % (node.upper(), text), level)

    def _on_ros_survivors(self, survivors: List[Dict[str, Any]]) -> None:
        current = set()
        for s in survivors:
            sid = int(s["id"])
            current.add(sid)
            self._upsert_survivor(sid, grid_label(int(s["grid_x"]), int(s["grid_y"])),
                                  round(100.0 * float(s["confidence"]), 1),
                                  s["x"], s["y"], s.get("z", 0.0))
        for sid in [k for k in self.survivors if k not in current]:
            self.add_event("SURVIVOR #%02d MERGED INTO A NEIGHBOURING TAG" % sid, "INFO")
            del self.survivors[sid]

    def _upsert_survivor(self, sid: int, label: str, conf: float, x: float, y: float,
                         z: float = 0.0) -> None:
        old = self.survivors.get(sid)
        if old is None:
            self.last_new_survivor = time.time()
            self.add_event("SURVIVOR #%02d CONFIRMED — GRID %s — %.1f%% — (%.2f, %.2f)"
                           % (sid, label, conf, x, y), "SUCCESS")
        elif old.gridLabel != label:
            self.add_event("SURVIVOR #%02d RELOCALISED — GRID %s → %s" % (sid, old.gridLabel, label))
        self.survivors[sid] = Survivor(
            id=sid, gridLabel=label, position=SurvivorPosition(x=x, y=y, z=z), confidence=conf,
            status="LOCALIZED", detectedAt=old.detectedAt if old else _now_ms())

    def _on_statustext(self, severity: int, text: str, src_sys: int, src_comp: int) -> None:
        parsed = parse_nidar_text(text)
        if parsed is None:
            src = "FCU" if src_comp == (self.link.t.get("compid") or 1) else "ONBOARD"
            self.add_event("%s: %s" % (src, text), STATUSTEXT_LEVEL.get(severity, "INFO"))
            return
        if parsed["kind"] == "state":
            self.mavlink_edm, self.mavlink_edm_time = parsed["state"], time.time()
        elif parsed["kind"] == "survivor":
            if not self.ros.connected:  # ROS /survivors is authoritative when present
                self._upsert_survivor(parsed["id"], parsed["grid"], parsed["confidence"],
                                      parsed["x"], parsed["y"])
        elif parsed["kind"] == "ready":
            was = self.onboard_ready
            self.onboard_ready = (parsed["ready"], parsed["reason"])
            if was != self.onboard_ready:
                self.add_event("ONBOARD READY FOR TAKEOFF" if parsed["ready"]
                               else "ONBOARD NOT READY — %s" % parsed["reason"],
                               "SUCCESS" if parsed["ready"] else "WARN")

    # -- commands ------------------------------------------------------------------------------

    async def command(self, name: str) -> Dict[str, Any]:
        name = (name or "").lower()
        if name not in ("takeoff", "land", "rtl", "abort"):
            return {"ok": False, "detail": "unknown command %r" % name}
        if not self.connected:
            return {"ok": False, "detail": "drone not connected"}
        self.add_event("OPERATOR COMMAND — %s" % name.upper(),
                       "ERROR" if name == "abort" else "WARN")
        if name == "abort":
            return await self._abort()

        loop = asyncio.get_event_loop()
        tried: List[str] = []
        if self.ros.connected:
            res = await self.ros.command({"cmd": name}, timeout=20.0)
            if res.get("ok"):
                return self._command_done(name, "ROS", res.get("detail") or "accepted")
            tried.append("ROS: %s" % (res.get("detail") or "rejected"))
        if self.link.connected:
            if name == "land":
                ok, text = await loop.run_in_executor(None, self.link.set_px4_mode, "AUTO.LAND")
            else:
                ok, text = await loop.run_in_executor(
                    None, self.link.companion_command, COMPANION_ACTIONS[name],
                    settings.COMPANION_COMPID)
            if ok:
                return self._command_done(name, "MAVLink", text)
            tried.append("MAVLink: %s" % text)
        detail = "; ".join(tried) or "no link can carry this command"
        self.add_event("%s FAILED — %s" % (name.upper(), detail), "ERROR")
        return {"ok": False, "detail": detail}

    def _command_done(self, name: str, via: str, detail: str) -> Dict[str, Any]:
        self.add_event("%s ACCEPTED via %s — %s" % (name.upper(), via, detail), "SUCCESS")
        return {"ok": True, "via": via, "detail": detail}

    async def _abort(self) -> Dict[str, Any]:
        """Land where it is, over every link that is up (PX4 AUTO.LAND)."""
        self.aborted = True
        self.add_event("!!! EMERGENCY ABORT — PX4 AUTO.LAND COMMANDED !!!", "ERROR")
        loop = asyncio.get_event_loop()
        jobs = []
        if self.link.connected:
            jobs.append(loop.run_in_executor(None, self.link.set_px4_mode, "AUTO.LAND"))
        if self.ros.connected:
            jobs.append(self.ros.command({"cmd": "land"}, timeout=8.0))
        if not jobs:
            self.add_event("ABORT COULD NOT BE SENT — NO LINK UP", "ERROR")
            return {"ok": False, "detail": "no link up"}
        results = await asyncio.gather(*jobs, return_exceptions=True)
        oks = [r for r in results if (isinstance(r, tuple) and r[0])
               or (isinstance(r, dict) and r.get("ok"))]
        if not oks:
            self.add_event("ABORT LAND NOT ACKNOWLEDGED: %s" % results, "ERROR")
        return {"ok": bool(oks), "detail": "AUTO.LAND sent on %d link(s)" % len(jobs)}

    # -- periodic update -------------------------------------------------------------------------

    async def run(self) -> None:
        while True:
            try:
                self._update()
            except Exception as e:  # never let one bad sample stop the dashboard
                print("[HardwareService] update error: %r" % e)
            await asyncio.sleep(0.1)

    def _update(self) -> None:
        if self.state not in ("CONNECTED",):
            return
        now = time.time()
        p = self.prev

        def edge(key: str, value: Any) -> bool:
            changed = p.get(key) != value
            p[key] = value
            return changed

        # A Wi-Fi link that came up after CONNECT (auto master discovery needs the first packet).
        if not self.ros.running:
            self._start_ros()

        hb_ok = self.link.connected
        had = "fcu" in p
        if edge("fcu", self._fcu_connected()) and had:
            self.add_event("FLIGHT CONTROLLER LINK RESTORED" if p["fcu"]
                           else "FLIGHT CONTROLLER HEARTBEAT LOST", "SUCCESS" if p["fcu"] else "ERROR")
        mode = self.link.t.get("mode") if hb_ok else (self._ros_state().get("mavros") or {}).get("mode")
        if mode and mode != "--" and edge("mode", mode) and "mode_seen" in p:
            self.add_event("PX4 MODE → %s" % mode)
        p["mode_seen"] = True

        armed = self._armed()
        if edge("armed", armed) and "armed_seen" in p:
            self.add_event("VEHICLE ARMED" if armed else "VEHICLE DISARMED",
                           "SUCCESS" if armed else "INFO")
        p["armed_seen"] = True
        if armed and not self.was_armed:
            # A new flight: fresh clock and phases (survivors stay: /survivors is latched onboard).
            self.was_armed, self.arm_time = True, now
            self.final_timer, self.complete, self.aborted = None, False, False
            self.furthest_phase = None
        elif not armed and self.was_armed:
            self.was_armed = False
            self.final_timer = self.mission_timer()
            self.complete = True
            self.add_event("LANDED AND DISARMED — %d SURVIVOR(S) — %02d:%02d"
                           % (len(self.survivors), self.final_timer // 60, self.final_timer % 60),
                           "SUCCESS")

        edm = self._edm_state()
        if edm and edge("edm", edm):
            self.add_event("MISSION STATE → %s" % edm)
        st = self._ros_state()
        onboard = st.get("onboard") or {}
        if "ready" in onboard and edge("ready", (bool(onboard.get("ready")), onboard.get("reason"))):
            self.onboard_ready = (bool(onboard.get("ready")), onboard.get("reason") or "")
            if not armed:
                self.add_event("ONBOARD READY FOR TAKEOFF" if onboard.get("ready")
                               else "ONBOARD NOT READY — %s" % onboard.get("reason"),
                               "SUCCESS" if onboard.get("ready") else "WARN")
        if p.get("map") is None and self.ros.map is not None:
            p["map"] = True
            self.add_event("LIVE 2D MAP RECEIVED FROM THE DRONE (/map_2d)", "SUCCESS")
        bat = self.drone().battery.percentage
        if bat is not None and bat < 25 and armed and not p.get("bat_warned"):
            p["bat_warned"] = True
            self.add_event("BATTERY LOW — %.0f%%" % bat, "WARN")

        ms = self.mission_state()
        if ms in PHASE_ORDER and (self.furthest_phase is None
                                  or PHASE_ORDER.index(ms) > PHASE_ORDER.index(self.furthest_phase)):
            self.furthest_phase = ms

    # -- views ----------------------------------------------------------------------------------

    def _edm_state(self) -> Optional[str]:
        edm = self._ros_state().get("edm_state")
        if edm:
            return edm
        if self.mavlink_edm and time.time() - self.mavlink_edm_time < 600:
            return self.mavlink_edm
        return None

    def mission_timer(self) -> int:
        if self.final_timer is not None:
            return self.final_timer
        if self.arm_time is None:
            return 0
        return max(0, int(time.time() - self.arm_time))

    def mission_state(self) -> str:
        if self.state != "CONNECTED":
            return "IDLE"
        since = None if self.last_new_survivor is None else time.time() - self.last_new_survivor
        return derive_mission_state("RUNNING", self._edm_state(), self._armed(), self.complete,
                                    self.aborted, len(self.survivors), since)

    def drone(self) -> DroneState:
        name = (self.config.get("drone_name") or "NIDAR-01") if self.config else "NIDAR-01"
        if self.state != "CONNECTED":
            return DroneState(id=name, mode="--", connectionStatus=ConnectionStatus.DISCONNECTED)
        st = self._ros_state()
        t = self.link.snapshot()
        hb_ok = self.link.connected
        pos = vel = att = None
        if st.get("position"):
            pos, vel, att = st.get("position"), st.get("velocity"), st.get("attitude")
        elif hb_ok and t.get("local_ned") and time.time() - t["local_time"] < 3.0:
            pos = self.frame.position_from_ned(*t["local_ned"])
            vel = self.frame.velocity_from_ned(*t["local_vel_ned"])
        if att is None and hb_ok and t.get("attitude"):
            att = self.frame.attitude_from_ned(*t["attitude"])

        mav = st.get("mavros") or {}
        mode = t.get("mode") if hb_ok else (mav.get("mode") or "--")
        bat_ros = st.get("battery") or {}
        if hb_ok and (t.get("voltage") is not None or t.get("remaining") is not None):
            voltage, current, pct = t.get("voltage"), t.get("current"), t.get("remaining")
        else:
            voltage, current = bat_ros.get("voltage"), bat_ros.get("current")
            pct = None if bat_ros.get("percentage") is None else 100.0 * bat_ros["percentage"]

        if hb_ok or mav.get("connected"):
            status = ConnectionStatus.CONNECTED
        elif self.link.is_open or self.ros.connected:
            status = ConnectionStatus.DEGRADED
        else:
            status = ConnectionStatus.DISCONNECTED
        r = optional_round
        return DroneState(
            id=name, mode=mode or "--",
            position=Vec3(x=r(pos[0], 3), y=r(pos[1], 3), z=r(pos[2], 3)) if pos else Vec3(),
            velocity=Vec3(x=r(vel[0], 2), y=r(vel[1], 2), z=r(vel[2], 2)) if vel else Vec3(),
            attitude=Attitude(roll=r(att[0], 4), pitch=r(att[1], 4), yaw=r(att[2], 4)) if att else Attitude(),
            battery=Battery(voltage=r(voltage, 2), current=r(current, 2), percentage=r(pct, 1)),
            connectionStatus=status, timestamp=_now_ms())

    def health(self) -> Dict[str, SubsystemHealth]:
        t_ms = _now_ms()
        L = self.ros
        st = self._ros_state()
        snap = self.link.snapshot()

        def h(name: str, status: SubsystemStatus) -> SubsystemHealth:
            return SubsystemHealth(name=name, status=status, lastUpdate=t_ms)

        def fresh(key: str, ok: float, degraded: float, good: SubsystemStatus) -> SubsystemStatus:
            if not L.connected:
                return SubsystemStatus.UNKNOWN if self.connected else SubsystemStatus.OFFLINE
            a = L.age(key)
            return good if a < ok else SubsystemStatus.DEGRADED if a < degraded else SubsystemStatus.OFFLINE

        hb_age = self.link.heartbeat_age()
        if hb_age < HEARTBEAT_TIMEOUT_S or (st.get("mavros") or {}).get("connected"):
            px4 = SubsystemStatus.CONNECTED
        elif hb_age < 10.0:
            px4 = SubsystemStatus.DEGRADED
        else:
            px4 = SubsystemStatus.OFFLINE

        imu = fresh("imu", 1.0, 3.0, SubsystemStatus.ONLINE)
        if imu == SubsystemStatus.UNKNOWN and self.link.connected and snap.get("sensors_present"):
            ok = (snap["sensors_health"] & SENSOR_GYRO_ACCEL) == SENSOR_GYRO_ACCEL
            imu = SubsystemStatus.ONLINE if ok else SubsystemStatus.DEGRADED

        planner = fresh("planner", 1.0, 1.0, SubsystemStatus.RUNNING)
        if planner == SubsystemStatus.OFFLINE and L.age("guard") < 2.0:
            planner = SubsystemStatus.READY
        return {
            "px4": h("PX4", px4),
            "ros2": h("ROS (JETSON)", SubsystemStatus.CONNECTED if L.connected
                      else SubsystemStatus.OFFLINE),
            "fastlio2": h("FAST-LIO2", fresh("fastlio", 0.5, 3.0, SubsystemStatus.TRACKING)),
            "lidar": h("LiDAR", fresh("lidar", 1.0, 3.0, SubsystemStatus.ONLINE)),
            "imu": h("IMU", imu),
            "camera": h("Camera", fresh("camera", 1.0, 3.0, SubsystemStatus.ONLINE)),
            "yolo": h("YOLO", fresh("yolo", 2.0, 6.0, SubsystemStatus.RUNNING)),
            "planner": h("Planner", planner),
        }

    def autonomy(self) -> AutonomyInfo:
        st = self._ros_state()
        ms = self.mission_state()
        cov = st.get("coverage")
        n = len(self.survivors)
        pos = st.get("position")
        if not pos and self.link.connected and self.link.t.get("local_ned"):
            pos = self.frame.position_from_ned(*self.link.t["local_ned"])
        target, dist = "—", 0.0
        goal = st.get("goal")
        if ms in ("EXPLORE", "SURVIVOR_DETECTED", "CONTINUE_EXPLORE") and goal:
            target = "FRONTIER (%.1f, %.1f)" % (goal[0], goal[1])
            if pos:
                dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        elif ms in ("RETURN", "LAND", "COMPLETE"):
            target = "LAUNCH PAD (%.1f, %.1f)" % (self.pad["x"], self.pad["y"])
            if pos:
                dist = math.hypot(self.pad["x"] - pos[0], self.pad["y"] - pos[1])
        elif ms == "LOCALIZATION":
            target = "ARENA ENTRY (%.1f, %.1f)" % (self.entry["x"], self.entry["y"])
            if pos:
                dist = math.hypot(self.entry["x"] - pos[0], self.entry["y"] - pos[1])
        cov_s = "" if cov is None else " — coverage %.1f%%" % cov
        if self.onboard_ready is None:
            init = ("Waiting for the onboard stack status (ROS link or commander STATUSTEXT)"
                    if not self.ros.connected else "Waiting for the onboard mission commander")
        elif self.onboard_ready[0]:
            init = "Onboard stack READY — press TAKEOFF to start the autonomous mission"
        else:
            init = "Onboard stack NOT READY — %s" % self.onboard_ready[1]
        reasons = {
            "IDLE": "Drone disconnected — CONNECT DRONE to attach to the real vehicle",
            "INIT": init,
            "TAKEOFF": "Armed in OFFBOARD — climbing over the launch pad",
            "LOCALIZATION": "Entry detection: locating and crossing the arena door",
            "EXPLORE": "FUEL frontier exploration%s" % cov_s,
            "SURVIVOR_DETECTED": "Survivor confirmed and tagged on the grid%s" % cov_s,
            "CONTINUE_EXPLORE": "Exploring — %d survivor(s) tagged%s" % (n, cov_s),
            "RETURN": "Returning along the explored path to the launch pad",
            "LAND": "Final approach and landing (PX4 AUTO.LAND)",
            "COMPLETE": "Landed — %d survivor(s) tagged%s" % (n, cov_s),
            "ABORT": "Operator abort — PX4 AUTO.LAND",
            "FAILSAFE": "Failsafe",
        }
        if not self.ros.connected and ms not in ("IDLE",):
            via = "MAVLink only (no ROS link to the Jetson)"
        else:
            via = ""
        reason = reasons.get(ms, "")
        if via and ms != "INIT":
            reason = "%s · %s" % (reason, via)
        return AutonomyInfo(state=ms, planner="FUEL", localization="FAST-LIO2 (Mid-360)",
                            currentTarget=target, distanceToTarget=round(dist, 2), reason=reason)

    def link_info(self) -> Dict[str, Any]:
        t = self.link.snapshot()
        return {
            "state": self.state,
            "error": self.error,
            "mavlink": {"open": self.link.is_open, "connected": self.link.connected,
                        "link": self.link.cfg.label() if self.link.cfg else None,
                        "heartbeat_age": None if math.isinf(self.link.heartbeat_age())
                        else round(self.link.heartbeat_age(), 2),
                        "packets": t.get("packets"), "radio": t.get("radio"),
                        "companion_seen": bool(t.get("companion_heartbeat")),
                        "error": t.get("link_error")},
            "ros": {"master": self.ros_master, "status": self.ros.bridge_status,
                    "connected": self.ros.connected},
        }

    def get_state(self) -> Dict[str, Any]:
        ms = self.mission_state()
        return {
            "drone": self.drone(),
            "health": self.health(),
            "mission_state": ms,
            "mission_timer": self.mission_timer(),
            "mission_phases": mission_phases(ms, len(self.survivors), self.furthest_phase),
            "autonomy": self.autonomy(),
            "survivors": sorted(self.survivors.values(), key=lambda s: s.id),
            "link": self.link_info(),
        }

    # -- map & camera ------------------------------------------------------------------------------

    def get_map(self) -> Optional[Dict[str, Any]]:
        m = self.ros.map
        if m is None:
            return None
        return {"meta": m["meta"], "encoding": m.get("encoding", "raw"), "rle": m.get("rle"),
                "data": m.get("data"), "frame": m.get("frame"),
                "timestamp": self.ros.map_time * 1000.0}

    def camera_live(self) -> bool:
        return self.ros.connected and self.ros.age("camera") < 2.0

    def camera_status(self) -> Dict[str, Any]:
        st = self.ros.state
        live = self.camera_live()
        return {
            "camera_source": "Drone onboard camera (Jetson) — %s" % settings.DRONE_CAMERA_TOPIC,
            "fc_host": urlparse(self.ros_master).hostname if self.ros_master else "--",
            "stream_url": "ros://%s" % settings.DRONE_CAMERA_TOPIC.lstrip("/"),
            "camera_status": "CONNECTED" if live else "OFFLINE",
            "receiving_frames": live,
            "frame_fps": st.get("camera_fps", 0.0) if live else 0,
            "model_status": "RUNNING ONBOARD (nidar_perception)",
            "model_name": "Onboard YOLO (survivor_detector)",
            "model_fps": st.get("detect_fps", 0.0) if live else 0,
            "inference_latency_ms": st.get("detect_latency_ms") or 0,
            "last_frame_timestamp": self.ros.frame_time,
            "detected_persons": st.get("detect_count", 0) if live else 0,
        }

    def mjpeg(self):
        return self.ros.mjpeg()
