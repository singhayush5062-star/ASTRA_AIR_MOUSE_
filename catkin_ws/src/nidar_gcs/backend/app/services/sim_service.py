"""SimulationService — the real data source behind the Simulation dashboard.

Replaces MockDataProvider (kept for GCS_MOCK=1). It
  * starts the simulation exactly as a terminal user would (scripts/test_takeoff.sh), stops it
    with the same kill list the orchestrator uses (nidar_bringup/scripts/stop_sim.sh), and
    pauses/resumes Gazebo physics;
  * reads the running stack through RosLink (the ROS bridge process) and shapes it into the
    models the UI already uses (DroneState, SystemHealth, Survivor, MissionEvent, AutonomyInfo);
  * keeps the operator event timeline.

All positions are in the arena `world` frame (the frame of /map_2d, /survivors and the grid),
so the UI's map, drone and survivor coordinates agree with each other and with the grid labels.
"""

from __future__ import annotations

import asyncio
import math
import os
import re
import signal
import subprocess
import time
from typing import Any, AsyncGenerator, Dict, List, Optional

from app.core.config import settings
from app.models.mission import AutonomyInfo, MissionEvent
from app.models.survivor import Survivor, SurvivorPosition
from app.models.telemetry import (
    Attitude, Battery, ConnectionStatus, DroneState, SubsystemHealth, SubsystemStatus, Vec3,
)
from app.services.mission_view import (
    PHASE_ORDER, derive_mission_state, grid_label, mission_phases,
)
from app.services.ros_link import RosLink

# Orchestrator stdout lines worth putting on the timeline (prefix match) ...
ORCH_INFO = (
    "Starting clean FUEL exploration test", "Waiting for MAVROS", "MAVROS Connected",
    "Launching FAST-LIO2", "Waiting for EKF Local Position Lock", "Local Position Locked",
    "Starting Flight Envelope Guard", "Launching Upstream FUEL", "Launching NIDAR mission layer",
    "Arming Drone", "Drone successfully armed", "Switching MAVROS to OFFBOARD",
    "Climbing to takeoff altitude", "Takeoff altitude reached", "Entry Detection Module active",
    "Publishing trigger to start autonomous exploration", "Clean Upstream FUEL Autonomous",
    "Simulation test execution complete",
)
# ... and lines that mean the start failed.
ORCH_ERROR = ("Refusing to launch", "Error: ", "ERROR")

SCENARIOS = {
    # UI scenario id -> arena key in mission_config.yaml. Only the active arena can be flown
    # from the UI today: switching arenas means regenerating the world (apply_mission_config.py).
    "scenario_01": None,  # filled with the active arena name at load time
}

LOG_DEDUP_S = 20.0
# Warnings reach the operator timeline only from the mission-level nodes. FUEL and FAST-LIO log
# their routine per-replan / per-scan diagnostics at WARN level, which would bury everything
# else; errors from any node always get through.
WARN_NODES = ("entry_detection_module", "flight_envelope_guard", "survivor_detector",
              "map_2d_slicer", "lidar_map_2d", "grid_visualizer", "coverage_reporter")
# Planner/SLAM internals report routine replanning failures ("search 1 fail") as ERROR. They
# still reach the timeline, as warnings and at most once a minute per node; the health panel is
# where a genuinely failed planner or SLAM shows up.
CHATTY_NODES = ("exploration_node", "traj_server", "laserMapping")
CHATTY_DEDUP_S = 60.0


def _now_ms() -> float:
    return time.time() * 1000.0


def _load_yaml(path: str, key: str) -> Dict[str, Any]:
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f).get(key, {}) or {}
    except Exception as e:
        print("[SimulationService] could not read %s: %s" % (path, e))
        return {}


def _load_mission_config() -> Dict[str, Any]:
    return _load_yaml(settings.MISSION_CONFIG, "nidar")


class SimulationService:
    def __init__(self) -> None:
        self.link = RosLink()
        self.link.on_log = self._on_log
        self.link.on_survivors = self._on_survivors
        self.link.on_status = self._on_bridge_status

        cfg = _load_mission_config()
        self.active_arena = cfg.get("arena", {}).get("active", "arena")
        SCENARIOS["scenario_01"] = self.active_arena
        arena = cfg.get("arenas", {}).get(self.active_arena, {})
        self.arena_info = {
            "name": self.active_arena,
            "bounds": arena.get("bounds", {"x_min": -7.5, "x_max": 7.5, "y_min": -7.5, "y_max": 7.5}),
            "entry": arena.get("entry", {}).get("center", {"x": 0.0, "y": -7.5}),
            "launch_pad": cfg.get("launch_pad", {}).get("center", {"x": 0.0, "y": -9.5}),
            "scenarios": {k: v for k, v in SCENARIOS.items()},
            # The competition grid survivors are reported in (same file the detector, the RViz
            # overlay and the grid labels use): cell (0, 0) = "A1" at the south-west corner.
            "grid": self._grid_info(),
        }

        self.sim_state = "STOPPED"
        self.scenario = "scenario_01"
        self.proc: Optional[subprocess.Popen] = None
        self.proc_log_path: Optional[str] = None
        self._log_fh = None
        self._log_pos = 0
        self._log_buf = ""
        self.start_time = 0.0
        self._busy = False

        self.events: List[MissionEvent] = []
        self.event_seq = 0
        self._log_seen: Dict[str, float] = {}
        self._reset_run()

    @staticmethod
    def _grid_info() -> Dict[str, Any]:
        g = _load_yaml(settings.ARENA_GRID_CONFIG, "arena_grid")
        return {"originX": float(g.get("origin_x", -7.0)), "originY": float(g.get("origin_y", -7.0)),
                "cellSize": float(g.get("cell_size", 2.0)), "cellsX": int(g.get("cells_x", 7)),
                "cellsY": int(g.get("cells_y", 7))}

    # -- run bookkeeping ---------------------------------------------------------------------

    def _reset_run(self) -> None:
        self.survivors: Dict[int, Survivor] = {}
        self.survivor_cells: Dict[int, str] = {}
        self.last_new_survivor = None   # wall time of the latest new confirmation
        self.arm_sim_time: Optional[float] = None
        self.final_timer: Optional[int] = None
        self.complete = False
        self.aborted = False
        self.furthest_phase: Optional[str] = None
        self.prev: Dict[str, Any] = {}
        self.coverage_milestone = 0
        self.saw_land = False

    def add_event(self, message: str, level: str = "INFO") -> None:
        self.event_seq += 1
        self.events.append(MissionEvent(id="gcs-%d" % self.event_seq, timestamp=_now_ms(),
                                        message=message, level=level))
        self.events = self.events[-300:]

    def events_since(self, seq: int) -> List[MissionEvent]:
        """Events with a sequence number above `seq`, oldest first."""
        return [e for e in self.events if int(e.id.split("-")[1]) > seq]

    # -- bridge callbacks ---------------------------------------------------------------------

    def _on_bridge_status(self, status: str) -> None:
        if status == "connected":
            self.add_event("ROS BRIDGE ONLINE — ATTACHED TO ROS MASTER", "SUCCESS")
        elif status == "stopped" and self.sim_state in ("RUNNING", "PAUSED", "STARTING"):
            self.add_event("ROS BRIDGE LOST THE ROS MASTER", "WARN")

    def _on_log(self, msg: Dict[str, Any]) -> None:
        text = msg.get("msg", "")
        if "[EDM] State Transition" in text:
            return  # reported from /edm/mission_state instead
        level = {"WARN": "WARN", "ERROR": "ERROR", "FATAL": "ERROR"}.get(msg.get("level"), "INFO")
        node = msg.get("node", "").lstrip("/")
        if level == "WARN" and not (node.startswith(WARN_NODES) or "FCU:" in text):
            return
        now = time.time()
        if node.startswith(CHATTY_NODES):
            level = "WARN"
            key, window = node + "|chatty", CHATTY_DEDUP_S
        else:
            key, window = node + "|" + re.sub(r"[0-9.\-]+", "#", text)[:90], LOG_DEDUP_S
        if now - self._log_seen.get(key, 0.0) < window:
            return
        self._log_seen[key] = now
        if len(self._log_seen) > 2000:
            self._log_seen = {k: v for k, v in self._log_seen.items() if now - v < CHATTY_DEDUP_S}
        self.add_event("%s: %s" % (node.upper(), text), level)

    def _on_survivors(self, survivors: List[Dict[str, Any]]) -> None:
        if self.sim_state not in ("STARTING", "RUNNING", "PAUSED"):
            return  # /survivors is latched: RosLink keeps it and _attach() replays it
        for s in survivors:
            sid = int(s["id"])
            label = grid_label(int(s["grid_x"]), int(s["grid_y"]))
            conf = round(100.0 * float(s["confidence"]), 1)
            if sid not in self.survivors:
                self.last_new_survivor = time.time()
                self.add_event("SURVIVOR #%02d CONFIRMED — GRID %s — %.1f%% — (%.2f, %.2f)"
                               % (sid, label, conf, s["x"], s["y"]), "SUCCESS")
                detected = _now_ms()
            else:
                detected = self.survivors[sid].detectedAt
                if self.survivor_cells.get(sid) != label:
                    self.add_event("SURVIVOR #%02d RELOCALISED — GRID %s → %s"
                                   % (sid, self.survivor_cells.get(sid), label), "INFO")
            self.survivor_cells[sid] = label
            self.survivors[sid] = Survivor(
                id=sid, gridLabel=label,
                position=SurvivorPosition(x=s["x"], y=s["y"], z=s["z"]),
                confidence=conf, status="LOCALIZED", detectedAt=detected)
        # Survivors merged away by the detector (duplicate tags) disappear from /survivors.
        current = {int(s["id"]) for s in survivors}
        for sid in [k for k in self.survivors if k not in current]:
            self.add_event("SURVIVOR #%02d MERGED INTO A NEIGHBOURING TAG" % sid, "INFO")
            del self.survivors[sid]
            self.survivor_cells.pop(sid, None)

    # -- controls --------------------------------------------------------------------------------

    def _sim_running_elsewhere(self) -> bool:
        mav = self.link.state.get("mavros") or {}
        return self.link.connected and bool(mav.get("connected"))

    def _proc_alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    async def handle_sim_command(self, action: str, body: Dict[str, Any]) -> Dict[str, Any]:
        if "scenario" in body and body["scenario"]:
            self.scenario = str(body["scenario"])
        if self._busy:
            return {"ok": False, "detail": "busy"}
        self._busy = True
        try:
            if action == "start":
                return await self._start()
            if action == "pause":
                return await self._pause()
            if action == "reset":
                return await self._reset()
            return {"ok": False, "detail": "unknown action %r" % action}
        finally:
            self._busy = False

    async def _start(self) -> Dict[str, Any]:
        if self.sim_state == "PAUSED":
            res = await self.link.command({"cmd": "unpause"})
            if res.get("ok"):
                self.sim_state = "RUNNING"
                self.add_event("SIMULATION RESUMED", "SUCCESS")
            else:
                self.add_event("RESUME FAILED: %s" % res.get("detail"), "ERROR")
            return res
        if self._proc_alive() or self.sim_state in ("STARTING", "RUNNING"):
            self.add_event("SIMULATION ALREADY %s" % self.sim_state, "WARN")
            return {"ok": False, "detail": "already %s" % self.sim_state}
        if self.scenario not in SCENARIOS:
            self.add_event("%s IS NOT INSTALLED — AVAILABLE: SCENARIO_01 (%s)"
                           % (self.scenario.upper(), self.active_arena.upper()), "ERROR")
            return {"ok": False, "detail": "scenario not installed"}
        if self._sim_running_elsewhere():
            self._attach("ATTACHED TO A SIMULATION ALREADY RUNNING")
            return {"ok": True, "detail": "attached"}

        self._reset_run()
        log_dir = os.path.join(settings.NIDAR_ROOT, "logs", "gcs")
        os.makedirs(log_dir, exist_ok=True)
        self.proc_log_path = os.path.join(log_dir, "sim_%s.log" % time.strftime("%Y%m%d_%H%M%S"))
        log_fh = open(self.proc_log_path, "wb")
        gui = "true" if settings.SIM_GAZEBO_GUI else "false"
        # New session: the orchestrator and every roslaunch it backgrounds share one process
        # group, so RESET can signal the whole run at once. The backend's own Python packages
        # are on sys.path only (run.py), so os.environ is the plain ROS environment.
        self.proc = subprocess.Popen(["bash", settings.SIM_START_SCRIPT, gui],
                                     cwd=settings.NIDAR_ROOT, stdout=log_fh,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                     start_new_session=True, env=dict(os.environ))
        log_fh.close()
        self._log_fh = open(self.proc_log_path, "r", errors="replace")
        self._log_buf = ""
        self.start_time = time.time()
        self.sim_state = "STARTING"
        self.add_event("SIMULATION START — %s (%s) — GAZEBO + PX4 SITL + FAST-LIO2 + FUEL"
                       % (self.scenario.upper(), self.active_arena), "INFO")
        self.add_event("RUN LOG: %s" % os.path.relpath(self.proc_log_path, settings.NIDAR_ROOT),
                       "INFO")
        return {"ok": True, "detail": "starting", "log": self.proc_log_path}

    async def _pause(self) -> Dict[str, Any]:
        if self.sim_state != "RUNNING":
            return {"ok": False, "detail": "not running"}
        res = await self.link.command({"cmd": "pause"})
        if res.get("ok"):
            self.sim_state = "PAUSED"
            self.add_event("SIMULATION PAUSED — GAZEBO PHYSICS FROZEN (PX4 LOCKSTEP HOLDS)", "WARN")
        else:
            self.add_event("PAUSE FAILED: %s" % res.get("detail"), "ERROR")
        return res

    async def _reset(self) -> Dict[str, Any]:
        self.sim_state = "RESETTING"
        self.add_event("SIMULATION RESET — STOPPING ALL SIMULATION PROCESSES", "WARN")
        if self._proc_alive():
            try:
                os.killpg(self.proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            for _ in range(50):
                if self.proc.poll() is not None:
                    break
                await asyncio.sleep(0.1)
            if self.proc.poll() is None:
                try:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        try:
            stopper = await asyncio.create_subprocess_exec(
                "bash", settings.SIM_STOP_SCRIPT, stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL, env=dict(os.environ))
            await asyncio.wait_for(stopper.wait(), 60.0)
        except Exception as e:
            self.add_event("STOP SCRIPT FAILED: %s" % e, "ERROR")
        await self.link.restart()
        self.proc = None
        if self._log_fh:
            self._log_fh.close()
            self._log_fh = None
        self._reset_run()
        self.sim_state = "STOPPED"
        self.add_event("SIMULATION STOPPED", "WARN")
        return {"ok": True}

    def _attach(self, message: str) -> None:
        """Adopt a run this GCS did not start (or started before a GCS restart)."""
        self._reset_run()
        self.sim_state = "RUNNING"
        self.add_event(message, "SUCCESS")
        self._on_survivors(self.link.survivors)

    def trigger_abort(self) -> None:
        """Operator abort: land in place now (PX4 AUTO.LAND)."""
        if not self.link.connected:
            self.add_event("ABORT IGNORED — NO SIMULATION RUNNING", "WARN")
            return
        self.aborted = True
        self.add_event("!!! EMERGENCY ABORT — PX4 AUTO.LAND COMMANDED !!!", "ERROR")
        asyncio.get_event_loop().create_task(self._abort_land())

    async def _abort_land(self) -> None:
        res = await self.link.command({"cmd": "land"})
        if not res.get("ok"):
            self.add_event("ABORT LAND COMMAND FAILED: %s" % res.get("detail"), "ERROR")

    # -- periodic update -----------------------------------------------------------------------

    def _tail_orchestrator(self) -> None:
        if self._log_fh is None:
            return
        chunk = self._log_fh.read()
        if not chunk:
            return
        self._log_buf += chunk
        lines = self._log_buf.split("\n")
        self._log_buf = lines.pop()
        for line in lines:
            line = line.strip()
            if not line or line.startswith("[mission]"):
                continue
            if any(line.startswith(p) for p in ORCH_ERROR):
                self.add_event("ORCHESTRATOR: %s" % line[:300], "ERROR")
            elif any(line.startswith(p) for p in ORCH_INFO):
                self.add_event(line[:300].upper(), "INFO")

    def _update(self) -> None:
        now = time.time()
        self._tail_orchestrator()
        st = self.link.state if self.link.connected else {}
        mav = st.get("mavros") or {}

        # process lifecycle
        if self.proc is not None and self.proc.poll() is not None:
            rc = self.proc.returncode
            self.proc = None
            if self.sim_state == "STARTING":
                self.sim_state = "ERROR"
                self.add_event("SIMULATION FAILED TO START (orchestrator exit %d) — see %s"
                               % (rc, os.path.relpath(self.proc_log_path or "", settings.NIDAR_ROOT)),
                               "ERROR")
            else:
                self.add_event("ORCHESTRATOR FINISHED (exit %d) — SIMULATION STILL UP; RESET TO STOP"
                               % rc, "INFO" if rc == 0 else "WARN")

        # simulation state
        if self.sim_state == "STARTING":
            if mav.get("connected"):
                self.sim_state = "RUNNING"
            elif now - self.start_time > settings.SIM_START_TIMEOUT_S:
                self.sim_state = "ERROR"
                self.add_event("SIMULATION START TIMED OUT AFTER %.0f s" % settings.SIM_START_TIMEOUT_S,
                               "ERROR")
        elif self.sim_state in ("RUNNING", "PAUSED"):
            paused = st.get("paused")
            if self.sim_state == "RUNNING" and paused is True:
                self.sim_state = "PAUSED"
            elif self.sim_state == "PAUSED" and paused is False:
                self.sim_state = "RUNNING"
            if not self.link.connected and self.link.bridge_status != "connected" \
                    and not self._proc_alive():
                self.sim_state = "STOPPED"
                self.add_event("SIMULATION STOPPED (ROS MASTER GONE)", "WARN")
        elif self.sim_state in ("STOPPED", "ERROR") and not self._proc_alive() \
                and self._sim_running_elsewhere():
            self._attach("ATTACHED TO A SIMULATION STARTED OUTSIDE THE GCS")

        if not st or self.sim_state not in ("STARTING", "RUNNING", "PAUSED"):
            return  # nothing to narrate until this GCS owns or has attached to a run
        p = self.prev

        def edge(key: str, value: Any) -> bool:
            changed = p.get(key) != value
            p[key] = value
            return changed

        if not p.get("gazebo") and self.link.age("clock") < 2.0:
            p["gazebo"] = True   # once per run: /clock pauses with physics and would re-trigger
            self.add_event("GAZEBO SIMULATION CONNECTED", "SUCCESS")
        had_px4 = bool(p.get("px4"))
        if edge("px4", bool(mav.get("connected"))) and (p["px4"] or had_px4):
            self.add_event("PX4 SITL HEARTBEAT LOCKED" if p["px4"] else "PX4 SITL HEARTBEAT LOST",
                           "SUCCESS" if p["px4"] else "ERROR")
        if edge("fastlio", self.link.age("fastlio") < 1.0) and p["fastlio"]:
            self.add_event("FAST-LIO2 TRACKING", "SUCCESS")
        had_armed = "armed" in p
        if edge("armed", bool(mav.get("armed"))) and had_armed:
            self.add_event("VEHICLE ARMED" if p["armed"] else "VEHICLE DISARMED",
                           "SUCCESS" if p["armed"] else "INFO")
        if mav.get("mode") and not str(mav.get("mode")).startswith("CMODE") \
                and edge("mode", mav.get("mode")):
            self.add_event("PX4 MODE → %s" % mav.get("mode"), "INFO")
        edm = st.get("edm_state")
        if edm and edge("edm", edm):
            self.add_event("MISSION STATE → %s" % edm, "INFO")
        if edm in ("DESCEND", "LAND"):
            self.saw_land = True
        if p.get("map") is None and self.link.map is not None:
            p["map"] = True
            self.add_event("LIVE 2D MAP PUBLISHING (/map_2d)", "SUCCESS")

        # mission clock: sim time since arming, frozen at completion
        sim_time = st.get("sim_time")
        if mav.get("armed") and self.arm_sim_time is None and sim_time:
            self.arm_sim_time = sim_time

        cov = st.get("coverage")
        if cov is not None and cov >= self.coverage_milestone + 10.0:
            self.coverage_milestone = int(cov // 10) * 10
            self.add_event("COVERAGE %d%%" % self.coverage_milestone, "INFO")
        if edge("explore_done", bool(st.get("exploration_done"))) and p["explore_done"]:
            self.add_event("EXPLORATION COMPLETE — COVERAGE %.1f%%" % (cov or 0.0), "SUCCESS")

        if not self.complete and self.saw_land and mav.get("connected") and not mav.get("armed"):
            self.complete = True
            self.final_timer = self.mission_timer()
            self.add_event("MISSION COMPLETE — %d SURVIVOR(S) — %02d:%02d — LANDED ON PAD"
                           % (len(self.survivors), self.final_timer // 60, self.final_timer % 60),
                           "SUCCESS")

        ms = self.mission_state()
        if ms in PHASE_ORDER:
            if self.furthest_phase is None or PHASE_ORDER.index(ms) > PHASE_ORDER.index(self.furthest_phase):
                self.furthest_phase = ms

    async def run(self) -> None:
        self.link.start()
        while True:
            try:
                self._update()
            except Exception as e:  # the loop must survive anything a run throws at it
                print("[SimulationService] update error: %r" % e)
            await asyncio.sleep(0.1)

    # -- views ----------------------------------------------------------------------------------

    def mission_timer(self) -> int:
        if self.final_timer is not None:
            return self.final_timer
        sim_time = self.link.state.get("sim_time") if self.link.connected else None
        if self.arm_sim_time is None or not sim_time:
            return 0
        return max(0, int(sim_time - self.arm_sim_time))

    def mission_state(self) -> str:
        st = self.link.state if self.link.connected else {}
        mav = st.get("mavros") or {}
        since = None if self.last_new_survivor is None else time.time() - self.last_new_survivor
        return derive_mission_state(self.sim_state, st.get("edm_state"), bool(mav.get("armed")),
                                    self.complete, self.aborted, len(self.survivors), since)

    def drone(self) -> DroneState:
        st = self.link.state if self.link.connected else {}
        mav = st.get("mavros") or {}
        pos, vel, att, bat = st.get("position"), st.get("velocity"), st.get("attitude"), st.get("battery")
        pct = None if not bat or bat.get("percentage") is None else round(100.0 * bat["percentage"], 1)
        return DroneState(
            id="NIDAR-SIM",
            mode=mav.get("mode") or ("--" if not st else "IDLE"),
            position=Vec3(x=pos[0], y=pos[1], z=pos[2]) if pos else Vec3(),
            velocity=Vec3(x=vel[0], y=vel[1], z=vel[2]) if vel else Vec3(),
            attitude=Attitude(roll=att[0], pitch=att[1], yaw=att[2]) if att else Attitude(),
            battery=Battery(voltage=(bat or {}).get("voltage"), current=(bat or {}).get("current"),
                            percentage=pct),
            connectionStatus=ConnectionStatus.CONNECTED if mav.get("connected") else ConnectionStatus.DISCONNECTED,
            timestamp=_now_ms(),
        )

    def health(self) -> Dict[str, SubsystemHealth]:
        L = self.link
        st = L.state if L.connected else {}
        mav = st.get("mavros") or {}
        t = _now_ms()

        def h(name: str, status: SubsystemStatus) -> SubsystemHealth:
            return SubsystemHealth(name=name, status=status, lastUpdate=t)

        def fresh(key: str, ok: float, degraded: float, good: SubsystemStatus) -> SubsystemStatus:
            a = L.age(key)
            if a < ok:
                return good
            if a < degraded:
                return SubsystemStatus.DEGRADED
            return SubsystemStatus.OFFLINE

        planner = SubsystemStatus.OFFLINE
        if L.age("planner") < 1.0:
            planner = SubsystemStatus.RUNNING
        elif L.age("guard") < 2.0:
            planner = SubsystemStatus.READY
        return {
            "px4": h("PX4", SubsystemStatus.CONNECTED if mav.get("connected") else SubsystemStatus.OFFLINE),
            "ros2": h("ROS", SubsystemStatus.CONNECTED if L.connected else SubsystemStatus.OFFLINE),
            "fastlio2": h("FAST-LIO2", fresh("fastlio", 0.5, 3.0, SubsystemStatus.TRACKING)),
            "lidar": h("LiDAR", fresh("lidar", 1.0, 3.0, SubsystemStatus.ONLINE)),
            "imu": h("IMU", fresh("imu", 1.0, 3.0, SubsystemStatus.ONLINE)),
            "camera": h("Camera", fresh("camera", 1.0, 3.0, SubsystemStatus.ONLINE)),
            "yolo": h("YOLO", fresh("yolo", 2.0, 6.0, SubsystemStatus.RUNNING)),
            "planner": h("Planner", planner),
        }

    def autonomy(self) -> AutonomyInfo:
        st = self.link.state if self.link.connected else {}
        ms = self.mission_state()
        cov = st.get("coverage")
        n = len(self.survivors)
        target, dist = "—", 0.0
        pos, goal = st.get("position"), st.get("goal")
        if ms in ("EXPLORE", "SURVIVOR_DETECTED", "CONTINUE_EXPLORE") and goal:
            target = "FRONTIER (%.1f, %.1f)" % (goal[0], goal[1])
            if pos:
                dist = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        elif ms in ("RETURN", "LAND", "COMPLETE"):
            pad = self.arena_info["launch_pad"]
            target = "LAUNCH PAD (%.1f, %.1f)" % (pad["x"], pad["y"])
            if pos:
                dist = math.hypot(pad["x"] - pos[0], pad["y"] - pos[1])
        elif ms == "LOCALIZATION":
            e = self.arena_info["entry"]
            target = "ARENA ENTRY (%.1f, %.1f)" % (e["x"], e["y"])
            if pos:
                dist = math.hypot(e["x"] - pos[0], e["y"] - pos[1])
        cov_s = "" if cov is None else " — coverage %.1f%%" % cov
        reasons = {
            "IDLE": "Simulation stopped. Press START to launch Gazebo + PX4 SITL + FAST-LIO2 + FUEL.",
            "INIT": self._init_reason(),
            "TAKEOFF": "Armed in OFFBOARD — climbing to cruise altitude",
            "LOCALIZATION": "Entry detection: locating and crossing the arena door",
            "EXPLORE": "FUEL frontier exploration%s" % cov_s,
            "SURVIVOR_DETECTED": "Survivor confirmed and tagged on the grid%s" % cov_s,
            "CONTINUE_EXPLORE": "Exploring — %d survivor(s) tagged%s" % (n, cov_s),
            "RETURN": "Exploration finished — returning to the launch pad",
            "LAND": "Final approach and landing on the pad",
            "COMPLETE": "Mission complete — %d survivor(s) tagged%s" % (n, cov_s),
            "ABORT": "Operator abort — PX4 AUTO.LAND",
            "FAILSAFE": "Simulation failed to start — check the run log",
        }
        return AutonomyInfo(state=ms, planner="FUEL", localization="FAST-LIO2", currentTarget=target,
                            distanceToTarget=round(dist, 2), reason=reasons.get(ms, ""))

    def _init_reason(self) -> str:
        waiting = []
        st = self.link.state if self.link.connected else {}
        if not self.link.connected:
            waiting.append("ROS master")
        elif not (st.get("mavros") or {}).get("connected"):
            waiting.append("PX4 SITL")
        else:
            if self.link.age("fastlio") > 1.0:
                waiting.append("FAST-LIO2")
            if self.link.age("guard") > 2.0:
                waiting.append("FUEL + guard")
            if not st.get("edm_state"):
                waiting.append("mission manager")
        return "Bringing up the stack — waiting for %s" % ", ".join(waiting) if waiting \
            else "Stack up — arming"

    def get_state(self) -> Dict[str, Any]:
        mav = (self.link.state.get("mavros") or {}) if self.link.connected else {}
        ms = self.mission_state()
        return {
            "mode": "SIMULATION",
            "drone": self.drone(),
            "health": self.health(),
            "mission_state": ms,
            "mission_timer": self.mission_timer(),
            "mission_phases": mission_phases(ms, len(self.survivors), self.furthest_phase),
            "autonomy": self.autonomy(),
            "survivors": sorted(self.survivors.values(), key=lambda s: s.id),
            "events": self.events[-200:],
            "gazebo_connected": self.link.age("clock") < 2.0 or self.sim_state == "PAUSED",
            "px4_sitl_connected": bool(mav.get("connected")),
            "ros2_connected": self.link.connected,
            "sim_state": self.sim_state,
            "scenario": self.scenario,
            "coverage": (self.link.state.get("coverage") if self.link.connected else None),
        }

    # -- map & camera ------------------------------------------------------------------------------

    def get_map(self) -> Optional[Dict[str, Any]]:
        m = self.link.map
        if m is None:
            return None
        return {"meta": m["meta"], "encoding": m.get("encoding", "raw"), "rle": m.get("rle"),
                "data": m.get("data"), "frame": m.get("frame"),
                "timestamp": self.link.map_time * 1000.0}

    def get_bboxes(self) -> List[Dict[str, Any]]:
        """Not used for the simulation: its boxes are drawn into the MJPEG frames themselves."""
        return []

    def camera_live(self) -> bool:
        return self.link.connected and self.link.age("camera") < 2.0

    def camera_status(self) -> Dict[str, Any]:
        st = self.link.state
        live = self.camera_live()
        return {
            "camera_source": "Gazebo SITL — /camera/image_raw",
            "fc_host": "localhost (simulation)",
            "stream_url": "http://localhost:%d/api/camera/stream" % settings.PORT,
            "camera_status": "CONNECTED" if live else "OFFLINE",
            "receiving_frames": live,
            "frame_fps": st.get("camera_fps", 0.0) if live else 0,
            "model_status": "RUNNING ONBOARD (nidar_perception)",
            "model_name": "PERSON_DETECTION_MODEL_V3 (YOLOv8)",
            "model_fps": st.get("detect_fps", 0.0) if live else 0,
            "inference_latency_ms": st.get("detect_latency_ms") or 0,
            "last_frame_timestamp": self.link.frame_time,
            "detected_persons": st.get("detect_count", 0) if live else 0,
        }

    async def mjpeg(self) -> AsyncGenerator[bytes, None]:
        self.link.camera_client(+1)
        try:
            last = -1
            idle = 0.0
            while True:
                ev = self.link.frame_event
                if self.link.frame_seq == last and ev is not None:
                    try:
                        await asyncio.wait_for(ev.wait(), 1.0)
                    except asyncio.TimeoutError:
                        idle += 1.0
                        if idle > 10.0:
                            return  # camera gone; the UI falls back to its offline panel
                        continue
                idle = 0.0
                jpg = self.link.frame_jpeg
                last = self.link.frame_seq
                if jpg:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                           + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        finally:
            self.link.camera_client(-1)
