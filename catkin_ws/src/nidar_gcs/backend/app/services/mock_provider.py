"""
MockDataProvider — Backend mock data generator.
Runs in a background asyncio task and maintains the full
shared state that REST endpoints and WebSocket streams read from.
"""

from __future__ import annotations

import asyncio
import math
import random
import time
from typing import Any

from app.models.telemetry import (
    DroneState, Vec3, Attitude, Battery, ConnectionStatus, SystemHealth,
    SubsystemHealth, SubsystemStatus, MissionState,
)
from app.models.mission import MissionEvent, AutonomyInfo
from app.models.survivor import Survivor, SurvivorPosition


class MockDataProvider:
    def __init__(self) -> None:
        self._t = 0.0
        self._mission_timer = 0
        self._mission_state: str = MissionState.IDLE
        self._aborted = False
        self._sim_state = "STOPPED"
        self._scenario = "scenario_01"
        self._gazebo = False
        self._px4_sitl = False
        self._ros2 = False
        self._events: list[MissionEvent] = []
        self._survivors: list[Survivor] = []
        self._ev_counter = 0
        self._autonomy = AutonomyInfo()
        self._drone = DroneState()
        self._health = SystemHealth()
        self._map_seed = random.randint(0, 9999)

    # ── State access ─────────────────────────────────────────

    def get_state(self) -> dict[str, Any]:
        return {
            "mode": "MOCK",
            "drone": self._drone,
            "health": {
                k: getattr(self._health, k)
                for k in self._health.model_fields
            },
            "mission_state": self._mission_state,
            "mission_timer": self._mission_timer,
            "autonomy": self._autonomy,
            "survivors": self._survivors,
            "events": self._events[-200:],
            "gazebo_connected": self._gazebo,
            "px4_sitl_connected": self._px4_sitl,
            "ros2_connected": self._ros2,
            "sim_state": self._sim_state,
            "scenario": self._scenario,
        }

    def get_map_data(self) -> list[int]:
        """Return a 80x80 occupancy grid (0=free,100=occupied,-1=unknown)."""
        grid = []
        w, h = 80, 80
        cx, cy = w // 2, h // 2
        walls = set()

        # Outer walls
        for x in range(w):
            walls.add((x, 0)); walls.add((x, h - 1))
        for y in range(h):
            walls.add((0, y)); walls.add((w - 1, y))

        # Interior rooms (deterministic based on seed)
        rng = random.Random(self._map_seed)
        for _ in range(12):
            rx = rng.randint(5, w - 20)
            ry = rng.randint(5, h - 20)
            rw = rng.randint(5, 16)
            rh = rng.randint(5, 12)
            for x in range(rx, min(rx + rw, w)):
                walls.add((x, ry)); walls.add((x, min(ry + rh, h - 1)))
            for y in range(ry, min(ry + rh, h)):
                walls.add((rx, y)); walls.add((min(rx + rw, w - 1), y))
            # Door opening
            side = rng.choice(['T', 'B', 'L', 'R'])
            if side == 'T':
                dx = rng.randint(rx + 1, rx + rw - 2)
                walls.discard((dx, ry))
            elif side == 'B':
                dx = rng.randint(rx + 1, rx + rw - 2)
                walls.discard((dx, min(ry + rh, h - 1)))
            elif side == 'L':
                dy = rng.randint(ry + 1, ry + rh - 2)
                walls.discard((rx, dy))
            else:
                dy = rng.randint(ry + 1, ry + rh - 2)
                walls.discard((min(rx + rw, w - 1), dy))

        for y in range(h):
            for x in range(w):
                if (x, y) in walls:
                    grid.append(100)
                elif abs(x - cx) < 3 and abs(y - cy) < 3:
                    grid.append(0)   # origin free
                else:
                    # unknown in unexplored areas, free in explored radius
                    dist = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
                    explored = min(40, self._mission_timer / 2)
                    grid.append(0 if dist < explored else -1)
        return grid

    def get_bboxes(self) -> list[dict[str, Any]]:
        if not self._survivors:
            return []
        boxes = []
        for s in self._survivors:
            boxes.append({
                "x": 0.3 + random.uniform(-0.02, 0.02),
                "y": 0.2 + random.uniform(-0.02, 0.02),
                "w": 0.2,
                "h": 0.4,
                "label": f"Survivor #{s.id}",
                "confidence": s.confidence / 100,
            })
        return boxes

    def trigger_abort(self) -> None:
        self._aborted = True
        self._mission_state = MissionState.ABORT
        self._add_event("!!! EMERGENCY ABORT TRIGGERED !!!", "ERROR")

    def handle_sim_command(self, action: str, body: dict[str, Any]) -> None:
        if action == "start":
            self._sim_state = "RUNNING"
            self._add_event("SIMULATION STARTED", "SUCCESS")
        elif action == "pause":
            self._sim_state = "PAUSED"
            self._add_event("SIMULATION PAUSED", "WARN")
        elif action == "reset":
            self._sim_state = "STOPPED"
            self._mission_state = MissionState.IDLE
            self._mission_timer = 0
            self._survivors = []
            self._events = []
            self._add_event("SIMULATION RESET", "WARN")
        if "scenario" in body:
            self._scenario = body["scenario"]

    # ── Internal helpers ──────────────────────────────────────

    def _add_event(self, msg: str, level: str = "INFO") -> None:
        self._ev_counter += 1
        self._events.insert(0, MissionEvent(
            id=f"evt-{self._ev_counter}",
            timestamp=time.time(),
            message=msg,
            level=level,
        ))
        self._events = self._events[:200]

    def _update_health(self) -> None:
        def h(name: str, status: SubsystemStatus) -> SubsystemHealth:
            return SubsystemHealth(name=name, status=status, lastUpdate=time.time())

        self._health = SystemHealth(
            px4      = h("PX4",       SubsystemStatus.CONNECTED  if self._t > 1.5 else SubsystemStatus.OFFLINE),
            ros2     = h("ROS 2",     SubsystemStatus.CONNECTED  if self._t > 1.5 else SubsystemStatus.OFFLINE),
            fastlio2 = h("FAST-LIO2", SubsystemStatus.TRACKING   if self._t > 9   else SubsystemStatus.OFFLINE),
            lidar    = h("LiDAR",     SubsystemStatus.ONLINE     if self._t > 1.5 else SubsystemStatus.OFFLINE),
            imu      = h("IMU",       SubsystemStatus.ONLINE     if self._t > 1.5 else SubsystemStatus.OFFLINE),
            camera   = h("Camera",    SubsystemStatus.ONLINE     if self._t > 1.5 else SubsystemStatus.OFFLINE),
            yolo     = h("YOLO",      SubsystemStatus.READY      if self._t > 1.5 else SubsystemStatus.OFFLINE),
            planner  = h("Planner",   SubsystemStatus.RUNNING    if self._t > 9   else SubsystemStatus.OFFLINE),
        )

    # ── Mission sequence ──────────────────────────────────────

    SEQUENCE = [
        (2,  MissionState.INIT,              "MISSION INITIALIZED",            "INFO"),
        (4,  MissionState.TAKEOFF,           "TAKEOFF COMMAND ISSUED",         "INFO"),
        (8,  MissionState.LOCALIZATION,      "FAST-LIO2 LOCALIZATION ACQUIRING","INFO"),
        (12, MissionState.EXPLORE,           "FAST-LIO2 LOCKED — EXPLORING",   "SUCCESS"),
        (20, MissionState.EXPLORE,           "FRONTIER F03 — NAVIGATING",      "INFO"),
        (30, MissionState.SURVIVOR_DETECTED, "SURVIVOR #01 DETECTED — 91.2%",  "SUCCESS"),
        (33, MissionState.CONTINUE_EXPLORE,  "EXPLORATION RESUMED",            "INFO"),
        (48, MissionState.SURVIVOR_DETECTED, "SURVIVOR #02 DETECTED — 87.6%",  "SUCCESS"),
        (51, MissionState.CONTINUE_EXPLORE,  "EXPLORATION RESUMED",            "INFO"),
        (65, MissionState.RETURN,            "RETURNING TO ORIGIN",            "INFO"),
        (72, MissionState.LAND,              "LANDING INITIATED",              "INFO"),
        (78, MissionState.COMPLETE,          "MISSION COMPLETE — 2 SURVIVORS", "SUCCESS"),
    ]

    def _advance_mission(self) -> None:
        t = self._mission_timer
        for (trigger, state, msg, level) in self.SEQUENCE:
            if t == trigger:
                self._mission_state = state
                self._add_event(msg, level)
                if trigger == 30:
                    self._survivors.append(Survivor(
                        id=1, gridLabel="A3",
                        position=SurvivorPosition(x=2.1, y=3.4, z=0.5),
                        confidence=91.2, status="CONFIRMED", detectedAt=time.time(),
                    ))
                elif trigger == 48:
                    self._survivors.append(Survivor(
                        id=2, gridLabel="C5",
                        position=SurvivorPosition(x=-3.7, y=5.1, z=0.5),
                        confidence=87.6, status="CONFIRMED", detectedAt=time.time(),
                    ))

    # ── Main async loop ───────────────────────────────────────

    async def run(self) -> None:
        """Background task: update state at 20 Hz."""
        last_second = 0.0
        while True:
            now = time.time()
            self._t += 0.05
            t = self._t

            # Telemetry
            r = 3 + math.sin(t * 0.3) * 2
            self._drone = DroneState(
                id="NIDAR-01",
                mode="OFFBOARD" if t > 1.5 else "IDLE",
                connectionStatus=ConnectionStatus.CONNECTED if t > 1.5 else ConnectionStatus.DISCONNECTED,
                position=Vec3(
                    x=round(math.cos(t * 0.4) * r, 3),
                    y=round(math.sin(t * 0.4) * r, 3),
                    z=round(1.5 + math.sin(t * 0.7) * 0.3, 2),
                ),
                velocity=Vec3(
                    x=round(-math.sin(t * 0.4) * r * 0.4, 2),
                    y=round(math.cos(t * 0.4) * r * 0.4, 2),
                    z=round(math.cos(t * 0.7) * 0.3, 2),
                ),
                attitude=Attitude(
                    roll  = round(math.sin(t * 1.2) * 0.08, 3),
                    pitch = round(math.cos(t * 0.9) * 0.06, 3),
                    yaw   = round((t * 0.4) % (2 * math.pi), 3),
                ),
                battery=Battery(
                    voltage    = round(max(10, 16.8 - t * 0.005), 2),
                    current    = round(8 + math.sin(t) * 0.5, 2),
                    percentage = round(max(0, 100 - t * 0.1), 1),
                ),
                timestamp=now,
            )

            self._update_health()

            # 1 Hz mission timer + sequence
            if now - last_second >= 1.0:
                last_second = now
                self._mission_timer += 1
                if not self._aborted:
                    self._advance_mission()
                # Connection flags
                self._gazebo   = t > 1.5
                self._px4_sitl = t > 1.5
                self._ros2     = t > 1.5

            await asyncio.sleep(0.05)
