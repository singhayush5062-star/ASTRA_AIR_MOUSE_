"""
NIDAR AirMouse GCS — FastAPI Backend
=====================================
Provides strictly separated REST endpoints and WebSocket streams
for both Simulation and Real Hardware operating modes.
"""

from __future__ import annotations

import asyncio
import os
import time
from contextlib import asynccontextmanager
from typing import Any, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from pydantic import BaseModel

from app.core.config import settings

from app.models.telemetry import (
    DroneState, MissionState, SystemHealth, SubsystemHealth, SubsystemStatus,
    Vec3, Attitude, Battery, ConnectionStatus,
)
from app.models.mission import MissionEvent, MissionInfo, AutonomyInfo
from app.models.survivor import Survivor
from app.models.map import MapMetadata
from app.services.hardware_service import HardwareService
from app.services.mock_provider import MockDataProvider
from app.services.serial_ports import list_serial_ports
from app.services.sim_service import SimulationService
from app.services.vision_service import vision_service

# ─── State & Providers ──────────────────────────────────────

# The Simulation dashboard is backed by the real NIDAR simulation (Gazebo + PX4 SITL + the ROS
# stack) through SimulationService. GCS_MOCK=1 restores the original synthetic data source for
# UI work on a machine without ROS.
MOCK = settings.GCS_MOCK
sim_provider = MockDataProvider() if MOCK else SimulationService()
# The Hardware dashboard is backed by the real drone: a MAVLink link (T12 / SiK / USB / Wi-Fi
# UDP) plus, when reachable, the Jetson's ROS master. See app/services/hardware_service.py.
hw = HardwareService()


class ConnectRequest(BaseModel):
    connection_type: str = "serial"  # serial | udp | tcp | simulator
    serial_port: Optional[str] = None
    baud_rate: int = 57600
    host: Optional[str] = None
    udp_port: int = 14550
    tcp_port: int = 5760
    drone_name: str = "Drone Alpha"
    sys_id: int = 1
    home_lat: Optional[float] = 28.6754
    home_lon: Optional[float] = 77.5029
    force_connect: bool = False


class CommandRequest(BaseModel):
    command: str  # takeoff | land | rtl | abort


# ─── Lifespan ────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start background simulation telemetry and YOLO vision inference."""
    task = asyncio.create_task(sim_provider.run())
    hw_task = asyncio.create_task(hw.run())
    try:
        vision_service.start_camera()
    except Exception as e:
        print(f"[Vision] Camera start failed: {e}")
    yield
    task.cancel()
    hw_task.cancel()
    vision_service.stop_camera()
    for t in (task, hw_task):
        try:
            await t
        except asyncio.CancelledError:
            pass
    # Close the drone links (MAVLink port, ROS bridge). The drone itself is unaffected: it flies
    # its mission onboard, and a restarted GCS reconnects to it.
    await hw.shutdown()
    # Stop only the ROS bridge. A running simulation is deliberately left up: restarting the
    # GCS must not crash a flight, and the next GCS instance re-attaches to it.
    if not MOCK:
        await sim_provider.link.stop()


# ─── App Setup ───────────────────────────────────────────────

app = FastAPI(
    title="NIDAR AirMouse GCS API",
    description="Ground Control Station for NIDAR 2026 autonomous indoor SAR drone",
    version="2.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ─── System & Common Routes ──────────────────────────────────

@app.get("/api/health")
async def health() -> dict[str, Any]:
    return {"status": "ok", "timestamp": time.time(), "version": "2.1.0"}


@app.get("/api/system/status")
async def system_status() -> dict[str, Any]:
    return {
        "hardware_connected": hw.connected,
        "simulation_running": sim_provider.get_state().get("sim_state") == "RUNNING",
        "timestamp": time.time(),
    }


# ─── Hardware Routes (/api/hardware/*) ───────────────────────

@app.get("/api/hardware/ports")
async def get_hardware_ports() -> dict[str, Any]:
    """Serial ports on THIS computer (the one running the backend): T12 / SiK radio dongles, the
    FC's USB port, Jetson UARTs. Annotated with permission / in-use problems."""
    loop = asyncio.get_event_loop()
    ports = await loop.run_in_executor(None, list_serial_ports)
    return {"ports": ports, "count": len(ports), "timestamp": time.time()}


def _hardware_snapshot() -> dict[str, Any]:
    state = hw.get_state()
    return {
        "connected": hw.connected,
        "status": hw.state,
        "error": hw.error,
        "drone": state["drone"].model_dump(),
        "health": {k: v.model_dump() for k, v in state["health"].items()},
        "mission_state": state["mission_state"],
        "mission_timer": state["mission_timer"],
        "mission_phases": state["mission_phases"],
        "autonomy": state["autonomy"].model_dump(),
        "survivors": [s.model_dump() for s in state["survivors"]],
        "link": state["link"],
        "config": hw.config,
        "timestamp": time.time(),
    }


@app.get("/api/hardware")
@app.get("/api/hardware/status")
async def get_hardware_status() -> dict[str, Any]:
    return _hardware_snapshot()


@app.post("/api/hardware/connect")
async def connect_hardware(req: Optional[ConnectRequest] = None) -> Any:
    """Open the MAVLink link and wait for the flight controller's heartbeat; on success also
    attach to the Jetson's ROS master when reachable. 503 with the reason otherwise."""
    if req is None:
        req = ConnectRequest()
    ok, message = await hw.connect(req.model_dump())
    if ok:
        return {"connected": True, "status": "CONNECTED", "message": message,
                "config": hw.config, "timestamp": time.time()}
    return JSONResponse(status_code=503, content={
        "connected": False, "status": "ERROR", "error": message, "timestamp": time.time()})


@app.post("/api/hardware/disconnect")
async def disconnect_hardware() -> dict[str, Any]:
    await hw.disconnect()
    return {"connected": False, "status": "DISCONNECTED",
            "message": "Drone links closed by operator (the drone keeps flying its mission)",
            "timestamp": time.time()}


@app.post("/api/hardware/command")
async def hardware_command(req: CommandRequest) -> Any:
    """TAKEOFF (start the autonomous mission), LAND (PX4 AUTO.LAND here), RTL (mission return
    to the launch pad through the arena door), ABORT (AUTO.LAND on every link)."""
    res = await hw.command(req.command)
    body = {"command": req.command.lower(), "timestamp": time.time(), **res}
    return body if res.get("ok") else JSONResponse(status_code=409, content=body)


@app.post("/api/hardware/abort")
async def hardware_abort() -> Any:
    res = await hw.command("abort")
    body = {"status": "ABORT_TRIGGERED" if res.get("ok") else "ABORT_FAILED",
            "timestamp": time.time(), **res}
    return body if res.get("ok") else JSONResponse(status_code=409, content=body)


@app.get("/api/hardware/map")
async def get_hardware_map() -> dict[str, Any]:
    m = hw.get_map()
    if m is None:
        return {"meta": None, "data": [], "timestamp": time.time() * 1000.0}
    return m


@app.get("/api/hardware/arena")
async def hardware_arena() -> dict[str, Any]:
    """Same arena geometry as the simulation: the real arena is laid out per mission_config."""
    if MOCK:
        return await simulation_arena()
    return sim_provider.arena_info


# ─── Simulation Routes (/api/simulation/*) ───────────────────

@app.get("/api/simulation")
@app.get("/api/simulation/status")
async def get_simulation_status() -> dict[str, Any]:
    state = sim_provider.get_state()
    return {
        "gazebo_connected": state.get("gazebo_connected", False),
        "px4_sitl_connected": state.get("px4_sitl_connected", False),
        "ros2_connected": state.get("ros2_connected", False),
        "sim_state": state.get("sim_state", "STOPPED"),
        "scenario": state.get("scenario", "scenario_01"),
        "timestamp": time.time(),
    }


@app.post("/api/simulation/control")
async def simulation_control(body: dict[str, Any]) -> dict[str, Any]:
    action = body.get("action", "")
    if MOCK:
        sim_provider.handle_sim_command(action, body)
        result: dict[str, Any] = {"ok": True}
    else:
        result = await sim_provider.handle_sim_command(action, body)
    return {"status": "OK" if result.get("ok") else "REJECTED", "action": action,
            "result": result, "sim_state": sim_provider.get_state().get("sim_state"),
            "timestamp": time.time()}


@app.get("/api/simulation/arena")
async def simulation_arena() -> dict[str, Any]:
    """Arena geometry the 3D view needs (bounds, entry door, launch pad, scenarios)."""
    if MOCK:
        return {"name": "mock", "bounds": {"x_min": -7.5, "x_max": 7.5, "y_min": -7.5, "y_max": 7.5},
                "entry": {"x": 0.0, "y": -7.2}, "launch_pad": {"x": 0.0, "y": -9.5},
                "scenarios": {"scenario_01": "mock"}}
    return sim_provider.arena_info


# ─── Mission Routes (/api/mission/*) ─────────────────────────

@app.get("/api/mission")
@app.get("/api/mission/status")
async def get_mission() -> dict[str, Any]:
    state = sim_provider.get_state()
    return {
        "state": state["mission_state"],
        "timer": state["mission_timer"],
        "phases": state.get("mission_phases"),
        "autonomy": state["autonomy"].model_dump(),
        "timestamp": time.time(),
    }


@app.post("/api/mission/abort")
async def abort_mission() -> dict[str, Any]:
    sim_provider.trigger_abort()
    return {"status": "ABORT_TRIGGERED", "timestamp": time.time()}


# ─── Survivors & Map Routes ──────────────────────────────────

@app.get("/api/survivors")
async def get_survivors() -> dict[str, Any]:
    state = sim_provider.get_state()
    return {
        "count": len(state["survivors"]),
        "survivors": [s.model_dump() for s in state["survivors"]],
        "timestamp": time.time(),
    }


@app.get("/api/map")
async def get_map() -> dict[str, Any]:
    if MOCK:
        return {
            "meta": {"width": 80, "height": 80, "resolution": 0.2, "originX": -8.0, "originY": -8.0},
            "data": sim_provider.get_map_data(),
            "timestamp": time.time(),
        }
    m = sim_provider.get_map()
    if m is None:
        return {"meta": None, "data": [], "timestamp": time.time() * 1000.0}
    return m


# ─── WebSocket Connection Manager ────────────────────────────

class ConnectionManager:
    def __init__(self) -> None:
        self.active: dict[str, list[WebSocket]] = {
            "simulation": [],
            "hardware": [],
            "hardware_map": [],
            "hardware_events": [],
            "telemetry": [],
            "map": [],
            "camera": [],
            "events": [],
        }

    async def connect(self, ws: WebSocket, channel: str) -> None:
        await ws.accept()
        self.active.setdefault(channel, []).append(ws)

    def disconnect(self, ws: WebSocket, channel: str) -> None:
        ch = self.active.get(channel, [])
        if ws in ch:
            ch.remove(ws)

    async def broadcast(self, channel: str, data: dict[str, Any]) -> None:
        dead: list[WebSocket] = []
        for ws in list(self.active.get(channel, [])):
            try:
                await ws.send_json(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws, channel)


manager = ConnectionManager()


# ─── WebSocket Endpoints ─────────────────────────────────────

# Channels that only send on change (events, map) also send a small ping this often: a closed
# client is only noticed when a write fails, and without it a handler for a closed tab would
# loop until the next event (forever, after a reset) and block server shutdown.
WS_HEARTBEAT_S = 2.0

@app.websocket("/api/ws/simulation")
@app.websocket("/api/ws/telemetry")
async def ws_simulation(ws: WebSocket):
    await manager.connect(ws, "simulation")
    try:
        while True:
            state = sim_provider.get_state()
            await ws.send_json({
                "type": "telemetry",
                "mode": "SIMULATION",
                "payload": {
                    "drone": state["drone"].model_dump(),
                    "mission_state": state["mission_state"],
                    "mission_timer": state["mission_timer"],
                    "mission_phases": state.get("mission_phases"),
                    "health": {k: v.model_dump() for k, v in state["health"].items()},
                    "autonomy": state["autonomy"].model_dump(),
                    "survivors": [s.model_dump() for s in state["survivors"]],
                    "sim": {
                        "state": state.get("sim_state", "STOPPED"),
                        "scenario": state.get("scenario"),
                        "gazeboConnected": state.get("gazebo_connected", False),
                        "px4SitlConnected": state.get("px4_sitl_connected", False),
                        "ros2Connected": state.get("ros2_connected", False),
                    },
                },
                "timestamp": time.time(),
            })
            await asyncio.sleep(0.1)  # 10 Hz: the bridge snapshots the stack at 10 Hz
    except WebSocketDisconnect:
        manager.disconnect(ws, "simulation")


@app.websocket("/api/ws/hardware")
async def ws_hardware(ws: WebSocket):
    """Real-drone telemetry at 10 Hz. Same envelope as /api/ws/simulation (type "telemetry",
    payload.drone, ...) so the AirMouse panel and the Hardware dashboard read it the same way."""
    await manager.connect(ws, "hardware")
    try:
        while True:
            snap = _hardware_snapshot()
            await ws.send_json({
                "type": "telemetry" if hw.connected else "status",
                "mode": "HARDWARE",
                "connected": hw.connected,
                "payload": snap,
                "timestamp": time.time(),
            })
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        manager.disconnect(ws, "hardware")


@app.websocket("/api/ws/hardware/map")
async def ws_hardware_map(ws: WebSocket):
    await manager.connect(ws, "hardware_map")
    try:
        last_sent: Any = -1
        last_write = time.time()
        while True:
            m = hw.get_map()
            stamp = None if m is None else m["timestamp"]
            if stamp != last_sent:
                last_sent, last_write = stamp, time.time()
                await ws.send_json({"type": "map", "payload": m, "timestamp": time.time()})
            elif time.time() - last_write > WS_HEARTBEAT_S:
                last_write = time.time()
                await ws.send_json({"type": "ping", "timestamp": time.time()})
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        manager.disconnect(ws, "hardware_map")


@app.websocket("/api/ws/hardware/events")
async def ws_hardware_events(ws: WebSocket):
    await manager.connect(ws, "hardware_events")
    last_seq = 0
    last_write = time.time()
    try:
        while True:
            for evt in hw.events_since(last_seq):
                last_seq = int(evt.id.split("-")[1])
                last_write = time.time()
                await ws.send_json({"type": "event", "payload": evt.model_dump(),
                                    "timestamp": time.time()})
            if time.time() - last_write > WS_HEARTBEAT_S:
                last_write = time.time()
                await ws.send_json({"type": "ping", "timestamp": time.time()})
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        manager.disconnect(ws, "hardware_events")


@app.websocket("/api/ws/map")
async def ws_map(ws: WebSocket):
    await manager.connect(ws, "map")
    try:
        last_sent = None
        last_write = time.time()
        while True:
            if MOCK:
                await ws.send_json({
                    "type": "map",
                    "payload": {
                        "meta": {"width": 80, "height": 80, "resolution": 0.2, "originX": -8.0, "originY": -8.0},
                        "data": sim_provider.get_map_data(),
                    },
                    "timestamp": time.time(),
                })
                await asyncio.sleep(0.2)  # 5 Hz
                continue
            m = sim_provider.get_map()
            stamp = None if m is None else m["timestamp"]
            if stamp != last_sent:
                # /map_2d is published at 2 Hz and forwarded at most at 1 Hz; send only changes.
                # A null payload tells the UI the run's map is gone (simulation reset).
                last_sent = stamp
                last_write = time.time()
                await ws.send_json({"type": "map", "payload": m, "timestamp": time.time()})
            elif time.time() - last_write > WS_HEARTBEAT_S:
                last_write = time.time()
                await ws.send_json({"type": "ping", "timestamp": time.time()})
            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        manager.disconnect(ws, "map")


@app.websocket("/api/ws/events")
async def ws_events(ws: WebSocket):
    await manager.connect(ws, "events")
    last_event_count = 0
    last_seq = 0
    last_write = time.time()
    try:
        while True:
            if not MOCK:
                # Sequence-numbered: a (re)connecting client gets the backlog once, oldest
                # first, then only new events.
                for evt in sim_provider.events_since(last_seq):
                    last_seq = int(evt.id.split("-")[1])
                    last_write = time.time()
                    await ws.send_json({"type": "event", "payload": evt.model_dump(),
                                        "timestamp": time.time()})
                if time.time() - last_write > WS_HEARTBEAT_S:
                    last_write = time.time()
                    await ws.send_json({"type": "ping", "timestamp": time.time()})
                await asyncio.sleep(0.1)
                continue
            state = sim_provider.get_state()
            events = state["events"]
            if len(events) != last_event_count:
                new_events = events[: len(events) - last_event_count]
                last_event_count = len(events)
                for evt in reversed(new_events):
                    await ws.send_json({
                        "type": "event",
                        "payload": evt.model_dump(),
                        "timestamp": time.time(),
                    })
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        manager.disconnect(ws, "events")


@app.websocket("/api/ws/camera")
async def ws_camera(ws: WebSocket):
    await manager.connect(ws, "camera")
    frame = 0
    try:
        while True:
            frame += 1
            # Prefer real detections from vision_service
            bboxes = [
                {
                    "x": d["normalized_box"][0],
                    "y": d["normalized_box"][1],
                    "w": d["normalized_box"][2],
                    "h": d["normalized_box"][3],
                    "label": f"Person #{i+1}",
                    "confidence": d["confidence"],
                }
                for i, d in enumerate(vision_service.latest_detections)
            ] if vision_service.latest_detections else sim_provider.get_bboxes()

            await ws.send_json({
                "type": "camera",
                "payload": {
                    "connected": vision_service.receiving_frames,
                    "camera_source": "FC / Jetson",
                    "camera_status": vision_service.camera_status,
                    "receiving_frames": vision_service.receiving_frames,
                    "fps": round(vision_service.fps, 1),
                    "latency_ms": round(vision_service.latest_latency_ms, 1),
                    "stream_url": vision_service.stream_url,
                    "width": 640,
                    "height": 480,
                    "frame": frame,
                    "bounding_boxes": bboxes,
                    "model": vision_service.model_name,
                    "model_status": "RUNNING ON LAPTOP",
                },
                "timestamp": time.time(),
            })
            await asyncio.sleep(1 / 30)
    except WebSocketDisconnect:
        manager.disconnect(ws, "camera")


# ─── FC / Jetson Camera & Laptop Vision REST Endpoints ───────

def _onboard_camera(mode: Optional[str]):
    """The ROS-side camera the requesting page should show, if it is live.

    mode=hardware: the real drone's onboard camera (Jetson, over the ROS link), unless the
    operator configured an explicit FC/T12 stream URL that is delivering frames.
    mode=simulation (or none): the simulated drone camera, as before."""
    if MOCK:
        return None
    if mode == "hardware":
        if vision_service.receiving_frames:
            return None
        return hw if hw.camera_live() else None
    return sim_provider if sim_provider.camera_live() else None


@app.get("/api/camera/stream")
async def get_camera_stream(mode: Optional[str] = None):
    """Live MJPEG video stream from FC/Jetson with laptop-side YOLO26s annotations.

    While a simulation is running, this is the simulated drone camera instead, annotated with
    the onboard detector's boxes (nidar_perception); on the Hardware page (mode=hardware) it is
    the real drone's onboard camera when the ROS link carries it."""
    src = _onboard_camera(mode)
    if src is not None:
        return StreamingResponse(src.mjpeg(),
                                 media_type="multipart/x-mixed-replace; boundary=frame")
    if not vision_service.is_running or (not vision_service.receiving_frames and vision_service.latest_annotated_jpeg is None):
        return JSONResponse(
            status_code=503,
            content={
                "status": "OFFLINE",
                "camera_source": "FC / Jetson",
                "error": "FC CAMERA OFFLINE",
                "message": f"Waiting for FC/Jetson camera stream at {vision_service.stream_url}",
                "stream_url": vision_service.stream_url,
                "fc_host": vision_service.fc_host,
            },
        )
    return StreamingResponse(
        vision_service.generate_mjpeg_stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/camera/status")
async def get_camera_status(mode: Optional[str] = None) -> dict[str, Any]:
    """Status metrics of the FC/Jetson camera link and Laptop YOLO model (or of the simulated /
    onboard camera and the onboard detector while one is live)."""
    src = _onboard_camera(mode)
    if src is not None:
        return src.camera_status()
    return vision_service.get_status()


@app.post("/api/camera/config")
async def update_camera_config(body: dict[str, Any]) -> dict[str, Any]:
    """Update FC camera stream URL and host dynamically from GCS UI."""
    stream_url = body.get("stream_url")
    fc_host = body.get("fc_host")
    vision_service.update_config(stream_url=stream_url, fc_host=fc_host)
    return vision_service.get_status()


@app.post("/api/camera/control")
async def control_camera(body: dict[str, Any]) -> dict[str, Any]:
    """Control camera receiver connection."""
    action = body.get("action", "")
    if action in ("start", "reconnect"):
        url = body.get("stream_url")
        vision_service.stop_camera()
        vision_service.start_camera(url)
    elif action == "stop":
        vision_service.stop_camera()
    return vision_service.get_status()



# ─── Web UI (built frontend) ─────────────────────────────────
# scripts/setup_gcs.sh builds the React app into frontend/dist; serving it from here means one
# process and one port (http://localhost:8000) for the whole GCS. Registered last so every /api
# route above wins; unknown paths fall back to index.html for the client-side router.

if os.path.isdir(settings.FRONTEND_DIST):
    _DIST = os.path.realpath(settings.FRONTEND_DIST)

    @app.get("/{full_path:path}", include_in_schema=False)
    async def web_ui(full_path: str):
        if full_path.startswith("api/"):
            return JSONResponse(status_code=404, content={"detail": "Not Found"})
        candidate = os.path.realpath(os.path.join(_DIST, full_path))
        if full_path and candidate.startswith(_DIST + os.sep) and os.path.isfile(candidate):
            return FileResponse(candidate)
        return FileResponse(os.path.join(_DIST, "index.html"))
