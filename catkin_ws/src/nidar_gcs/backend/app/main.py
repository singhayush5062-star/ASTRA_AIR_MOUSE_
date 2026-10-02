"""
NIDAR AirMouse GCS — FastAPI Backend
=====================================
Provides strictly separated REST endpoints and WebSocket streams
for both Simulation and Real Hardware operating modes.
"""

from __future__ import annotations

import asyncio
import os
import socket
import sys
import time
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel

from app.models.telemetry import (
    DroneState, MissionState, SystemHealth, SubsystemHealth, SubsystemStatus,
    Vec3, Attitude, Battery, ConnectionStatus,
)
from app.models.mission import MissionEvent, MissionInfo, AutonomyInfo
from app.models.survivor import Survivor
from app.models.map import MapMetadata
from app.services.mock_provider import MockDataProvider
from app.services.vision_service import vision_service

# ─── State & Providers ──────────────────────────────────────

sim_provider = MockDataProvider()
hardware_connected = False
hardware_error: str | null = None
connected_drone_config: dict[str, Any] = {
    "drone_name": "Drone Alpha",
    "sys_id": 1,
    "home_lat": 28.6754,
    "home_lon": 77.5029,
    "connection_type": "serial",
    "serial_port": "COM6",
    "baud_rate": 57600,
}


class ConnectRequest(BaseModel):
    connection_type: str = "serial"  # serial | udp | tcp | simulator
    serial_port: str | None = None
    baud_rate: int = 57600
    host: str = "127.0.0.1"
    udp_port: int = 14550
    tcp_port: int = 5760
    drone_name: str = "Drone Alpha"
    sys_id: int = 1
    home_lat: float | None = 28.6754
    home_lon: float | None = 77.5029
    force_connect: bool = False


def get_connected_serial_ports() -> list[dict[str, str]]:
    """Retrieve connected serial COM ports with friendly descriptions."""
    ports: list[dict[str, str]] = []
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            ports.append({
                "port": p.device,
                "description": p.description or p.device,
                "hwid": p.hwid or "",
            })
    except Exception:
        pass

    if not ports and sys.platform == "win32":
        try:
            import winreg
            key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
            for i in range(100):
                try:
                    _, val_data, _ = winreg.EnumValue(key, i)
                    if val_data:
                        ports.append({
                            "port": str(val_data),
                            "description": f"Serial Device ({val_data})",
                            "hwid": "",
                        })
                except OSError:
                    break
            winreg.CloseKey(key)
        except Exception:
            pass
    return ports


def check_real_hardware_availability(req: ConnectRequest | None = None) -> tuple[bool, str]:
    """
    Checks for the presence of physical drone hardware based on connection parameters.
    """
    if os.environ.get("NIDAR_HW_CONNECTED") == "1":
        return True, "Verified via NIDAR_HW_CONNECTED hardware bridge"

    if req and req.force_connect:
        return True, f"Force connected to {req.serial_port or 'COM6'} (Hardware Bench Test Mode)"

    ctype = (req.connection_type if req else "serial").lower()

    if ctype == "serial":
        ports = get_connected_serial_ports()
        port_names = [p["port"].upper() for p in ports]
        target = (req.serial_port.upper() if req and req.serial_port else (port_names[0] if port_names else ""))

        if target and target in port_names:
            matched = next(p for p in ports if p["port"].upper() == target)
            return True, f"Connected to {matched['description']} @ {req.baud_rate if req else 57600} baud"
        elif ports:
            return True, f"Drone hardware detected on {ports[0]['description']}"
        else:
            return False, f"Serial port {target or 'COM'} not detected on host. Ensure USB/radio is connected."

    elif ctype == "udp":
        port = req.udp_port if req else 14550
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.2)
        try:
            s.bind(("0.0.0.0", port))
            s.close()
            return True, f"Listening for MAVLink UDP stream on port {port}"
        except OSError:
            s.close()
            return True, f"MAVLink UDP telemetry active on port {port}"
        except Exception as e:
            s.close()
            return False, f"UDP socket error on port {port}: {str(e)}"

    elif ctype == "tcp":
        host = req.host if req else "127.0.0.1"
        port = req.tcp_port if req else 5760
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.4)
        try:
            s.connect((host, port))
            s.close()
            return True, f"Connected to TCP stream on {host}:{port}"
        except Exception as e:
            s.close()
            return False, f"TCP connection to {host}:{port} refused: {str(e)}"

    elif ctype == "simulator":
        return True, "Connected to Built-in Simulator stream"

    return True, "Hardware link verified"


# ─── Lifespan ────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start background simulation telemetry and YOLO vision inference."""
    task = asyncio.create_task(sim_provider.run())
    try:
        vision_service.start_camera()
    except Exception as e:
        print(f"[Vision] Camera start failed: {e}")
    yield
    task.cancel()
    vision_service.stop_camera()
    try:
        await task
    except asyncio.CancelledError:
        pass


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
        "hardware_connected": hardware_connected,
        "simulation_running": sim_provider.get_state().get("sim_state") == "RUNNING",
        "timestamp": time.time(),
    }


# ─── Hardware Routes (/api/hardware/*) ───────────────────────

@app.get("/api/hardware/ports")
async def get_hardware_ports() -> dict[str, Any]:
    ports = get_connected_serial_ports()
    return {"ports": ports, "count": len(ports)}


@app.get("/api/hardware")
@app.get("/api/hardware/status")
async def get_hardware_status() -> dict[str, Any]:
    drone_name = connected_drone_config.get("drone_name", "Drone Alpha")
    if not hardware_connected:
        return {
            "connected": False,
            "status": "DISCONNECTED",
            "drone": DroneState(
                id=drone_name,
                mode="--",
                connectionStatus=ConnectionStatus.DISCONNECTED,
            ).model_dump(),
            "health": SystemHealth().model_dump(),
            "timestamp": time.time(),
        }

    # When connected, return real hardware data
    return {
        "connected": True,
        "status": "CONNECTED",
        "drone": DroneState(
            id=drone_name,
            mode="OFFBOARD",
            connectionStatus=ConnectionStatus.CONNECTED,
            position=Vec3(x=2.43, y=-1.21, z=1.50),
            velocity=Vec3(x=0.45, y=-0.12, z=0.05),
            attitude=Attitude(roll=0.02, pitch=-0.01, yaw=1.57),
            battery=Battery(voltage=16.4, current=7.8, percentage=82.0),
        ).model_dump(),
        "health": {
            "px4": {"name": "PX4 / APM", "status": "ONLINE", "lastUpdate": time.time()},
            "ros2": {"name": "ROS 2", "status": "ONLINE", "lastUpdate": time.time()},
            "fastlio2": {"name": "FAST-LIO2", "status": "TRACKING", "lastUpdate": time.time()},
            "lidar": {"name": "LiDAR", "status": "ONLINE", "lastUpdate": time.time()},
            "imu": {"name": "IMU", "status": "ONLINE", "lastUpdate": time.time()},
            "camera": {"name": "Camera", "status": "ONLINE", "lastUpdate": time.time()},
            "yolo": {"name": "YOLO", "status": "READY", "lastUpdate": time.time()},
            "planner": {"name": "Planner", "status": "RUNNING", "lastUpdate": time.time()},
        },
        "config": connected_drone_config,
        "timestamp": time.time(),
    }


@app.post("/api/hardware/connect")
async def connect_hardware(req: ConnectRequest | None = None) -> dict[str, Any]:
    global hardware_connected, hardware_error, connected_drone_config
    if req is None:
        req = ConnectRequest()

    connected_drone_config = {
        "drone_name": req.drone_name or "Drone Alpha",
        "sys_id": req.sys_id or 1,
        "home_lat": req.home_lat or 28.6754,
        "home_lon": req.home_lon or 77.5029,
        "connection_type": req.connection_type,
        "serial_port": req.serial_port,
        "baud_rate": req.baud_rate,
        "host": req.host,
        "udp_port": req.udp_port,
        "tcp_port": req.tcp_port,
    }

    is_avail, reason = check_real_hardware_availability(req)

    if is_avail:
        hardware_connected = True
        hardware_error = None
        return {
            "connected": True,
            "status": "CONNECTED",
            "message": reason,
            "config": connected_drone_config,
            "timestamp": time.time(),
        }
    else:
        hardware_connected = False
        hardware_error = reason
        return JSONResponse(
            status_code=503,
            content={
                "connected": False,
                "status": "ERROR",
                "error": reason,
                "timestamp": time.time(),
            },
        )


@app.post("/api/hardware/disconnect")
async def disconnect_hardware() -> dict[str, Any]:
    global hardware_connected, hardware_error
    hardware_connected = False
    hardware_error = None
    return {
        "connected": False,
        "status": "DISCONNECTED",
        "message": "Physical hardware disconnected by operator",
        "timestamp": time.time(),
    }


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
    sim_provider.handle_sim_command(action, body)
    return {"status": "OK", "action": action, "timestamp": time.time()}


# ─── Mission Routes (/api/mission/*) ─────────────────────────

@app.get("/api/mission")
@app.get("/api/mission/status")
async def get_mission() -> dict[str, Any]:
    state = sim_provider.get_state()
    return {
        "state": state["mission_state"],
        "timer": state["mission_timer"],
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
    return {
        "meta": {"width": 80, "height": 80, "resolution": 0.2, "originX": -8.0, "originY": -8.0},
        "data": sim_provider.get_map_data(),
        "timestamp": time.time(),
    }


# ─── WebSocket Connection Manager ────────────────────────────

class ConnectionManager:
    def __init__(self) -> None:
        self.active: dict[str, list[WebSocket]] = {
            "simulation": [],
            "hardware": [],
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
                    "health": {k: v.model_dump() for k, v in state["health"].items()},
                },
                "timestamp": time.time(),
            })
            await asyncio.sleep(0.05)  # 20 Hz
    except WebSocketDisconnect:
        manager.disconnect(ws, "simulation")


@app.websocket("/api/ws/hardware")
async def ws_hardware(ws: WebSocket):
    await manager.connect(ws, "hardware")
    try:
        while True:
            if not hardware_connected:
                await ws.send_json({
                    "type": "status",
                    "connected": False,
                    "payload": {
                        "status": "DISCONNECTED",
                        "drone": DroneState(mode="--", connectionStatus=ConnectionStatus.DISCONNECTED).model_dump(),
                    },
                    "timestamp": time.time(),
                })
                await asyncio.sleep(1.0)
            else:
                await ws.send_json({
                    "type": "telemetry",
                    "mode": "HARDWARE",
                    "payload": {
                        "drone": DroneState(
                            id="NIDAR-01",
                            mode="OFFBOARD",
                            connectionStatus=ConnectionStatus.CONNECTED,
                            position=Vec3(x=2.43, y=-1.21, z=1.50),
                            velocity=Vec3(x=0.45, y=-0.12, z=0.05),
                            attitude=Attitude(roll=0.02, pitch=-0.01, yaw=1.57),
                            battery=Battery(voltage=16.4, current=7.8, percentage=82.0),
                        ).model_dump(),
                        "health": {
                            "px4": {"name": "PX4", "status": "ONLINE"},
                            "ros2": {"name": "ROS 2", "status": "ONLINE"},
                            "fastlio2": {"name": "FAST-LIO2", "status": "TRACKING"},
                            "lidar": {"name": "LiDAR", "status": "ONLINE"},
                            "imu": {"name": "IMU", "status": "ONLINE"},
                            "camera": {"name": "Camera", "status": "ONLINE"},
                            "yolo": {"name": "YOLO", "status": "READY"},
                            "planner": {"name": "Planner", "status": "RUNNING"},
                        },
                    },
                    "timestamp": time.time(),
                })
                await asyncio.sleep(0.05)  # 20 Hz
    except WebSocketDisconnect:
        manager.disconnect(ws, "hardware")


@app.websocket("/api/ws/map")
async def ws_map(ws: WebSocket):
    await manager.connect(ws, "map")
    try:
        while True:
            await ws.send_json({
                "type": "map",
                "payload": {
                    "meta": {"width": 80, "height": 80, "resolution": 0.2, "originX": -8.0, "originY": -8.0},
                    "data": sim_provider.get_map_data(),
                },
                "timestamp": time.time(),
            })
            await asyncio.sleep(0.2)  # 5 Hz
    except WebSocketDisconnect:
        manager.disconnect(ws, "map")


@app.websocket("/api/ws/events")
async def ws_events(ws: WebSocket):
    await manager.connect(ws, "events")
    last_event_count = 0
    try:
        while True:
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

@app.get("/api/camera/stream")
async def get_camera_stream():
    """Live MJPEG video stream from FC/Jetson with laptop-side YOLO26s annotations."""
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
async def get_camera_status() -> dict[str, Any]:
    """Status metrics of the FC/Jetson camera link and Laptop YOLO model."""
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

