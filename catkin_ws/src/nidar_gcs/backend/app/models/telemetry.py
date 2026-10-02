from __future__ import annotations
from enum import Enum
from pydantic import BaseModel


class ConnectionStatus(str, Enum):
    CONNECTED    = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    CONNECTING   = "CONNECTING"
    DEGRADED     = "DEGRADED"
    ERROR        = "ERROR"


class SubsystemStatus(str, Enum):
    ONLINE     = "ONLINE"
    OFFLINE    = "OFFLINE"
    DEGRADED   = "DEGRADED"
    CONNECTED  = "CONNECTED"
    TRACKING   = "TRACKING"
    RUNNING    = "RUNNING"
    READY      = "READY"
    ERROR      = "ERROR"


class Vec3(BaseModel):
    x: float | None = None
    y: float | None = None
    z: float | None = None


class Attitude(BaseModel):
    roll:  float | None = None
    pitch: float | None = None
    yaw:   float | None = None


class Battery(BaseModel):
    voltage:    float | None = None
    current:    float | None = None
    percentage: float | None = None


class DroneState(BaseModel):
    id:               str = "NIDAR-01"
    mode:             str = "--"
    position:         Vec3 = Vec3()
    velocity:         Vec3 = Vec3()
    attitude:         Attitude = Attitude()
    battery:          Battery = Battery()
    connectionStatus: ConnectionStatus = ConnectionStatus.DISCONNECTED
    timestamp:        float = 0.0


class SubsystemHealth(BaseModel):
    name:       str
    status:     SubsystemStatus = SubsystemStatus.OFFLINE
    lastUpdate: float = 0.0


class SystemHealth(BaseModel):
    px4:      SubsystemHealth = SubsystemHealth(name="PX4")
    ros2:     SubsystemHealth = SubsystemHealth(name="ROS 2")
    fastlio2: SubsystemHealth = SubsystemHealth(name="FAST-LIO2")
    lidar:    SubsystemHealth = SubsystemHealth(name="LiDAR")
    imu:      SubsystemHealth = SubsystemHealth(name="IMU")
    camera:   SubsystemHealth = SubsystemHealth(name="Camera")
    yolo:     SubsystemHealth = SubsystemHealth(name="YOLO")
    planner:  SubsystemHealth = SubsystemHealth(name="Planner")


class MissionState(str, Enum):
    IDLE              = "IDLE"
    INIT              = "INIT"
    TAKEOFF           = "TAKEOFF"
    LOCALIZATION      = "LOCALIZATION"
    EXPLORE           = "EXPLORE"
    SURVIVOR_DETECTED = "SURVIVOR_DETECTED"
    CONTINUE_EXPLORE  = "CONTINUE_EXPLORE"
    RETURN            = "RETURN"
    LAND              = "LAND"
    COMPLETE          = "COMPLETE"
    ABORT             = "ABORT"
