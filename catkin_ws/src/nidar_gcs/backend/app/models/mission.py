from __future__ import annotations
from pydantic import BaseModel


class MissionEvent(BaseModel):
    id:        str
    timestamp: float
    message:   str
    level:     str = "INFO"   # INFO | SUCCESS | WARN | ERROR


class MissionInfo(BaseModel):
    state:   str = "IDLE"
    timer:   int = 0
    elapsed: str = "00:00"


class AutonomyInfo(BaseModel):
    state:              str = "IDLE"
    planner:            str = "FUEL"
    localization:       str = "FAST-LIO2"
    currentTarget:      str = "—"
    distanceToTarget:   float = 0.0
    reason:             str = "Awaiting mission start"
