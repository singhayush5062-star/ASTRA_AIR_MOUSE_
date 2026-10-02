from __future__ import annotations
from pydantic import BaseModel


class SurvivorPosition(BaseModel):
    x: float
    y: float
    z: float


class Survivor(BaseModel):
    id:         int
    gridLabel:  str
    position:   SurvivorPosition
    confidence: float
    status:     str = "CONFIRMED"
    detectedAt: float
