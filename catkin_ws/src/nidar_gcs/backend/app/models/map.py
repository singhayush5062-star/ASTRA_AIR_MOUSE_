from __future__ import annotations
from pydantic import BaseModel


class MapMetadata(BaseModel):
    width:      int   = 80
    height:     int   = 80
    resolution: float = 0.2      # metres/cell
    originX:    float = -8.0
    originY:    float = -8.0
