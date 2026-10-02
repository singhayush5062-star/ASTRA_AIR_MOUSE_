"""Pure mapping from the NIDAR stack's state to the UI's mission vocabulary.

The UI (frontend/src/types: MissionState, MissionPhase) has its own phase names; the stack's
mission manager (nidar_mission/scripts/mission_manager.py, published on /edm/mission_state) has
others. Everything that translates between them lives here, free of I/O, so it can be unit
tested (test/test_mission_view.py).

  stack (EDM) state              UI mission state
  ─────────────────────────────  ────────────────────────────────────────────────────────────
  (no EDM yet / not armed)       INIT
  TAKEOFF (armed)                TAKEOFF
  ENTRY_SEARCH/CONFIRMATION      LOCALIZATION   (UI's slot before EXPLORE: entering the arena)
  EXPLORATION                    EXPLORE, SURVIVOR_DETECTED (for a few seconds after a new
                                 confirmation), then CONTINUE_EXPLORE once any survivor exists
  RETURN                         RETURN
  DESCEND / LAND                 LAND
  landed + disarmed after LAND   COMPLETE
"""

from __future__ import annotations

import string
from typing import Dict, List, Optional

PHASE_ORDER = ["INIT", "TAKEOFF", "LOCALIZATION", "EXPLORE", "SURVIVOR_DETECTED",
               "CONTINUE_EXPLORE", "RETURN", "LAND"]

EDM_TO_UI = {
    "TAKEOFF": "TAKEOFF",
    "ENTRY_SEARCH": "LOCALIZATION",
    "ENTRY_CONFIRMATION": "LOCALIZATION",
    "EXPLORATION": "EXPLORE",
    "RETURN": "RETURN",
    "DESCEND": "LAND",
    "LAND": "LAND",
}

SURVIVOR_FLASH_S = 4.0


def grid_label(grid_x: int, grid_y: int) -> str:
    """Same convention as nidar_map2d/grid_visualizer.py: row letter A.. from grid_y (south to
    north), column number 1.. from grid_x (west to east). World (0, 0) is D4."""
    if grid_x < 0 or grid_y < 0:
        return "OFF-GRID"
    row = string.ascii_uppercase[grid_y] if grid_y < 26 else "Z"
    return "%s%d" % (row, grid_x + 1)


def derive_mission_state(sim_state: str, edm_state: Optional[str], armed: bool,
                         complete: bool, aborted: bool, n_survivors: int,
                         seconds_since_new_survivor: Optional[float]) -> str:
    if aborted:
        return "ABORT"
    if sim_state in ("STOPPED", "RESETTING"):
        return "IDLE"
    if sim_state == "ERROR":
        return "FAILSAFE"
    if complete:
        return "COMPLETE"
    if edm_state is None or (edm_state == "TAKEOFF" and not armed):
        return "INIT"
    ui = EDM_TO_UI.get(edm_state, "EXPLORE")
    if ui == "EXPLORE":
        if seconds_since_new_survivor is not None and seconds_since_new_survivor < SURVIVOR_FLASH_S:
            return "SURVIVOR_DETECTED"
        if n_survivors > 0:
            return "CONTINUE_EXPLORE"
    return ui


def mission_phases(current: str, n_survivors: int, furthest: Optional[str] = None) -> List[Dict[str, str]]:
    """Phase list for the UI's MissionStateTracker.

    current   the derived UI mission state
    furthest  the furthest phase reached in this run (used for ABORT/FAILSAFE, where the phase
              the run died in is marked FAILED)
    """
    def build(idx: int, failed: bool = False) -> List[Dict[str, str]]:
        out = []
        for i, name in enumerate(PHASE_ORDER):
            if i < idx:
                skipped = name in ("SURVIVOR_DETECTED", "CONTINUE_EXPLORE") and n_survivors == 0
                status = "PENDING" if skipped else "COMPLETE"
            elif i == idx:
                status = "FAILED" if failed else "ACTIVE"
            else:
                status = "PENDING"
            out.append({"state": name, "status": status})
        return out

    if current == "IDLE":
        return [{"state": n, "status": "PENDING"} for n in PHASE_ORDER]
    if current == "COMPLETE":
        return build(len(PHASE_ORDER))
    if current in ("ABORT", "FAILSAFE"):
        if furthest in PHASE_ORDER:
            return build(PHASE_ORDER.index(furthest), failed=True)
        return build(0, failed=True)
    if current in PHASE_ORDER:
        return build(PHASE_ORDER.index(current))
    return build(0)
