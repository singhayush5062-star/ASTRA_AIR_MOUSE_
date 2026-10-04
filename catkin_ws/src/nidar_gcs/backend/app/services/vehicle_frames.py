"""Pure helpers shared by the hardware link: PX4 flight modes and the vehicle -> arena frames.

No I/O here, so everything is unit tested (tests/test_vehicle_frames.py).

FRAMES. The UI draws everything in the arena `world` frame: the frame of /map_2d, /survivors and
the A1..G7 grid. Onboard, FAST-LIO's `camera_init` is planted on the launch pad at start-up and
the static TF world -> map -> camera_init (nidar_planner/launch/nidar_fuel_upstream.launch,
generated from mission_config.yaml) places it in the arena:

    world = Rz(spawn_yaw) * camera_init + (pad_x, pad_y, spawn_z)

PX4 receives FAST-LIO's pose as external vision through MAVROS, which converts ENU <-> NED, so
PX4's local frame is camera_init with its axes swapped:

    x_ned = y_cinit,  y_ned = x_cinit,  z_ned = -z_cinit

Over ROS the GCS bridge reads the world pose straight from tf. Over a bare MAVLink link (T12,
SiK radio, USB) only LOCAL_POSITION_NED / ATTITUDE arrive, and this module applies the same
chain by hand so both paths put the drone in the same place on the map.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

# ─── PX4 flight modes ────────────────────────────────────────────────────────────────────────
# custom_mode packs (reserved u16, main_mode u8, sub_mode u8), little-endian
# (PX4 src/modules/commander/px4_custom_mode.h). Names follow MAVROS (/mavros/state.mode), which
# is what the Simulation dashboard shows, so both dashboards read the same.
PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO", 6: "OFFBOARD",
            7: "STABILIZED", 8: "RATTITUDE"}
PX4_AUTO_SUB = {1: "AUTO.READY", 2: "AUTO.TAKEOFF", 3: "AUTO.LOITER", 4: "AUTO.MISSION",
                5: "AUTO.RTL", 6: "AUTO.LAND", 7: "AUTO.RTGS", 8: "AUTO.FOLLOW_TARGET",
                9: "AUTO.PRECLAND", 10: "AUTO.VTOL_TAKEOFF"}
PX4_POSCTL_SUB = {0: "POSCTL", 1: "ORBIT", 2: "POSCTL.SLOW"}

# Name -> (main_mode, sub_mode) for MAV_CMD_DO_SET_MODE.
PX4_MODE_IDS = {name: (4, sub) for sub, name in PX4_AUTO_SUB.items()}
PX4_MODE_IDS.update({"MANUAL": (1, 0), "ALTCTL": (2, 0), "POSCTL": (3, 0), "ACRO": (5, 0),
                     "OFFBOARD": (6, 0), "STABILIZED": (7, 0)})

MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_AUTOPILOT_PX4 = 12
MAV_AUTOPILOT_INVALID = 8


def px4_mode_name(custom_mode: int) -> str:
    main = (int(custom_mode) >> 16) & 0xFF
    sub = (int(custom_mode) >> 24) & 0xFF
    if main == 4:
        return PX4_AUTO_SUB.get(sub, "AUTO.%d" % sub)
    if main == 3:
        return PX4_POSCTL_SUB.get(sub, "POSCTL")
    return PX4_MAIN.get(main, "MODE_%d" % main)


# ArduCopter custom modes, used only if the vehicle turns out not to run PX4 (the stack itself is
# PX4-only; this keeps the display meaningful during bench tests with other boards).
COPTER_MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
                6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
                15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
                20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
                25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"}


def mode_name(autopilot: int, base_mode: int, custom_mode: int) -> str:
    if not (int(base_mode) & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED):
        return "--"
    if int(autopilot) == MAV_AUTOPILOT_PX4:
        return px4_mode_name(custom_mode)
    return COPTER_MODES.get(int(custom_mode), "MODE_%d" % int(custom_mode))


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ─── Arena frame ─────────────────────────────────────────────────────────────────────────────

class WorldFrame(object):
    """camera_init (== PX4 local frame, axes swapped) placed in the arena world frame."""

    def __init__(self, pad_x: float = 0.0, pad_y: float = -9.5, spawn_z: float = 0.26,
                 spawn_yaw: float = math.pi / 2.0) -> None:
        self.pad_x = float(pad_x)
        self.pad_y = float(pad_y)
        self.spawn_z = float(spawn_z)
        self.spawn_yaw = float(spawn_yaw)
        self._c = math.cos(self.spawn_yaw)
        self._s = math.sin(self.spawn_yaw)

    @classmethod
    def from_mission_config(cls, cfg: Dict) -> "WorldFrame":
        """Same numbers apply_mission_config.py writes into the world -> map static TF."""
        pad = cfg.get("launch_pad", {}) or {}
        center = pad.get("center", {}) or {}
        veh = cfg.get("vehicle", {}) or {}
        spawn_z = float(pad.get("thickness", 0.03)) + float(veh.get("belly_clearance", 0.23))
        return cls(pad_x=float(center.get("x", 0.0)), pad_y=float(center.get("y", -9.5)),
                   spawn_z=round(spawn_z, 3), spawn_yaw=float(pad.get("spawn_yaw", math.pi / 2.0)))

    def _rot(self, x: float, y: float) -> Tuple[float, float]:
        return self._c * x - self._s * y, self._s * x + self._c * y

    def position_from_ned(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        xc, yc, zc = y, x, -z                      # NED -> camera_init (ENU)
        xw, yw = self._rot(xc, yc)
        return xw + self.pad_x, yw + self.pad_y, zc + self.spawn_z

    def velocity_from_ned(self, vx: float, vy: float, vz: float) -> Tuple[float, float, float]:
        xw, yw = self._rot(vy, vx)
        return xw, yw, -vz

    def attitude_from_ned(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float]:
        """FRD/NED Euler (MAVLink ATTITUDE) -> FLU/world Euler, as the ROS side reports it."""
        return roll, -pitch, wrap_pi(math.pi / 2.0 - yaw + self.spawn_yaw)

    def world_to_local_xy(self, xw: float, yw: float) -> Tuple[float, float]:
        dx, dy = xw - self.pad_x, yw - self.pad_y
        return self._c * dx + self._s * dy, -self._s * dx + self._c * dy


def optional_round(v: Optional[float], nd: int) -> Optional[float]:
    if v is None or (isinstance(v, float) and (math.isnan(v) or math.isinf(v))):
        return None
    return round(float(v), nd)
