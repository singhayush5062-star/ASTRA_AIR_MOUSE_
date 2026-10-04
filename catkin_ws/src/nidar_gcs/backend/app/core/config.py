import os

try:
    from pydantic_settings import BaseSettings
except ImportError:
    from pydantic import BaseModel as BaseSettings

# backend/app/core/config.py -> backend/ -> nidar_gcs/ -> catkin_ws/src/ -> repo root
_BACKEND_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_GCS_DIR = os.path.dirname(_BACKEND_DIR)
_REPO_ROOT = os.path.abspath(os.path.join(_GCS_DIR, "..", "..", ".."))


class Settings(BaseSettings):
    # Backend
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    RELOAD: bool = False
    LOG_LEVEL: str = "info"

    # Mode: DEVELOPMENT | SIMULATION | HARDWARE
    MODE: str = "DEVELOPMENT"

    # ── NIDAR simulation integration ───────────────────────────
    # Repo checkout and this package. Derived from this file's location; override only if the
    # backend is run from a copy outside the repo.
    NIDAR_ROOT: str = _REPO_ROOT
    NIDAR_GCS_DIR: str = _GCS_DIR
    # Orchestrator the START button runs (the same one used from a terminal) and the teardown
    # RESET runs (the same kill list test_takeoff.sh uses for its clean slate).
    SIM_START_SCRIPT: str = os.path.join(_REPO_ROOT, "scripts", "test_takeoff.sh")
    SIM_STOP_SCRIPT: str = os.path.join(_REPO_ROOT, "catkin_ws", "src", "nidar_bringup",
                                        "scripts", "stop_sim.sh")
    MISSION_CONFIG: str = os.path.join(_REPO_ROOT, "catkin_ws", "src", "nidar_config", "config",
                                       "mission_config.yaml")
    ARENA_GRID_CONFIG: str = os.path.join(_REPO_ROOT, "catkin_ws", "src", "nidar_config", "config",
                                          "arena_grid.yaml")
    # Open the Gazebo client window as well as the web UI. Off by default: the web UI is the
    # operator display, and gzclient costs CPU that FAST-LIO and FUEL need.
    SIM_GAZEBO_GUI: bool = False
    # Seconds the stack may take from START to a connected PX4 before the UI reports ERROR.
    SIM_START_TIMEOUT_S: float = 300.0
    # Serve the original browser-side mock data instead of the simulation (UI development
    # without ROS). The real simulation path is the default.
    GCS_MOCK: bool = False
    # Built web UI, served at / when present (scripts/setup_gcs.sh builds it).
    FRONTEND_DIST: str = os.path.join(_GCS_DIR, "frontend", "dist")

    # ── Real drone (Hardware dashboard) ────────────────────────
    # MAVLink identity of this GCS (QGroundControl uses 255/190 too; PX4 needs a GCS sysid that
    # differs from the vehicle's).
    GCS_SYSID: int = 255
    GCS_COMPID: int = 190
    # Seconds CONNECT waits for the flight controller's first heartbeat. Radio links (T12, SiK)
    # can take a few seconds after power-up.
    MAVLINK_CONNECT_TIMEOUT_S: float = 6.0
    # MAVLink component id of MAVROS on the Jetson (its default, 240). TAKEOFF/RTL sent over a
    # radio link are addressed to it; PX4 forwards them when MAV_0/1_FORWARD are enabled.
    COMPANION_COMPID: int = 240
    # The Jetson's ROS master, for the live map, survivors, mission state, onboard camera and
    # health. "auto" = the IP the MAVLink UDP/TCP packets come from (Wi-Fi via the Jetson),
    # "" = never (MAVLink only), or an explicit URI such as http://192.168.144.50:11311.
    DRONE_ROS_MASTER_URI: str = "auto"
    # This laptop's IP as the Jetson must reach it (ROS_IP). Empty = the address of the interface
    # that routes to the Jetson, detected automatically.
    DRONE_ROS_IP: str = ""
    # Onboard JPEG stream for the GCS (published by nidar_hardware/camera_publisher.py).
    DRONE_CAMERA_TOPIC: str = "/nidar/gcs/camera/compressed"
    # UDP port the "Built-in Simulator" connection listens on: PX4 SITL's GCS link.
    SITL_MAVLINK_UDP_PORT: int = 14550

    # ROS 2 topics (used in SIMULATION/HARDWARE modes)
    ROS_ODOM_TOPIC: str = "/Odometry"
    ROS_MAP_TOPIC: str = "/projected_map"
    ROS_CAMERA_TOPIC: str = "/camera/image_raw"
    ROS_MISSION_TOPIC: str = "/nidar/mission_state"
    ROS_SURVIVOR_TOPIC: str = "/nidar/survivors"

    # CORS / frontend
    FRONTEND_URL: str = "http://localhost:5173"

    # FC / Jetson Camera Network Stream Configuration
    FC_HOST: str = "192.168.1.100"
    FC_CAMERA_PORT: int = 8554
    FC_CAMERA_URL: str = "rtsp://192.168.1.100:8554/live"
    FC_CAMERA_TYPE: str = "rtsp"  # rtsp | http | udp | tcp
    FC_RECONNECT_INTERVAL: float = 3.0

    class Config:
        env_file = os.path.join(_BACKEND_DIR, ".env")
        case_sensitive = True
        extra = "ignore"


settings = Settings()
