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
