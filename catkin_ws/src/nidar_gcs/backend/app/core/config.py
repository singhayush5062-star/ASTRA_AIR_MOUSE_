try:
    from pydantic_settings import BaseSettings
except ImportError:
    from pydantic import BaseModel as BaseSettings


class Settings(BaseSettings):
    # Backend
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    RELOAD: bool = True
    LOG_LEVEL: str = "info"

    # Mode: DEVELOPMENT | SIMULATION | HARDWARE
    MODE: str = "DEVELOPMENT"

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
        env_file = ".env"
        case_sensitive = True


settings = Settings()
