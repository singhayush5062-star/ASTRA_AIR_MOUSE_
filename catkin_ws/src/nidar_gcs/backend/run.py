#!/usr/bin/env python3
"""Entry point for the NIDAR GCS backend. Normally started by scripts/start_gcs.sh.

The backend's own Python packages (FastAPI, uvicorn, pydantic 2, ...) live in an isolated
directory installed by scripts/setup_gcs.sh. They are added to sys.path here, NOT to PYTHONPATH,
so the processes the backend starts (the simulation orchestrator, the ROS bridge) keep the plain
ROS environment and can never pick up a conflicting package version.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PYDEPS = os.path.join(os.environ.get("NIDAR_GCS_HOME",
                                     os.path.expanduser("~/.local/share/nidar_gcs")), "pydeps")
if os.path.isdir(PYDEPS):
    sys.path.insert(0, PYDEPS)
os.chdir(HERE)          # relative paths (weights/, .env) are relative to backend/
sys.path.insert(0, HERE)

import uvicorn  # noqa: E402
from app.core.config import settings  # noqa: E402

if __name__ == "__main__":
    uvicorn.run(
        "app.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.RELOAD,
        log_level=settings.LOG_LEVEL,
        # A browser showing the camera keeps an MJPEG response open indefinitely; without a cap,
        # a graceful shutdown waits for it forever and the old process never exits.
        timeout_graceful_shutdown=3,
    )
