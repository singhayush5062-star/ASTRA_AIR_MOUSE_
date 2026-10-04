#!/bin/bash
# Start the NIDAR GCS: backend + web UI on http://localhost:8000 (one process, one port).
#
#   catkin_ws/src/nidar_gcs/scripts/start_gcs.sh
#
# Then open http://localhost:8000 -> SIMULATION -> START. The START button runs
# scripts/test_takeoff.sh (headless; set SIM_GAZEBO_GUI=True in backend/.env for the Gazebo window),
# RESET stops every simulation process. A simulation already running (e.g. started from a
# terminal) is picked up automatically. Stopping the GCS (Ctrl-C) leaves a running simulation up.
#
# First time on a machine: catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh
set -e
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(cd "$PKG_DIR/../../.." && pwd)"
GCS_HOME="${NIDAR_GCS_HOME:-$HOME/.local/share/nidar_gcs}"

if [ ! -d "$GCS_HOME/pydeps/fastapi" ]; then
    echo "[start_gcs] backend dependencies missing -- run: $PKG_DIR/scripts/setup_gcs.sh"
    exit 1
fi
if [ ! -f "$PKG_DIR/frontend/dist/index.html" ]; then
    echo "[start_gcs] web UI not built -- run: $PKG_DIR/scripts/setup_gcs.sh"
    exit 1
fi

# The ROS environment the simulation and the ROS bridge run in (inherited by both).
set +e
source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/catkin_ws/devel/setup.bash"
set -e

PORT=$(grep -E '^PORT=' "$PKG_DIR/backend/.env" 2>/dev/null | cut -d= -f2)
PORT=${PORT:-8000}
if python3 -c "import socket,sys; s=socket.socket(); sys.exit(0 if s.connect_ex(('127.0.0.1',$PORT))==0 else 1)"; then
    echo "[start_gcs] port $PORT is already in use (another GCS running?)"
    exit 1
fi

# Real drone over a serial radio (T12 / SiK) or the FC's USB port: the two usual blockers.
if [ "$(id -u)" != "0" ] && ! id -nG | grep -qw dialout; then
    echo "[start_gcs] WARNING: $(id -un) is not in the dialout group -- serial ports will not open"
    echo "            (sudo usermod -aG dialout \$USER, log out/in; in the dev container: recreate it)"
fi
if command -v systemctl > /dev/null 2>&1 && systemctl is-active --quiet ModemManager 2>/dev/null; then
    echo "[start_gcs] WARNING: ModemManager is running and grabs new USB serial devices for a few"
    echo "            seconds (sudo systemctl disable --now ModemManager)"
fi

echo "[start_gcs] NIDAR GCS on http://localhost:$PORT  (Ctrl-C to stop the GCS; the sim keeps running)"
for ip in $(hostname -I 2>/dev/null); do echo "[start_gcs]   from another device: http://$ip:$PORT"; done
exec python3 "$PKG_DIR/backend/run.py"
