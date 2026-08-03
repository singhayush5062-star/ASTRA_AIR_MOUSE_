#!/bin/bash
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Clean stale instances
pkill -9 px4 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true

# Source setup environment
source "$ROOT_DIR/scripts/setup_env.sh"

# Launch SITL + MAVROS + Gazebo
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 "$@"
