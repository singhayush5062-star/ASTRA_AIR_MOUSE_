#!/bin/bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Export Gazebo Model & Plugin paths
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$ROOT_DIR/simulation/custom_models:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:+$GAZEBO_PLUGIN_PATH:}$ROOT_DIR/catkin_ws/devel/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROOT_DIR/catkin_ws/devel/lib"

MODEL="${1:-gazebo-classic_iris_vlp16}"

cd "$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3"

echo "Building PX4 SITL for target: $MODEL..."

DONT_RUN=1 make px4_sitl "$MODEL"

echo "PX4 build completed successfully for $MODEL."
