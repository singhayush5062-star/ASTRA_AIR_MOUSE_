#!/bin/bash
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/noetic/setup.bash

if [ -f "$ROOT_DIR/catkin_ws/devel/setup.bash" ]; then
    source "$ROOT_DIR/catkin_ws/devel/setup.bash"
else
    echo "Notice: $ROOT_DIR/catkin_ws/devel/setup.bash is missing. Running 'catkin build' to relink merged devel space and generate setup environment..."
    (cd "$ROOT_DIR/catkin_ws" && catkin build)
    if [ -f "$ROOT_DIR/catkin_ws/devel/setup.bash" ]; then
        source "$ROOT_DIR/catkin_ws/devel/setup.bash"
    else
        echo "ERROR: Failed to generate $ROOT_DIR/catkin_ws/devel/setup.bash! Run 'catkin build' manually inside catkin_ws."
        exit 1
    fi
fi

export DISPLAY="${DISPLAY:-:1}"
# Gazebo rendering path. NIDAR_GAZEBO_GPU=1 (default) uses the NVIDIA GLX vendor,
# which requires the container to have been launched with `--gpus all` and
# NVIDIA_DRIVER_CAPABILITIES including 'graphics' (both handled by
# scripts/docker_dev_start.sh and docker/Dockerfile).
#
# WHY IT MATTERS. At LIBGL_ALWAYS_SOFTWARE=1 gzserver renders through llvmpipe
# and can burn 200-300% CPU on the scene alone at the 250 Hz world update rate
# in nidar_competition.world -- competing with FAST-LIO and FUEL for the same
# cores. On the RTX 3050 laptop this is the difference between a healthy 8-12
# Hz FAST-LIO and one that starves under detector+SLAM load (Phase 4 §4.2).
#
# Set NIDAR_GAZEBO_GPU=0 to fall back to software rendering (headless CI, or
# a host with a broken NVIDIA driver). If the GPU init fails at Gazebo start,
# the symptoms are `libEGL` warnings and a black gzclient window; toggle this
# back to 0 to recover.
if [ "${NIDAR_GAZEBO_GPU:-1}" = "1" ]; then
    export LIBGL_ALWAYS_SOFTWARE=0
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
else
    export LIBGL_ALWAYS_SOFTWARE=1
fi
export QT_X11_NO_MITSHM=1

export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$ROOT_DIR/catkin_ws/src/nidar_sim/models:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ROOT_DIR/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$ROOT_DIR/catkin_ws/devel/lib:${LD_LIBRARY_PATH}"
export GAZEBO_PLUGIN_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}"
