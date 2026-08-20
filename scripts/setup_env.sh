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
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1

export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$ROOT_DIR/simulation/custom_models:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_PLUGIN_PATH="$ROOT_DIR/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$ROOT_DIR/catkin_ws/devel/lib:${LD_LIBRARY_PATH}"
export GAZEBO_PLUGIN_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}"
export LD_LIBRARY_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}"
