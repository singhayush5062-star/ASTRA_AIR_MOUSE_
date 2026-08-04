#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
if [ -f /home/developer/NIDAR/catkin_ws/devel/setup.bash ]; then
    source /home/developer/NIDAR/catkin_ws/devel/setup.bash
fi

PX4_DIR=/home/developer/NIDAR/simulation/PX4-Autopilot-v1.14.3
source ${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX4_DIR} ${PX4_DIR}/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PX4_DIR}:${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export PX4_SIM_MODEL=iris_vlp16
export SYS_AUTOSTART=1023

GUI_ARG=${1:-true}

echo "=========================================================================="
echo "Starting Obstacle Arena Simulation & Square Flight Test (GUI=$GUI_ARG)"
echo "=========================================================================="

echo "1. Launching PX4 SITL & Gazebo Classic (warehouse.world)..."
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 world:=${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/warehouse.world gui:=$GUI_ARG interactive:=false > /tmp/sim_test.log 2>&1 &
sleep 5

echo "2. Launching FAST-LIO2 Mapping & RViz Visualizer..."
roslaunch /home/developer/NIDAR/nidar_mapping.launch rviz:=$GUI_ARG > /tmp/fast_lio.log 2>&1 &
sleep 3

echo "3. Starting Vision Odometry Relay..."
python3 /home/developer/NIDAR/relay_odometry.py > /tmp/relay.log 2>&1 &
sleep 2

echo "4. Executing Autonomous Square Flight Script..."
python3 /home/developer/NIDAR/scripts/robust_square_flight.py

echo "=========================================================================="
echo "Test Execution Finished!"
echo "=========================================================================="
