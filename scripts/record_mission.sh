#!/bin/bash
# Rosbag Mission Recorder for NIDAR AirMouse Project
# Logs all critical topics required for offline analysis, verification, and playback

BAG_DIR="/home/developer/NIDAR/rosbags"
mkdir -p "$BAG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="${BAG_DIR}/nidar_mission_${TIMESTAMP}.bag"

echo "============================================================"
echo "Starting NIDAR Rosbag Mission Recording..."
echo "Output file: ${BAG_NAME}"
echo "============================================================"

# Record critical SLAM, TF, telemetry, and setpoint topics
rosbag record -O "$BAG_NAME" \
    /Fast_LIO/odometry \
    /cloud_registered \
    /mavros/imu/data \
    /mavros/local_position/pose \
    /mavros/state \
    /mavros/battery/battery \
    /mavros/setpoint_raw/local \
    /mavros/setpoint_position/local \
    /planning/pos_cmd \
    /tf \
    /tf_static \
    /exploration_node/frontier_num \
    __name:=nidar_rosbag_recorder
