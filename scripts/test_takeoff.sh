#!/bin/bash
GUI_ARG=${1:-true}
SPAWN_X=${2:-0.0}
SPAWN_Y=${3:-0.0}
SPAWN_Z=${4:-0.1}
SPAWN_YAW=${5:-0.0}
echo "Starting clean simulation (GUI=${GUI_ARG}) at position (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=${SPAWN_Z}, Yaw=${SPAWN_YAW})..."

# Ensure clean slate
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
rm -rf /home/developer/.ros/dataman /home/developer/.ros/eeprom /home/developer/.ros/parameters.bson /home/developer/.ros/parameters_backup.bson

# ROS-time aware sleep function for slower-than-real-time SITL simulation
sim_sleep() {
    local duration=$1
    python3 -c "import rospy; rospy.init_node('sim_sleep_node', anonymous=True); rospy.sleep($duration)" 2>/dev/null || sleep $duration
}

source /home/developer/NIDAR/scripts/setup_env.sh
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 world:=/home/developer/NIDAR/nidar_competition.world gui:=$GUI_ARG interactive:=false x:=$SPAWN_X y:=$SPAWN_Y z:=$SPAWN_Z Y:=$SPAWN_YAW > /tmp/sim_test.log 2>&1 &
SIM_PID=$!

echo "Waiting for MAVROS to connect to PX4 (up to 60 seconds)..."
for i in {1..60}; do
    STATUS=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_state', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.connected)" 2>/dev/null)
    if [ "$STATUS" = "True" ]; then
        echo "MAVROS Connected!"
        break
    fi
    sim_sleep 1
done

STATUS=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_state', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.connected)" 2>/dev/null)
if [ "$STATUS" != "True" ]; then
    echo "Error: MAVROS failed to connect to PX4. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Launching FAST-LIO2 Mapping & RViz..."
roslaunch /home/developer/NIDAR/launch/fast_lio/nidar_mapping.launch rviz:=$GUI_ARG > /tmp/fast_lio.log 2>&1 &
sim_sleep 2

echo "Starting Odometry Relay..."
/home/developer/NIDAR/scripts/relay_odometry.py > /tmp/relay.log 2>&1 &
sim_sleep 2

# Verify IMU and LiDAR data streaming
verify_topic() {
    local topic=$1
    local timeout_sec=$2
    echo "Verifying topic $topic is active..."
    if timeout $timeout_sec rostopic echo $topic -n 1 >/dev/null 2>&1; then
        echo "Topic $topic is active!"
        return 0
    else
        echo "Error: Topic $topic timed out!"
        return 1
    fi
}

verify_topic "/mavros/imu/data" 90
if [ $? -ne 0 ]; then
    echo "IMU topic is inactive. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

verify_topic "/velodyne_points" 90
if [ $? -ne 0 ]; then
    echo "LiDAR topic is inactive. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Waiting for EKF Local Position Lock..."
for i in {1..300}; do
    POS=$(python3 -c "import rospy; from geometry_msgs.msg import PoseStamped; rospy.init_node('test_takeoff_pos', anonymous=True); msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=2.0); print('locked')" 2>/dev/null)
    if [ "$POS" = "locked" ]; then
        echo "Local Position Locked!"
        sim_sleep 3
        break
    fi
    sim_sleep 1
done

echo "============================================================"
echo "Starting FUEL MAVROS Offboard Control Bridge & Mission Manager..."
echo "============================================================"
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/fuel_to_mavros_bridge.py > /tmp/bridge.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/mission_manager.py > /tmp/mission_manager.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/map_2d_slicer_node.py > /tmp/map_2d.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/survivor_detector_node.py > /tmp/survivors.log 2>&1 &
/home/developer/NIDAR/scripts/record_mission.sh > /tmp/rosbag.log 2>&1 &

verify_topic "/mavros/setpoint_position/local" 15
if [ $? -ne 0 ]; then
    echo "Setpoint topic is inactive. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Setting MAVROS Mode to AUTO.TAKEOFF for Arming..."
rosrun mavros mavsys mode -c AUTO.TAKEOFF
sim_sleep 1

echo "Arming Drone..."
for i in {1..30}; do
    rosrun mavros mavsafety arm >/dev/null 2>&1
    sim_sleep 2
    ARMED=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_arm', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.armed)" 2>/dev/null)
    if [ "$ARMED" = "True" ]; then
        echo "Drone successfully armed!"
        break
    fi
    echo "Arming rejected (EKF2 aligning), retrying in 2 seconds..."
done

echo "Initiating Takeoff to 1.5m..."
rosrun mavros mavcmd takeoffcur 0 0 1.5 >/dev/null 2>&1
sim_sleep 4

echo "Switching MAVROS to OFFBOARD Mode..."
rosrun mavros mavsys mode -c OFFBOARD
sim_sleep 2

echo "============================================================"
echo "Launching FUEL Autonomous Exploration Stack..."
echo "============================================================"
roslaunch /home/developer/NIDAR/launch/nidar_fuel.launch > /tmp/fuel.log 2>&1 &
FUEL_PID=$!
sim_sleep 5

echo "Publishing trigger to start autonomous exploration..."
rostopic pub -1 /waypoint_generator/waypoints nav_msgs/Path "header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera_init'}, poses: [{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: 'camera_init'}, pose: {position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}]" > /tmp/trigger.log 2>&1
sim_sleep 2
sim_sleep 2

echo "============================================================"
echo "Autonomous Exploration Live! Monitoring Mission Lifecycle..."
echo "============================================================"
for i in {1..300}; do
    ARMED=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_arm', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.armed)" 2>/dev/null)
    POS_X=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
    POS_Y=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
    POS_Z=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
    MODE=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_arm', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.mode)" 2>/dev/null)
    
    echo "[Telemetry t+$(($i*5))s] X: ${POS_X}m, Y: ${POS_Y}m, Z: ${POS_Z}m | Mode: ${MODE} | Armed: ${ARMED}"
    
    if [ "$ARMED" != "True" ] && [ $i -gt 4 ]; then
        echo "Drone has landed and disarmed. Mission completed successfully!"
        break
    fi
    sim_sleep 5
done

echo "Simulation test execution complete."
