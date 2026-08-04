#!/bin/bash
GUI_ARG=${1:-true}
echo "Starting clean simulation (GUI=${GUI_ARG})..."

# Ensure clean slate
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true

# ROS-time aware sleep function for slower-than-real-time SITL simulation
sim_sleep() {
    local duration=$1
    python3 -c "import rospy; rospy.init_node('sim_sleep_node', anonymous=True); rospy.sleep($duration)" 2>/dev/null || sleep $duration
}

source /home/developer/NIDAR/scripts/setup_env.sh
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 world:=/home/developer/NIDAR/nidar_competition.world gui:=$GUI_ARG interactive:=false > /tmp/sim_test.log 2>&1 &
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
roslaunch /home/developer/NIDAR/nidar_mapping.launch rviz:=$GUI_ARG > /tmp/fast_lio.log 2>&1 &
sim_sleep 2

echo "Starting Odometry Relay..."
/home/developer/NIDAR/relay_odometry.py > /tmp/relay.log 2>&1 &
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
        break
    fi
    sim_sleep 1
done

echo "Setting Mode to AUTO.LOITER..."
rosrun mavros mavsys mode -c AUTO.LOITER
sim_sleep 2

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

ARMED=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_arm', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.armed)" 2>/dev/null)
if [ "$ARMED" != "True" ]; then
    echo "Failed to arm drone after 30 attempts. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Taking off to 1.5 meters..."
rosrun mavros mavcmd takeoffcur 0 0 1.5

echo "Waiting for drone to reach target altitude..."
for i in {1..30}; do
    ALT=$(python3 -c "import rospy; from geometry_msgs.msg import PoseStamped; rospy.init_node('test_takeoff_alt', anonymous=True); msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=2.0); print(msg.pose.position.z)" 2>/dev/null)
    if [ ! -z "$ALT" ]; then
        echo "Current altitude: $ALT meters"
        REACHED=$(python3 -c "import sys; print(1 if float(sys.argv[1]) >= 1.0 else 0)" "$ALT")
        if [ "$REACHED" -eq 1 ]; then
            echo "Target altitude reached!"
            break
        fi
    fi
    sim_sleep 1
done

echo "============================================================"
echo "Starting FUEL MAVROS Offboard Control Bridge..."
echo "============================================================"
/home/developer/NIDAR/scripts/fuel_to_mavros_bridge.py > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!
sim_sleep 2

echo "Switching MAVROS to OFFBOARD Mode..."
rosrun mavros mavsys mode -c OFFBOARD
sim_sleep 2

echo "============================================================"
echo "Launching FUEL Autonomous Exploration Stack..."
echo "============================================================"
roslaunch /home/developer/NIDAR/launch/nidar_fuel.launch > /tmp/fuel.log 2>&1 &
FUEL_PID=$!
sim_sleep 3

echo "============================================================"
echo "Autonomous Exploration Live! Monitoring Forward Flight Telemetry..."
echo "============================================================"
for i in {1..30}; do
    VEL=$(rostopic echo /mavros/local_position/velocity_local -n 1 2>/dev/null | grep -A 3 "linear:" | grep "x:" | awk '{print $2}')
    POS_X=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
    POS_Y=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
    POS_Z=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
    echo "[Telemetry t+${i}s] X: ${POS_X}m, Y: ${POS_Y}m, Z: ${POS_Z}m | Vel X: ${VEL} m/s"
    sim_sleep 2
done

echo "Simulation test execution complete."
