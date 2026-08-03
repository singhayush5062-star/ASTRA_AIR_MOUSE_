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
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 world:=/home/developer/NIDAR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/warehouse.world gui:=$GUI_ARG interactive:=false > /tmp/sim_test.log 2>&1 &
SIM_PID=$!

echo "Waiting for MAVROS to connect to PX4 (up to 60 seconds)..."
for i in {1..60}; do
    STATUS=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected: True")
    if [ ! -z "$STATUS" ]; then
        echo "MAVROS Connected!"
        break
    fi
    sim_sleep 1
done

STATUS=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "connected: True")
if [ -z "$STATUS" ]; then
    echo "Error: MAVROS failed to connect to PX4. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Launching FAST-LIO2 Mapping..."
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

echo "Waiting for EKF Local Position Lock (up to 300 seconds)..."
for i in {1..300}; do
    POS=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep "position:")
    if [ ! -z "$POS" ]; then
        echo "Local Position Locked!"
        break
    fi
    sim_sleep 1
done

echo "Setting Mode to AUTO.LOITER..."
rosrun mavros mavsys mode -c AUTO.LOITER
sim_sleep 2

echo "Arming Drone (attempting up to 30 times)..."
for i in {1..30}; do
    rosrun mavros mavsafety arm >/dev/null 2>&1
    sim_sleep 2
    ARMED=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "armed: True")
    if [ ! -z "$ARMED" ]; then
        echo "Drone successfully armed!"
        break
    fi
    echo "Arming rejected (EKF2 aligning), retrying in 2 seconds..."
done

ARMED=$(rostopic echo /mavros/state -n 1 2>/dev/null | grep "armed: True")
if [ -z "$ARMED" ]; then
    echo "Failed to arm drone after 30 attempts. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Taking off to 3 meters..."
rosrun mavros mavcmd takeoffcur 0 0 3.0

echo "Waiting for drone to reach target altitude..."
for i in {1..120}; do
    ALT=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
    if [ ! -z "$ALT" ]; then
        echo "Current altitude: $ALT meters"
        REACHED=$(python3 -c "import sys; print(1 if float(sys.argv[1]) >= 1.5 else 0)" "$ALT")
        if [ "$REACHED" -eq 1 ]; then
            echo "Target altitude reached!"
            break
        fi
    fi
    sim_sleep 1
done

echo "Current Drone Altitude (Z position):"
rostopic echo /mavros/local_position/pose -n 1 | grep -A 3 "position:"

echo "============================================================"
echo "Starting FUEL MAVROS Offboard Control Bridge..."
echo "============================================================"
python3 /home/developer/NIDAR/scripts/fuel_to_mavros_bridge.py > /tmp/bridge.log 2>&1 &
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
echo "Autonomous Exploration Live! Monitoring Flight Telemetry..."
echo "============================================================"
for i in {1..30}; do
    VEL=$(rostopic echo /mavros/local_position/velocity_local -n 1 2>/dev/null | grep -A 3 "linear:" | grep "x:" | awk '{print $2}')
    POS=$(rostopic echo /mavros/local_position/pose -n 1 2>/dev/null | grep -A 3 "position:" | tr '\n' ' ')
    if [ ! -z "$VEL" ]; then
        echo "[Telemetry @ t+${i}s] Velocity X: ${VEL} m/s | Pose: ${POS}"
    fi
    sim_sleep 2
done

echo "Simulation test complete. Terminating nodes cleanly..."
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink traj_server exploration_node 2>/dev/null || true
