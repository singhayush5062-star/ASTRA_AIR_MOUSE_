#!/bin/bash
GUI_ARG=${1:-true}
SPAWN_X=${2:-0.0}
SPAWN_Y=${3:-0.0}
SPAWN_Z=${4:-0.1}
SPAWN_YAW=${5:-0.0}
echo "Starting Clean Simulation & Square Flight Test in Obstacle Arena (GUI=${GUI_ARG}) at (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=${SPAWN_Z}, Yaw=${SPAWN_YAW})..."

# Kill lingering ROS nodes
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true

sim_sleep() {
    local duration=$1
    python3 -c "import rospy; rospy.init_node('sim_sleep_node', anonymous=True); rospy.sleep($duration)" 2>/dev/null || sleep $duration
}

source /home/developer/NIDAR/scripts/setup_env.sh

echo "1. Launching PX4 SITL & Gazebo Classic (nidar_competition.world)..."
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_vlp16 world:=/home/developer/NIDAR/nidar_competition.world gui:=$GUI_ARG interactive:=false x:=$SPAWN_X y:=$SPAWN_Y z:=$SPAWN_Z Y:=$SPAWN_YAW > /tmp/sim_test.log 2>&1 &
SIM_PID=$!

echo "Waiting for MAVROS FCU Connection..."
for i in {1..60}; do
    STATUS=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_state', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=2.0); print(msg.connected)" 2>/dev/null)
    if [ "$STATUS" = "True" ]; then
        echo "MAVROS FCU Connected!"
        break
    fi
    sim_sleep 1
done

echo "2. Launching FAST-LIO2 Mapping & RViz Visualizer..."
roslaunch /home/developer/NIDAR/nidar_mapping.launch rviz:=$GUI_ARG > /tmp/fast_lio.log 2>&1 &
sim_sleep 3

echo "3. Starting Odometry Relay..."
/home/developer/NIDAR/relay_odometry.py > /tmp/relay.log 2>&1 &
sim_sleep 2

echo "4. Executing Robust Square Waypoint Flight Script (3m x 3m Square Pattern)..."
/home/developer/NIDAR/scripts/robust_square_flight.py
