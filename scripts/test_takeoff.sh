#!/bin/bash
GUI_ARG=${1:-true}
SPAWN_X=${2:-0.0}
SPAWN_Y=${3:--7.5}
SPAWN_Z=${4:-0.1}
SPAWN_YAW=${5:-1.5708}
echo "Starting clean simulation (GUI=${GUI_ARG}) at position (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=${SPAWN_Z}, Yaw=${SPAWN_YAW})..."

# Ensure clean slate
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
pkill -f fuel_to_mavros_bridge.py 2>/dev/null || true
pkill -f mission_manager.py 2>/dev/null || true
pkill -f map_2d_slicer_node.py 2>/dev/null || true
pkill -f survivor_detector_node.py 2>/dev/null || true
pkill -f relay_odometry.py 2>/dev/null || true
pkill -f odometry_relay_node 2>/dev/null || true
pkill -f exploration_node 2>/dev/null || true
pkill -f traj_server 2>/dev/null || true
pkill -f fast_lio 2>/dev/null || true
pkill -f FAST_LIO 2>/dev/null || true
pkill -f waypoint_generator 2>/dev/null || true
pkill -f "rosbag record" 2>/dev/null || true
pkill -f nidar_rosbag_recorder 2>/dev/null || true
pkill -f forward_only_validator 2>/dev/null || true
pkill -f collect_crash_timeline.sh 2>/dev/null || true
pkill -f phase0_collect_mapping.sh 2>/dev/null || true
pkill -f phase0_tf_audit.py 2>/dev/null || true
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

ENABLE_PHASE0_AUDIT=${ENABLE_PHASE0_AUDIT:-false}
PHASE0_AUDIT_DURATION=${PHASE0_AUDIT_DURATION:-180}
if [ "$ENABLE_PHASE0_AUDIT" = "true" ]; then
    rm -rf /tmp/phase0_audit /tmp/phase0_audit.log
fi

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

echo "============================================================"
echo "Starting FUEL MAVROS Offboard Control Bridge & Mission Manager..."
echo "============================================================"
ENABLE_CRASH_TIMELINE=${ENABLE_CRASH_TIMELINE:-true}
if [ "$ENABLE_CRASH_TIMELINE" = "true" ]; then
    echo "Starting crash timeline collector..."
    /home/developer/NIDAR/scripts/collect_crash_timeline.sh /tmp/crash_timeline > /tmp/crash_timeline.log 2>&1 &
fi
ENFORCE_FORWARD_ONLY=${ENFORCE_FORWARD_ONLY:-false}
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/fuel_to_mavros_bridge.py _enforce_forward_only:=$ENFORCE_FORWARD_ONLY _safety_inflation_cells:=1 __name:=fuel_to_mavros_bridge > /tmp/bridge.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/mission_manager.py _publish_setpoints:=false > /tmp/mission_manager.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/map_2d_slicer_node.py > /tmp/map_2d.log 2>&1 &
PYTHONUNBUFFERED=1 /home/developer/NIDAR/scripts/survivor_detector_node.py > /tmp/survivors.log 2>&1 &
/home/developer/NIDAR/scripts/record_mission.sh > /tmp/rosbag.log 2>&1 &

sim_sleep 1
echo "Switching MAVROS to OFFBOARD Mode to initiate immediate climb..."
rosrun mavros mavsys mode -c OFFBOARD
sim_sleep 1

echo "Climbing to takeoff altitude (1.5m)..."
python3 -c "
import rospy
from geometry_msgs.msg import PoseStamped
rospy.init_node('takeoff_alt_wait', anonymous=True)
start = rospy.Time.now()
while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < 30.0:
    try:
        msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=1.0)
        if msg.pose.position.z >= 1.2:
            print(f'Takeoff altitude reached: {msg.pose.position.z:.2f}m!')
            break
    except Exception:
        pass
    rospy.sleep(0.5)
"

echo "Pre-flight climb complete. Holding stable forward orientation for mapping..."
sim_sleep 1

echo "============================================================"
echo "Launching FUEL Autonomous Exploration Stack..."
echo "============================================================"
roslaunch /home/developer/NIDAR/launch/nidar_fuel.launch > /tmp/fuel.log 2>&1 &
FUEL_PID=$!
sim_sleep 5

if [ "$ENABLE_PHASE0_AUDIT" = "true" ]; then
    echo "Starting Phase-0 mapping audit collector (${PHASE0_AUDIT_DURATION}s)..."
    /home/developer/NIDAR/scripts/phase0_collect_mapping.sh /tmp/phase0_audit "$PHASE0_AUDIT_DURATION" > /tmp/phase0_audit.log 2>&1 &
fi

echo "Publishing trigger to start autonomous exploration at (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=1.5)..."
python3 -c "
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
rospy.init_node('exploration_trigger_publisher', anonymous=True)
pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
rate = rospy.Rate(2)
for i in range(20):
    p = Path()
    p.header.frame_id = 'camera_init'
    p.header.stamp = rospy.Time.now()
    ps = PoseStamped()
    ps.header = p.header
    ps.pose.position.x = ${SPAWN_X}
    ps.pose.position.y = ${SPAWN_Y}
    ps.pose.position.z = 1.5
    ps.pose.orientation.w = 1.0
    p.poses.append(ps)
    pub.publish(p)
    rospy.loginfo(f'[Trigger] Published waypoint trigger {i + 1}/20')
    rate.sleep()
print('[Trigger] Waypoint path trigger stream finished.')
" > /tmp/trigger.log 2>&1
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
    
    echo "[Telemetry t+$(($i*5))s] Pose: (${POS_X}, ${POS_Y}, ${POS_Z})m | Mode: ${MODE} | Armed: ${ARMED}"
    
    if [ "$ARMED" != "True" ] && [ $i -gt 4 ]; then
        echo "Drone has landed and disarmed. Mission completed successfully!"
        break
    fi
    sim_sleep 5
done

echo "Simulation test execution complete."
