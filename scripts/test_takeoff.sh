#!/bin/bash
GUI_ARG=${1:-true}
SPAWN_X=${2:-0.0}
SPAWN_Y=${3:--6.5}
SPAWN_Z=${4:-0.1}
SPAWN_YAW=${5:-1.5708}
echo "Starting clean FUEL exploration test (GUI=${GUI_ARG}) at position (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=${SPAWN_Z}, Yaw=${SPAWN_YAW})..."

# Ensure clean slate
killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
pkill -f minimal_fuel_adapter.py 2>/dev/null || true
pkill -f flight_envelope_guard.py 2>/dev/null || true
pkill -f relay_odometry.py 2>/dev/null || true
pkill -f exploration_node 2>/dev/null || true
pkill -f traj_server 2>/dev/null || true
pkill -f waypoint_generator 2>/dev/null || true
pkill -f fast_lio 2>/dev/null || true
pkill -f FAST_LIO 2>/dev/null || true
pkill -f cpu_repin_loop.sh 2>/dev/null || true
rm -rf /home/developer/.ros/dataman /home/developer/.ros/eeprom /home/developer/.ros/parameters.bson /home/developer/.ros/parameters_backup.bson

sim_sleep() {
    local duration=$1
    python3 -c "import rospy; rospy.init_node('sim_sleep_node', anonymous=True); rospy.sleep($duration)" 2>/dev/null || sleep $duration
}

# CPU pinning (6 physical cores / 12 threads on this machine -- lscpu -p pairs: 0,1 / 2,3 / 4,5 / 6,7 / 8,9 / 10,11
# are hyperthread siblings of the same physical core, so pinning must stay aligned to those pairs to actually
# separate load across physical cores instead of just across threads of the same one).
# core0 (0,1): FAST-LIO -- most latency-critical, gets a dedicated core.
# core1 (2,3): gzserver -- physics + sensor sim.
# core2 (4,5): px4 SITL -- real-time flight control loop, needs low jitter.
# core3 (6,7): MAVROS + flight_envelope_guard.py + relay_odometry.py -- safety-critical bridge chain.
# core4-5 (8-11): left unpinned -- FUEL/exploration_manager, rosout, GUI rendering, logging, OS overhead.
pin_process() {
    # Pins every matching PID, not just the first -- some targets (e.g. gzserver, which roslaunch
    # starts via a /bin/sh wrapper that then forks the real binary as a separate child PID) run as a
    # wrapper+child pair, and affinity set on a parent after its child has already forked does not
    # retroactively apply to that child.
    #
    # For each matched PID, also pins every existing thread individually (/proc/<pid>/task/*), not just
    # the main thread: `taskset -pc <cores> <pid>` only sets affinity for that one thread's LWP: a
    # multithreaded process like gzserver (~15+ threads for physics/rendering/ROS) otherwise keeps
    # running its other threads unrestricted across all cores, silently defeating the pin.
    local pattern="$1" cores="$2" pid tid
    for pid in $(pgrep -f "$pattern"); do
        local pinned_any=0
        for tid in /proc/"$pid"/task/*; do
            [ -d "$tid" ] || continue
            taskset -pc "$cores" "$(basename "$tid")" >/dev/null 2>&1 && pinned_any=1
        done
        [ "$pinned_any" = "1" ] && echo "Pinned $pattern (pid $pid, all threads) -> cores $cores"
    done
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

echo "Pinning simulation/flight-control processes to dedicated physical cores..."
pin_process "gzserver" "2,3"
pin_process "bin/px4" "4,5"
pin_process "mavros_node" "6,7"

echo "Launching FAST-LIO2 Mapping & RViz..."
roslaunch /home/developer/NIDAR/launch/fast_lio/nidar_mapping.launch rviz:=$GUI_ARG > /tmp/fast_lio.log 2>&1 &
sim_sleep 2
pin_process "fastlio_mapping" "0,1"

echo "Starting Odometry Relay (FAST-LIO -> PX4 EKF2)..."
/home/developer/NIDAR/scripts/relay_odometry.py > /tmp/relay.log 2>&1 &
sim_sleep 2
pin_process "relay_odometry.py" "6,7"

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

verify_topic "/mavros/imu/data" 90 || exit 1
verify_topic "/velodyne_points" 90 || exit 1
verify_topic "/Fast_LIO/odometry" 90 || exit 1
verify_topic "/cloud_registered" 90 || exit 1

echo "Waiting for EKF Local Position Lock..."
for i in {1..300}; do
    POS=$(python3 -c "import rospy; from geometry_msgs.msg import PoseStamped; rospy.init_node('test_takeoff_pos', anonymous=True); msg = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=2.0); print('locked')" 2>/dev/null)
    if [ "$POS" = "locked" ]; then
        echo "Local Position Locked!"
        sim_sleep 2
        break
    fi
    sim_sleep 1
done

echo "Loading Flight Envelope Guard parameters onto ROS Parameter Server..."
rosparam load /home/developer/NIDAR/config/flight_envelope_guard.yaml /

echo "Starting Flight Envelope Guard (FUEL -> MAVROS Execution Safety Layer)..."
/home/developer/NIDAR/scripts/flight_envelope_guard.py > /tmp/bridge.log 2>&1 &
sim_sleep 1
pin_process "flight_envelope_guard.py" "6,7"

echo "Starting CPU-core re-pin loop (catches worker threads gzserver/px4 spawn after the initial pin)..."
/home/developer/NIDAR/scripts/cpu_repin_loop.sh > /tmp/cpu_repin.log 2>&1 &

echo "Setting PX4 Takeoff Altitude parameter to 1.5m..."
rosrun mavros mavparam set MIS_TAKEOFF_ALT 1.5 >/dev/null 2>&1 || true

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

echo "============================================================"
echo "Launching Upstream FUEL Exploration Stack..."
echo "============================================================"
roslaunch /home/developer/NIDAR/launch/nidar_fuel_upstream.launch > /tmp/fuel.log 2>&1 &
FUEL_PID=$!
sim_sleep 5

echo "Publishing trigger to start autonomous exploration..."
python3 -c "
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
rospy.init_node('exploration_trigger_publisher', anonymous=True)
pub = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
rate = rospy.Rate(2)
for i in range(15):
    p = Path()
    p.header.frame_id = 'camera_init'
    p.header.stamp = rospy.Time.now()
    ps = PoseStamped()
    ps.header = p.header
    ps.pose.position.x = 0.0
    ps.pose.position.y = 0.0
    ps.pose.position.z = 1.5
    ps.pose.orientation.w = 1.0
    p.poses.append(ps)
    pub.publish(p)
    rate.sleep()
print('[Trigger] Waypoint trigger published successfully.')
" > /tmp/trigger.log 2>&1
sim_sleep 2

echo "============================================================"
echo "Clean Upstream FUEL Autonomous Exploration Running!"
echo "Architecture: FAST-LIO2 -> Upstream FUEL -> Flight Envelope Guard -> MAVROS -> PX4"
echo "============================================================"

/home/developer/NIDAR/scripts/mission_telemetry_logger.py

echo "Simulation test execution complete."
