#!/bin/bash
# Spawn pose and vehicle model default to whatever
# catkin_ws/src/nidar_config/config/mission_config.yaml says, so moving the launch pad or
# switching the airframe is a one-line config edit rather than a hunt through shell scripts.
# Explicit positional arguments still override, and the hardcoded fallbacks after ":-" keep the
# script working if PyYAML or the config file is unavailable.
MISSION_CFG="$(dirname "$0")/../catkin_ws/src/nidar_config/config/mission_config.yaml"
cfg() {  # cfg <python-expression-over-`c`>  <fallback>
    python3 -c "
import yaml,sys
try:
    c=yaml.safe_load(open('$MISSION_CFG'))['nidar']
    print($1)
except Exception:
    print('$2')
" 2>/dev/null || echo "$2"
}
GUI_ARG=${1:-true}
SPAWN_X=${2:-$(cfg "c['launch_pad']['center']['x']" 0.0)}
SPAWN_Y=${3:-$(cfg "c['launch_pad']['center']['y']" -6.5)}
SPAWN_Z=${4:-$(cfg "round(c['launch_pad']['thickness']+c['vehicle']['belly_clearance'],3)" 0.19)}
SPAWN_YAW=${5:-$(cfg "c['launch_pad']['spawn_yaw']" 1.5708)}
# The ":-" fallback fires when PyYAML or mission_config.yaml is unavailable, so it must name the
# CURRENT airframe. It said iris_vlp16 well after the X500 migration, which would have silently
# flown a 0.386 m collision radius through an arena whose median corridor does not fit it.
VEHICLE=${VEHICLE:-$(cfg "c['vehicle']['model']" x500_vlp16)}
# RViz is controlled separately from the Gazebo GUI and defaults to OFF, because it is by far the
# most expensive optional process in this stack: measured at ~290% CPU, i.e. roughly 3 of this
# machine's 12 hardware threads, purely for visualisation. Leaving it off gives FAST-LIO and
# gzserver that headroom back. Set RVIZ=1 to bring the window back:
#     RVIZ=1 ./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708
RVIZ_ARG=${RVIZ:-0}
if [ "$RVIZ_ARG" = "1" ] || [ "$RVIZ_ARG" = "true" ]; then RVIZ_ARG=true; else RVIZ_ARG=false; fi
echo "Starting clean FUEL exploration test (GUI=${GUI_ARG}, vehicle=${VEHICLE}) at position (X=${SPAWN_X}, Y=${SPAWN_Y}, Z=${SPAWN_Z}, Yaw=${SPAWN_YAW})..."

# Render on the NVIDIA GPU. The X server behind DISPLAY is not NVIDIA-driven, so plain GLX falls
# back to Mesa llvmpipe (software, burns CPU). PRIME render offload routes GL to the RTX GPU.
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
export LIBGL_ALWAYS_SOFTWARE=0
# Ensure clean slate
# Refuse to fly a tree whose fixes are not actually in the binaries. On 2026-09-09
# entry_detection_module.py and fast_exploration_fsm.cpp were reverted twice (09:29:36 and
# 10:49:15) to a pre-handshake state; four runs were flown and analysed before anyone noticed,
# and one of them lost 136 s of its 164 s stall to the exact race the missing code prevents.
# Set SKIP_PARITY=1 to fly anyway (e.g. deliberately testing a baseline).
if [ "${SKIP_PARITY:-0}" != "1" ]; then
    if ! "$(dirname "$0")/verify_fix_parity.sh"; then
        echo
        echo "Refusing to launch: the code you think you are testing is not what would run."
        echo "  python3 scripts/apply_handshake_fix.py && catkin build exploration_manager"
        echo "  (or re-run with SKIP_PARITY=1 to fly the tree as-is)"
        exit 1
    fi
fi

killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
pkill -f flight_envelope_guard.py 2>/dev/null || true
pkill -f relay_odometry.py 2>/dev/null || true
pkill -f exploration_node 2>/dev/null || true
pkill -f traj_server 2>/dev/null || true
pkill -f waypoint_generator 2>/dev/null || true
pkill -f fast_lio 2>/dev/null || true
pkill -f FAST_LIO 2>/dev/null || true
pkill -f cpu_repin_loop.sh 2>/dev/null || true
# rviz and the telemetry logger were missing from this list, and both outlive a killed run:
# rviz is started by nidar_mapping.launch (rviz:=$GUI_ARG) and is not one of the `killall` names
# above, while mission_telemetry_logger.py runs in the foreground of the previous invocation.
# A surviving rviz idles at ~290% CPU and a surviving FAST-LIO publishes a SECOND, conflicting
# solution onto /Fast_LIO/odometry, which makes the next run's odometry jump between two
# estimates and look like a SLAM divergence. Observed on 2026-09-04: a re-run inherited 3 gzserver
# / 3 px4 / 2 fastlio_mapping processes and diverged to (-26, 56) within 30 s of takeoff.
pkill -f rviz 2>/dev/null || true
pkill -f mission_telemetry_logger.py 2>/dev/null || true
pkill -f flightlog/record.py 2>/dev/null || true
pkill -f flightlog/watchdog.py 2>/dev/null || true
pkill -f entry_detection_module.py 2>/dev/null || true
# robot_state_publisher / static_transform_publisher are started by nidar_mapping.launch and are
# not among the `killall` names either, so they accumulate across runs -- five of them were found
# alive with no simulation running.
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
# Give the SIGKILLs time to land before spawning replacements -- `killall` returns immediately and
# gzserver in particular (a /bin/sh wrapper plus a forked child) can outlive the call by a second.
sleep 3
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
roslaunch px4 mavros_posix_sitl.launch vehicle:=$VEHICLE world:=/home/developer/NIDAR/nidar_competition.world gui:=$GUI_ARG interactive:=false x:=$SPAWN_X y:=$SPAWN_Y z:=$SPAWN_Z Y:=$SPAWN_YAW > /tmp/sim_test.log 2>&1 &
SIM_PID=$!

echo "Waiting for MAVROS to connect to PX4 (up to 60 seconds)..."
MAVROS_CONNECTED=0
for i in {1..60}; do
    STATUS=$(python3 -c "import rospy; from mavros_msgs.msg import State; rospy.init_node('test_takeoff_state', anonymous=True); msg = rospy.wait_for_message('/mavros/state', State, timeout=5.0); print(msg.connected)" 2>/dev/null)
    if [ "$STATUS" = "True" ]; then
        echo "MAVROS Connected!"
        MAVROS_CONNECTED=1
        break
    fi
    sim_sleep 1
done

# Trust the loop's result: a second one-shot probe right after sim boot can
# time out under load and falsely report a disconnect.
if [ "$MAVROS_CONNECTED" != "1" ]; then
    echo "Error: MAVROS failed to connect to PX4. Exiting."
    killall -9 rosmaster rosout roslaunch gzserver gzclient px4 mavros_node rostopic px4-simulator_mavlink 2>/dev/null || true
    exit 1
fi

echo "Pinning simulation/flight-control processes to dedicated physical cores..."
pin_process "gzserver" "2,3"
pin_process "bin/px4" "4,5"
pin_process "mavros_node" "6,7"

echo "Launching FAST-LIO2 Mapping (RViz=${RVIZ_ARG}; set RVIZ=1 to show it)..."
roslaunch /home/developer/NIDAR/launch/fast_lio/nidar_mapping.launch rviz:=$RVIZ_ARG > /tmp/fast_lio.log 2>&1 &
sim_sleep 2
pin_process "fastlio_mapping" "0,1"

echo "Starting Odometry Relay (FAST-LIO -> PX4 EKF2)..."
/home/developer/NIDAR/catkin_ws/src/nidar_platform/scripts/relay_odometry.py > /tmp/relay.log 2>&1 &
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
rosparam load /home/developer/NIDAR/catkin_ws/src/nidar_config/config/flight_envelope_guard.yaml /

echo "Starting Flight Envelope Guard (FUEL -> MAVROS Execution Safety Layer)..."
/home/developer/NIDAR/catkin_ws/src/nidar_safety/scripts/flight_envelope_guard.py > /tmp/bridge.log 2>&1 &
sim_sleep 1
pin_process "flight_envelope_guard.py" "6,7"

echo "Starting CPU-core re-pin loop (catches worker threads gzserver/px4 spawn after the initial pin)..."
/home/developer/NIDAR/scripts/cpu_repin_loop.sh > /tmp/cpu_repin.log 2>&1 &

echo "Setting PX4 Takeoff Altitude parameter to 1.5m..."
rosrun mavros mavparam set MIS_TAKEOFF_ALT 1.5 >/dev/null 2>&1 || true

# FUEL starts before arming too. Its frontier box (box_min/max in nidar_fuel_upstream.launch)
# covers only the arena interior -- world y in [-6.8, 6.8], x in [-6.8, 6.8] -- so bringing it
# up early cannot produce frontiers outside the arena, and it stays idle until it receives a
# waypoint trigger. Starting it late is what broke the 08:30 run: the EDM crossed the door and
# published its handover trigger ~10 s after FUEL launched, before exploration_node had
# registered its subscriber, so the trigger was dropped and the vehicle hovered inside the
# arena on a stale setpoint for the rest of the run.
echo "============================================================"
echo "Launching Upstream FUEL Exploration Stack..."
echo "============================================================"
roslaunch /home/developer/NIDAR/launch/nidar_fuel_upstream.launch > /tmp/fuel.log 2>&1 &
FUEL_PID=$!
sim_sleep 5

# ---------------------------------------------------------------------------------------
# Flight recorder. Captures ONLY what dies with the simulation: the occupancy map, FUEL's
# commanded setpoints, the selected viewpoint and the coverage series. Ground truth is left
# to PX4's own .ulg (higher rate, already being written) and merged afterwards by pack.py.
#
# This exists because /tmp/fuel.log is TRUNCATED on every launch and the map was never saved
# at all, so a finished run left nothing an external evaluator could recompute coverage from.
# Set NIDAR_FLIGHTLOG=0 to skip it.
if [ "${NIDAR_FLIGHTLOG:-1}" != "0" ]; then
  python3 /home/developer/NIDAR/tools/flightlog/record.py > /tmp/flightlog.log 2>&1 &
  FLIGHTLOG_PID=$!
  echo "Flight recorder started (pid $FLIGHTLOG_PID) -> logs/runs/"

  # Watchdog. A PX4 crash DEADLOCKS this simulation rather than ending it -- lockstep means
  # gzserver waits forever for actuator outputs that a terminated controller never sends, so a
  # dead run sits at RTF ~0.005 until someone notices. This ends the run on a crash, on a PX4
  # failsafe announcement, on a stalled clock, or on disarm, packing the bundle before it
  # tears anything down. NIDAR_WATCHDOG=0 to disable, NIDAR_WATCHDOG_ARGS for thresholds.
  if [ "${NIDAR_WATCHDOG:-1}" != "0" ]; then
    python3 -u /home/developer/NIDAR/tools/flightlog/watchdog.py \
      ${NIDAR_WATCHDOG_ARGS:-} > /tmp/watchdog.log 2>&1 &
    WATCHDOG_PID=$!
    echo "Crash watchdog started (pid $WATCHDOG_PID) -> /tmp/watchdog.log"
  fi
fi

# ---------------------------------------------------------------------------------------
# The mission layer starts BEFORE arming, on purpose.
#
# It used to start after the vehicle was already airborne and in OFFBOARD. That left a ~10 s
# window in which the vehicle was flying with no mission module alive: the only setpoint
# source was flight_envelope_guard.py's fallback hover-hold, whose target is derived from
# /mavros/local_position/pose while EKF2 is still completing its external-vision alignment.
# On 2026-09-05 the vehicle drifted from world x=+0.60 to x=-1.04 during that window; the EDM
# then started, seeded its setpoint from wherever the vehicle had ended up, and drove that
# 1 m lateral error straight into the door jamb (the opening is only +-0.95 m wide).
# Starting the EDM first lets it hold the pad through the climb and own the whole approach.
# ---------------------------------------------------------------------------------------
# Mission layer: loads mission_config.yaml onto the parameter server and, when entry is
# enabled, starts the Entry Detection Module. ENTRY=0 skips it and falls back to the old
# behaviour of triggering FUEL immediately (useful when spawning inside the arena).
ENTRY_ARG=${ENTRY:-1}
if [ "$ENTRY_ARG" = "1" ] || [ "$ENTRY_ARG" = "true" ]; then ENTRY_ARG=true; else ENTRY_ARG=false; fi
echo "Launching NIDAR mission layer (entry_enabled=${ENTRY_ARG})..."
roslaunch nidar_mission nidar_mission.launch entry_enabled:=$ENTRY_ARG > /tmp/mission.log 2>&1 &
MISSION_PID=$!
sim_sleep 3

# Stream [SURVIVOR] and [Coverage] lines from the mission log to this shell's
# stdout. rospy.loginfo writes to /rosout only; ROS captures it into the node's
# log file but nothing prints it to the operator's terminal. Without this tail
# the only per-detection feedback the operator sees during a run is silence,
# which was the user's complaint after run 20260911_151851 -- five survivors
# had been detected and logged, but every one of them landed in the ROS log
# only and none in the terminal the operator was watching.
#
# Single awk process instead of `tail | grep | sed`: multi-process pipelines
# with backgrounded stages accrete buffering points that in practice never
# flush inside test_takeoff.sh (verified 2026-09-11 -- the previous
# tail|grep|sed variant produced zero output on stdout during the run, then
# the identical pipeline ran fine standalone against the same file after the
# run ended). One awk with fflush() after every print is deterministic.
awk '
  /\[SURVIVOR\]|\[Coverage\]|\[EDM\] State Transition|\[EDM\] DESCEND|\[EDM\] LAND|\[EDM\] RETURN/ {
    print "[mission] " $0
    fflush()
  }
' < <(tail -n0 -F /tmp/mission.log 2>/dev/null) &
MISSION_TAIL_PID=$!

# Auto-record the Phase 5 visualization topics into a rosbag so the 2D map,
# grid overlay and survivor tags can be replayed after the run. See
# scripts/view_map2d.sh --bag for the replay side.
# --lz4 halves the on-disk size; --split limits any single .bag file to 200 MB
# so a run that goes long doesn't produce one huge file that stalls rviz on
# open. The bag lands next to the ulog/summary bundle at run-end.
#
# NO /tf: run 20260911_153922 recorded 6204 /tf messages in 5 minutes; that is
# a continuous disk I/O storm that starves FAST-LIO's own writes and adds
# nothing to visualisation -- RViz reconstructs the world<->map chain from
# /tf_static alone (the dynamic /tf carries only camera_init -> base_link,
# which is the odometry we do not need for replay of the 2D floorplan). Dropping
# it takes the bag size from ~14 MB uncompressed to ~1 MB, and the recorder's
# CPU cost from measurable to negligible. view_map2d.sh --bag now publishes
# world_to_map itself at replay time so no chain is missing.
mkdir -p logs/bags 2>/dev/null || true
BAG_BASENAME="logs/bags/map2d_$(date +%Y%m%d_%H%M%S)"
rosbag record --lz4 --split --size=200 \
    -O "${BAG_BASENAME}" \
    /map_2d /grid_markers /survivor_tags /survivors /tf_static \
    > /tmp/rosbag_record.log 2>&1 &
BAG_PID=$!
echo "Recording visualization topics -> ${BAG_BASENAME}.bag (PID=${BAG_PID})"

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

if [ "$ENTRY_ARG" = "true" ]; then
    # The EDM owns the handover: it flies the vehicle from the pad through the arena opening
    # and only then publishes the FUEL waypoint trigger itself. Triggering FUEL here as well
    # would put traj_server and the EDM on /planning/pos_cmd simultaneously, which is exactly
    # the kind of two-publisher contention that has bitten this stack before.
    echo "Entry Detection Module active - it will trigger FUEL after crossing the arena door."
else

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
fi
sim_sleep 2

echo "============================================================"
echo "Clean Upstream FUEL Autonomous Exploration Running!"
echo "Architecture: FAST-LIO2 -> Upstream FUEL -> Flight Envelope Guard -> MAVROS -> PX4"
echo "============================================================"

/home/developer/NIDAR/scripts/mission_telemetry_logger.py

# The telemetry logger above runs in the foreground, so reaching here means the run is over.
# Stop the recorder cleanly (its shutdown hook writes a final flush) and consolidate the
# bundle while /tmp/fuel.log still holds THIS run -- the next launch truncates it.
if [ -n "${WATCHDOG_PID:-}" ]; then
  kill -TERM "$WATCHDOG_PID" 2>/dev/null || true
fi
if [ -n "${FLIGHTLOG_PID:-}" ]; then
  kill -INT "$FLIGHTLOG_PID" 2>/dev/null || true
  wait "$FLIGHTLOG_PID" 2>/dev/null || true
  python3 /home/developer/NIDAR/tools/flightlog/pack.py --fuel-log /tmp/fuel.log || true
fi

# Stop the mission-log tail cleanly (it would otherwise linger under the next
# run and double-print lines).
if [ -n "${MISSION_TAIL_PID:-}" ]; then
  kill "${MISSION_TAIL_PID}" 2>/dev/null || true
fi

# rosbag needs SIGINT to finish writing the trailing index cleanly. A SIGTERM
# leaves the file un-indexed and rviz refuses to open it. Wait a couple of
# seconds after SIGINT for rosbag to flush.
if [ -n "${BAG_PID:-}" ]; then
  kill -INT "${BAG_PID}" 2>/dev/null || true
  # Wait up to 5 s for rosbag to write the index and exit
  for _ in 1 2 3 4 5; do
    kill -0 "${BAG_PID}" 2>/dev/null || break
    sim_sleep 1
  done
  # Move the bag into the run bundle if we know where it went
  if compgen -G "${BAG_BASENAME}"*.bag > /dev/null 2>&1; then
    echo "Visualization bag(s):"
    ls -la ${BAG_BASENAME}*.bag 2>/dev/null | sed 's/^/  /'
    echo "Replay with: scripts/view_map2d.sh --bag ${BAG_BASENAME}.bag"
  fi
fi

echo "Simulation test execution complete."
