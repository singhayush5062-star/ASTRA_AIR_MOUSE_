#!/bin/bash
# Bring up the NIDAR stack on the REAL drone (run on the Jetson, inside the onboard container).
#
#   catkin_ws/src/nidar_bringup/scripts/hw_bringup.sh            # full stack
#   CAMERA=0 ./hw_bringup.sh                                     # without camera/detector
#   LIDAR=0 ./hw_bringup.sh                                      # bench: FC + rangefinder only
#   SKIP_PARITY=1 ./hw_bringup.sh                                # skip the build parity gate
#
# The hardware counterpart of test_takeoff.sh: same stack, same order, same readiness waits,
# with the physical sensors in place of Gazebo -- and it NEVER ARMS. When everything is up it
# reports READY; the operator presses TAKEOFF on the GCS (Hardware page), and the onboard
# mission commander arms and hands the vehicle to the mission manager.
#
# Settings come from catkin_ws/src/nidar_config/config/hardware.yaml (+ mission_config.yaml).
# Logs: logs/hw/<run>/  (one file per process) and a rosbag of the mission topics.
# Ctrl-C stops the stack. If the drone is flying, PX4's offboard-loss failsafe lands it
# (COM_OBL_RC_ACT) -- stop it on the ground.
set -u
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SRC="$ROOT_DIR/catkin_ws/src"
HW_CFG="$SRC/nidar_config/config/hardware.yaml"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT_DIR/logs/hw/$RUN_ID"
mkdir -p "$LOG_DIR"
PIDS=()

say()  { echo "[hw_bringup] $*"; }
die()  { echo "[hw_bringup] ERROR: $*" >&2; cleanup; exit 1; }
hw()   { python3 -c "
import yaml
c = yaml.safe_load(open('$HW_CFG'))['hardware']
v = $1
print('' if v is None else v)"; }

cleanup() {
    trap - INT TERM
    if [ ${#PIDS[@]} -gt 0 ]; then
        say "stopping the stack (logs: $LOG_DIR)"
        [ -n "${BAG_PID:-}" ] && kill -INT "$BAG_PID" 2>/dev/null
        sleep 2
        for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
        sleep 3
        for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
    fi
}
trap 'cleanup; exit 130' INT TERM

start() {  # start <logname> <command...>
    local name=$1; shift
    "$@" > "$LOG_DIR/$name.log" 2>&1 &
    PIDS+=($!)
    say "started $name (pid $!) -> logs/hw/$RUN_ID/$name.log"
}

wait_topic() {  # wait_topic <topic> <timeout_s> <what>
    say "waiting for $3 ($1, up to $2 s)..."
    timeout "$2" rostopic echo -n 1 "$1" > /dev/null 2>&1 || die "$3 not publishing on $1 (see $LOG_DIR)"
    say "  $1 OK"
}

# ── 0. configuration ────────────────────────────────────────────────────────────────────────
[ -f "$HW_CFG" ] || die "missing $HW_CFG"
FCU_URL=$(hw "c['fcu']['url']")
GCS_URL=$(hw "c['fcu'].get('gcs_url') or ''")
JETSON_IP=$(hw "c['network']['jetson_ip']")
LIDAR_IP=$(hw "c['lidar']['lidar_ip']")
TIME_SYNC=$(hw "c['lidar'].get('time_sync', 'restamp')")
IMU_SOURCE=$(hw "c['lidar'].get('imu_source', 'livox')")
CAM_ENABLED=$(hw "str(bool(c['camera'].get('enabled', True))).lower()")
CAM_SOURCE=$(hw "c['camera']['source']")
RNG_SOURCE=$(hw "c['rangefinder']['source']")
RNG_PORT=$(hw "c['rangefinder'].get('serial_port', '')")
MODEL=$(hw "c['perception']['model']")
[ "${MODEL:0:1}" = "/" ] || MODEL="$ROOT_DIR/$MODEL"
FL_FILTER_NUM=$(hw "c['lidar'].get('fastlio', {}).get('point_filter_num', 4)")
FL_FILTER_SIZE=$(hw "c['lidar'].get('fastlio', {}).get('filter_size', 0.5)")
LIDAR_ARG=true; [ "${LIDAR:-1}" = "0" ] && LIDAR_ARG=false
[ "${CAMERA:-1}" = "0" ] && CAM_ENABLED=false
IMU_TOPIC=/nidar/livox/imu; [ "$IMU_SOURCE" = "fcu" ] && IMU_TOPIC=/mavros/imu/data

say "run $RUN_ID  fcu=$FCU_URL  gcs_url=${GCS_URL:-none}  lidar=$LIDAR_ARG($LIDAR_IP, imu=$IMU_SOURCE, sync=$TIME_SYNC)  camera=$CAM_ENABLED($CAM_SOURCE)  range=$RNG_SOURCE"

# ── 1. pre-flight checks before anything starts ─────────────────────────────────────────────
source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/catkin_ws/devel/setup.bash" || die "catkin_ws not built (cd catkin_ws && catkin build)"
python3 "$SRC/nidar_config/scripts/apply_hardware_config.py" --check > "$LOG_DIR/config_check.log" 2>&1 \
    || die "generated hardware files are stale or hardware.yaml is invalid:
$(cat "$LOG_DIR/config_check.log")
  fix: python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py, then reload the PX4 params if they changed"
FCU_DEV=${FCU_URL%%:*}
if [ "${FCU_DEV:0:5}" = "/dev/" ]; then
    [ -e "$FCU_DEV" ] || die "flight controller port $FCU_DEV does not exist (ls -l /dev/ttyAMA* /dev/ttyTHS* /dev/ttyACM*; Pi 4: dtoverlay=disable-bt)"
    [ -r "$FCU_DEV" ] && [ -w "$FCU_DEV" ] || die "no permission on $FCU_DEV (add the user to dialout, or run the container with --privileged)"
fi
if [ "$RNG_SOURCE" = "serial" ] && [ ! -e "$RNG_PORT" ]; then
    die "rangefinder port $RNG_PORT does not exist"
fi
if [ "$LIDAR_ARG" = "true" ]; then
    rospack find livox_ros_driver2 > /dev/null 2>&1 \
        || die "livox_ros_driver2 is not built (hardware/DEPLOYMENT.md §4.2), or run with LIDAR=0"
    ping -c 2 -W 1 "$LIDAR_IP" > /dev/null 2>&1 \
        || die "Mid-360 at $LIDAR_IP does not answer ping: check power, the Ethernet cable and the onboard computer's static IP ($(hw "c['lidar']['host_ip']"))"
fi
if [ "$CAM_ENABLED" = "true" ] && [ "${CAM_SOURCE:0:10}" = "/dev/video" ] && [ ! -e "$CAM_SOURCE" ]; then
    die "camera $CAM_SOURCE does not exist (v4l2-ctl --list-devices), or run with CAMERA=0"
fi
if [ "$CAM_ENABLED" = "true" ] && [ ! -e "$MODEL" ]; then
    die "detector model $MODEL not found (hardware.yaml perception.model)"
fi
if [ "${SKIP_PARITY:-0}" != "1" ]; then
    "$SRC/nidar_qa/scripts/verify_fix_parity.sh" > "$LOG_DIR/parity.log" 2>&1 \
        || die "build parity check failed (see $LOG_DIR/parity.log); rebuild, or SKIP_PARITY=1"
fi

# ── 2. ROS master ───────────────────────────────────────────────────────────────────────────
# Advertise the Jetson's GCS-network address so the laptop's ROS link can reach every node.
if ip -4 addr 2>/dev/null | grep -q " $JETSON_IP/"; then
    export ROS_IP="$JETSON_IP"
else
    say "WARNING: $JETSON_IP is not on any interface; the GCS ROS link will not reach this onboard computer (set hardware.yaml network.jetson_ip)"
fi
export ROS_MASTER_URI=http://localhost:11311
unset ROS_HOSTNAME
if ! rosnode list > /dev/null 2>&1; then
    start roscore roscore
    for _ in $(seq 1 30); do rosnode list > /dev/null 2>&1 && break; sleep 0.5; done
fi
rosparam set /use_sim_time false

# ── 3. drivers: MAVROS, LiDAR, rangefinder, camera, static TF ─────────────────────────────────
start drivers roslaunch nidar_hardware hw_drivers.launch fcu_url:="$FCU_URL" gcs_url:="$GCS_URL" \
    lidar:="$LIDAR_ARG" time_sync:="$TIME_SYNC" camera:="$CAM_ENABLED"
say "waiting for MAVROS to connect to PX4..."
DRIVERS_PID=${PIDS[-1]}
for i in $(seq 1 60); do
    timeout 3 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q "connected: True" && break
    [ "$i" = 60 ] && die "MAVROS never connected to PX4 on $FCU_URL (baud = SER_TEL2_BAUD? TX/RX swapped? MAV_1_CONFIG = TELEM2?)"
    sleep 1
done
say "  MAVROS connected"
wait_topic /tfmini/range 20 "TFmini rangefinder"
if [ "$LIDAR_ARG" != "true" ]; then
    say "LIDAR=0: drivers only (bench mode). Ctrl-C to stop."
    wait "$DRIVERS_PID"; cleanup; exit 0
fi
wait_topic /nidar/livox/imu 30 "Mid-360 IMU"
wait_topic /nidar/livox/lidar 30 "Mid-360 scans"
[ "$CAM_ENABLED" = "true" ] && wait_topic /camera/image_raw 30 "camera"

# ── 4. localisation: FAST-LIO -> PX4 EKF2 ─────────────────────────────────────────────────────
start fast_lio roslaunch nidar_slam nidar_mapping.launch rviz:=false use_urdf:=false \
    config:="$SRC/nidar_slam/config/fast_lio/nidar_hw.yaml" \
    point_filter_num:="$FL_FILTER_NUM" filter_size:="$FL_FILTER_SIZE"
wait_topic /Fast_LIO/odometry 60 "FAST-LIO odometry"
start relay_odometry "$SRC/nidar_platform/scripts/relay_odometry.py"
say "waiting for PX4 to take the vision pose (EKF2 local position)..."
wait_topic /mavros/local_position/pose 60 "PX4 local position (EKF2_EV_CTRL set? params loaded?)"

# ── 5. safety, planner, mission layer ────────────────────────────────────────────────────────
rosparam load "$SRC/nidar_config/config/flight_envelope_guard.yaml" /
start guard "$SRC/nidar_safety/scripts/flight_envelope_guard.py"
wait_topic /mavros/setpoint_raw/local 20 "flight envelope guard setpoints"
start fuel roslaunch nidar_planner nidar_fuel_upstream.launch
sleep 5
start mission roslaunch nidar_hardware hw_mission.launch camera:="$CAM_ENABLED" \
    model_path:="$MODEL" imu_topic:="$IMU_TOPIC" \
    rotate_180:="$(hw "str(bool(c['camera'].get('rotate_180', False))).lower()")" \
    device:="$(hw "c['perception'].get('device', 'auto')")" \
    detect_hz:="$(hw "c['perception'].get('detect_hz', 3.0)")" \
    confidence_threshold:="$(hw "c['perception'].get('confidence_threshold', 0.55)")" \
    target_height_m:="$(hw "c['perception'].get('target_height_m', 0.35)")"
wait_topic /edm/mission_state 30 "mission manager"
wait_topic /nidar/onboard/status 20 "mission commander"

# ── 6. recording ──────────────────────────────────────────────────────────────────────────────
rosbag record --lz4 --split --size=500 -O "$LOG_DIR/mission" \
    /map_2d /grid_markers /survivor_tags /survivors /tf_static /edm/mission_state \
    /mavros/state /mavros/local_position/pose /mavros/vision_pose/pose /Fast_LIO/odometry \
    /tfmini/range /flight_envelope_guard/status /nidar/onboard/status /rosout_agg \
    > "$LOG_DIR/rosbag.log" 2>&1 &
BAG_PID=$!
PIDS+=("$BAG_PID")

# ── 7. ready ─────────────────────────────────────────────────────────────────────────────────
STATUS=$(timeout 5 rostopic echo -n 1 /nidar/onboard/status/data 2>/dev/null | head -1)
say "============================================================"
say "Onboard stack UP. Commander: $STATUS"
say "GCS: CONNECT DRONE (UDP 14550 over Wi-Fi, or the T12/SiK serial port), then TAKEOFF."
say "Ctrl-C stops the stack (on the ground only)."
say "============================================================"
wait "$DRIVERS_PID"
say "the driver launch exited (see $LOG_DIR/drivers.log)"
cleanup
