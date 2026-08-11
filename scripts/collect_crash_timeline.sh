#!/bin/bash
set -euo pipefail

OUT_DIR=${1:-/tmp/crash_timeline}
mkdir -p "$OUT_DIR"

source /home/developer/NIDAR/scripts/setup_env.sh

echo "[crash_timeline] writing logs to $OUT_DIR"

# Sample critical state for 180s at 5Hz (0.2s)
python3 - <<'PY' "$OUT_DIR/state_timeline.csv" &
import csv
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

out_path = __import__('sys').argv[1]
rospy.init_node('crash_timeline_state_sampler', anonymous=True)

state_msg = None
pose_msg = None


def state_cb(msg):
    global state_msg
    state_msg = msg


def pose_cb(msg):
    global pose_msg
    pose_msg = msg


rospy.Subscriber('/mavros/state', State, state_cb, queue_size=50)
rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb, queue_size=50)

rate = rospy.Rate(5)
start = rospy.Time.now().to_sec()

with open(out_path, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['ros_time','armed','mode','connected','x','y','z'])
    while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start) < 180.0:
        now = rospy.Time.now().to_sec()
        armed = state_msg.armed if state_msg else ''
        mode = state_msg.mode if state_msg else ''
        connected = state_msg.connected if state_msg else ''
        x = pose_msg.pose.position.x if pose_msg else ''
        y = pose_msg.pose.position.y if pose_msg else ''
        z = pose_msg.pose.position.z if pose_msg else ''
        w.writerow([now, armed, mode, connected, x, y, z])
        rate.sleep()
PY
STATE_PID=$!

# Capture planner and bridge command streams.
(timeout 185 rostopic echo /planning/pos_cmd > "$OUT_DIR/pos_cmd.log" 2>&1 || true) &
POS_PID=$!
(timeout 185 rostopic echo /planning/bspline > "$OUT_DIR/bspline.log" 2>&1 || true) &
BSPLINE_PID=$!
(timeout 185 rostopic echo /flight_path > "$OUT_DIR/flight_path.log" 2>&1 || true) &
PATH_PID=$!

# Capture relevant process logs snapshot at start and end windows.
(cp /tmp/bridge.log "$OUT_DIR/bridge_start.log" 2>/dev/null || true)
(cp /tmp/fuel.log "$OUT_DIR/fuel_start.log" 2>/dev/null || true)
(cp /tmp/mission_manager.log "$OUT_DIR/mission_start.log" 2>/dev/null || true)
(cp /tmp/sim_test.log "$OUT_DIR/sim_start.log" 2>/dev/null || true)

wait $STATE_PID || true
wait $POS_PID || true
wait $BSPLINE_PID || true
wait $PATH_PID || true

(cp /tmp/bridge.log "$OUT_DIR/bridge_end.log" 2>/dev/null || true)
(cp /tmp/fuel.log "$OUT_DIR/fuel_end.log" 2>/dev/null || true)
(cp /tmp/mission_manager.log "$OUT_DIR/mission_end.log" 2>/dev/null || true)
(cp /tmp/sim_test.log "$OUT_DIR/sim_end.log" 2>/dev/null || true)

echo "[crash_timeline] done. artifacts in $OUT_DIR"
