#!/bin/bash
set -euo pipefail

OUT_DIR=${1:-/tmp/phase0_audit}
DURATION=${2:-180}
mkdir -p "$OUT_DIR"

source /home/developer/NIDAR/scripts/setup_env.sh

echo "[phase0] collecting mapping/TF/planner diagnostics for ${DURATION}s into $OUT_DIR"

date > "$OUT_DIR/started_at.txt"
rosparam list > "$OUT_DIR/rosparam_list.txt" 2>&1 || true

# Topic metadata snapshot
for topic in /Fast_LIO/odometry /cloud_registered /sdf_map/occupancy_all /sdf_map/esdf /planning/bspline /planning/pos_cmd /mavros/local_position/pose /tf /tf_static; do
  {
    echo "=== $topic ==="
    rostopic type "$topic" || true
    rostopic info "$topic" || true
    echo
  } >> "$OUT_DIR/topic_info.txt" 2>&1
done

# Frequency probes (run concurrently)
HZ_PIDS=()
for topic in /Fast_LIO/odometry /cloud_registered /sdf_map/occupancy_all /sdf_map/esdf /planning/pos_cmd /mavros/local_position/pose; do
  (timeout "$DURATION" rostopic hz "$topic" > "$OUT_DIR/hz_${topic//\//_}.log" 2>&1 || true) &
  HZ_PIDS+=("$!")
done

# Capture one bounded sample per high-bandwidth topic.  Continuous PointCloud2
# YAML dumps create multi-gigabyte logs and make an otherwise healthy ESDF look
# absent when the collector is killed or starved.  The concurrent `rostopic hz`
# probes above are the authoritative liveness measurement.
(timeout 10 rostopic echo -n 1 /cloud_registered > "$OUT_DIR/cloud_registered_sample.log" 2>&1 || true) &
P1=$!
(timeout 10 rostopic echo -n 1 /sdf_map/occupancy_all > "$OUT_DIR/occupancy_all_sample.log" 2>&1 || true) &
P2=$!
(timeout 10 rostopic echo -n 1 /sdf_map/esdf > "$OUT_DIR/esdf_sample.log" 2>&1 || true) &
P3=$!
(timeout "$DURATION" rostopic echo /planning/bspline > "$OUT_DIR/bspline.log" 2>&1 || true) &
P4=$!
(timeout "$DURATION" rostopic echo /planning/pos_cmd > "$OUT_DIR/pos_cmd.log" 2>&1 || true) &
P5=$!
(timeout "$DURATION" rostopic echo /mavros/local_position/pose > "$OUT_DIR/local_pose.log" 2>&1 || true) &
P6=$!
(timeout "$DURATION" rostopic echo /Fast_LIO/odometry > "$OUT_DIR/fastlio_odom.log" 2>&1 || true) &
P7=$!
(timeout "$DURATION" rostopic echo /tf > "$OUT_DIR/tf.log" 2>&1 || true) &
P8=$!
(timeout "$DURATION" rostopic echo /tf_static > "$OUT_DIR/tf_static.log" 2>&1 || true) &
P9=$!

# TF health audit
python3 /home/developer/NIDAR/scripts/phase0_tf_audit.py _out:="$OUT_DIR/tf_audit.log" _duration:="$DURATION" _period:=0.5 > "$OUT_DIR/tf_audit_stdout.log" 2>&1 || true

wait $P1 || true
wait $P2 || true
wait $P3 || true
wait $P4 || true
wait $P5 || true
wait $P6 || true
wait $P7 || true
wait $P8 || true
wait $P9 || true
for pid in "${HZ_PIDS[@]}"; do
  wait "$pid" || true
done

# Simple timeline bundle
cp /tmp/bridge.log "$OUT_DIR/bridge.log" 2>/dev/null || true
cp /tmp/fuel.log "$OUT_DIR/fuel.log" 2>/dev/null || true
cp /tmp/mission_manager.log "$OUT_DIR/mission_manager.log" 2>/dev/null || true
cp /tmp/sim_test.log "$OUT_DIR/sim_test.log" 2>/dev/null || true
cp /tmp/relay.log "$OUT_DIR/relay.log" 2>/dev/null || true

echo "[phase0] complete: $OUT_DIR"
