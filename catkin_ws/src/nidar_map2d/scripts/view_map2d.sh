#!/bin/bash
# Open the 2D map / grid overlay / survivor tag view without a GCS.
#
# THREE MODES
# -----------
#   catkin_ws/src/nidar_map2d/scripts/view_map2d.sh                 # live: open RViz against the running sim
#   catkin_ws/src/nidar_map2d/scripts/view_map2d.sh --bag <path>    # offline: replay a recorded rosbag
#   catkin_ws/src/nidar_map2d/scripts/view_map2d.sh --check         # inspect what is publishing right now
#
# LIVE MODE
# ---------
# Requires the mission stack to be running (test_takeoff.sh brings roscore,
# FUEL, and the Phase 5 nodes up). The layout at nidar_map2d/rviz/nidar_map2d.rviz has
# fixed_frame=world and shows /map_2d, /grid_markers, /survivor_tags in a
# top-down orthographic camera.
#
# BAG MODE (post-run map inspection)
# ----------------------------------
# test_takeoff.sh auto-records logs/bags/map2d_<timestamp>.bag containing every
# Phase 5 topic. This mode:
#     1. starts a fresh roscore (or reuses one if already up)
#     2. plays back the bag with sim time enabled so RViz's TF buffer works
#     3. opens RViz with the same layout, paused at t=0
# So you can scrub through the exploration after the run has ended, without
# needing the Gazebo/PX4 stack. This answers the user's ask
# "how can we visualise the generated 2D map after simulation ends".
#
# CHECK MODE
# ----------
# Prints publish rates for the required topics and the world<->map TF, so
# operators can debug an empty RViz view without opening RViz first.

set -euo pipefail
# Lives in catkin_ws/src/nidar_map2d/scripts/; work from the repo root so --bag paths
# like logs/bags/... resolve the same way they always have.
cd "$(dirname "$0")/../../../.."

RVIZ_CFG="catkin_ws/src/nidar_map2d/rviz/nidar_map2d.rviz"

usage() {
    cat <<EOF
Usage:
  $0                          # live view (needs test_takeoff.sh running)
  $0 --bag <path/to/*.bag>    # replay a recorded bag from logs/bags/
  $0 --check                  # list required topic status
EOF
}

# --- CHECK MODE ---------------------------------------------------------
if [ "${1:-}" = "--check" ]; then
    echo "-- Required topic status (5 s sample) --"
    for topic in /map_2d /grid_markers /survivor_tags /tf_static; do
        if timeout 3 rostopic info "$topic" >/dev/null 2>&1; then
            hz=$(timeout 5 rostopic hz "$topic" 2>/dev/null \
                    | grep -oE 'average rate: [0-9.]+' | head -1)
            [ -z "$hz" ] && hz="(no messages)"
            printf "  ok    %-25s  %s\n" "$topic" "$hz"
        else
            printf "  MISS  %-25s  (topic not advertised)\n" "$topic"
        fi
    done
    echo
    echo "-- TF frames --"
    timeout 3 rosrun tf tf_echo world map 2>&1 | head -3 || true
    exit 0
fi

# --- BAG MODE -----------------------------------------------------------
if [ "${1:-}" = "--bag" ]; then
    BAG="${2:-}"
    if [ -z "$BAG" ]; then
        echo "error: --bag requires a path" >&2
        usage
        exit 2
    fi
    # Accept either a bag or its directory + prefix (test_takeoff.sh's rosbag
    # --split can produce foo.bag, foo_1.bag, foo_2.bag, ...)
    if [ ! -f "$BAG" ]; then
        # try globbing the prefix
        first=$(ls -1 "${BAG%.bag}"*.bag 2>/dev/null | head -1)
        if [ -n "$first" ]; then
            BAG="$first"
        else
            echo "error: bag not found: $BAG" >&2
            exit 3
        fi
    fi
    echo "Replaying $BAG"
    # Ensure a roscore. If none is running, start one in the background.
    if ! rostopic list >/dev/null 2>&1; then
        echo "Starting a local roscore..."
        roscore >/tmp/roscore_view.log 2>&1 &
        ROSCORE_PID=$!
        # Give it a couple of seconds to come up
        for _ in 1 2 3 4 5; do
            rostopic list >/dev/null 2>&1 && break
            sleep 1
        done
        trap "kill $ROSCORE_PID 2>/dev/null || true" EXIT
    fi
    # Enable sim time so RViz's TF buffer treats bag timestamps as authoritative.
    rosparam set /use_sim_time true >/dev/null 2>&1 || true

    # Publish the world <-> map static TFs OURSELVES rather than relying on the
    # bag's /tf_static replay. This is the "why is RViz empty?" fix from
    # 2026-09-11: rosbag replays /tf_static exactly once at its recorded
    # timestamp (bag t~0.5s), and RViz opened after that point (or in loop mode
    # between replays) never receives it. The chain must exist for the entire
    # RViz session or the OccupancyGrid display shows "Fixed Frame: No TF data"
    # and stays black. Numbers come from nidar_planner/launch/nidar_fuel_upstream.launch's
    # world_to_map_tf and map_to_camera_init_tf. Keep in sync if those change.
    rosrun tf2_ros static_transform_publisher 0 -9.5 0.26 1.5707963 0 0 world map \
        >/tmp/rviz_world_to_map_tf.log 2>&1 &
    TF1_PID=$!
    rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init \
        >/tmp/rviz_map_to_camera_init_tf.log 2>&1 &
    TF2_PID=$!
    # Extend the exit trap so these are cleaned up together with the roscore.
    trap "kill $TF1_PID $TF2_PID 2>/dev/null || true; kill ${ROSCORE_PID:-} 2>/dev/null || true" EXIT

    # Play the bag paused so the operator can scrub / step (space to play, s to
    # step, r to rewind). --clock so tf uses bag time.
    rosbag play --pause --clock --loop "$BAG" >/tmp/rosbag_play.log 2>&1 &
    BAG_PID=$!
    echo "rosbag play PID=$BAG_PID (paused; press space in the rosbag terminal or the RViz sim-time panel to advance)"
    # Give the bag a moment to advertise topics latched by rosbag
    sleep 1
    exec rviz -d "$RVIZ_CFG"
fi

if [ "${1:-}" != "" ]; then
    usage
    exit 2
fi

# --- LIVE MODE ----------------------------------------------------------
if [ ! -f "$RVIZ_CFG" ]; then
    echo "RViz config not found at $RVIZ_CFG" >&2
    exit 1
fi
if ! rostopic list >/dev/null 2>&1; then
    cat >&2 <<EOF
ROS master is not running. Either:
  * start the sim first (scripts/test_takeoff.sh) for a LIVE view, or
  * pass --bag <path>  to replay a recorded bag from logs/bags/.
EOF
    exit 1
fi
echo "Opening RViz with $RVIZ_CFG (fixed frame: world)"
echo "Topics required: /map_2d /grid_markers /survivor_tags"
exec rviz -d "$RVIZ_CFG"
