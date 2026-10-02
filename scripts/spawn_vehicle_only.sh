#!/bin/bash
# Spawn ONLY the vehicle model in Gazebo -- no PX4 SITL, no MAVROS, no FAST-LIO, no FUEL, no
# mission stack. For iterating on camera/TFmini/landing-gear SDF poses: edit the SDF, re-run
# this, look/measure, repeat. Comes up in ~5-10s instead of test_takeoff.sh's ~60-90s, because
# it skips PX4 boot, EKF2 convergence and every ROS node that only matters in flight.
#
# The model's mavlink_interface plugin will print connection-retry warnings with no PX4 to talk
# to -- harmless, ignore them; it does not affect spawning, rendering, or the sensor topics.
#
# Usage:
#   scripts/spawn_vehicle_only.sh [z] [gui]
#     z    spawn height, world frame (default: high enough to see the underside clearly)
#     gui  true|false (default true) -- false for a headless check of just the ROS topics
#
# While it's running, in another terminal:
#   rostopic echo /tfmini/range                    # live range reading
#   rosrun tf tf_echo world tfmini_lidar::link      # exact world pose of the sensor
#   rosrun tf tf_echo world camera_link             # exact world pose of the camera
#   rostopic hz /camera/image_raw                   # confirm the camera is actually publishing
# In the Gazebo GUI: View > Wireframe, then zoom into the belly to check the mounts by eye.
set -e
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/setup_env.sh"

# Default to the REAL pad pose from mission_config, not the world origin: the origin sits
# inside the arena mesh, so a vehicle spawned there rests on arena geometry and can settle
# tilted, which looks exactly like a model-stability bug but is not one.
CFG="$ROOT_DIR/catkin_ws/src/nidar_config/config/mission_config.yaml"
SPAWN_X=$(python3 -c "import yaml;c=yaml.safe_load(open(\"$CFG\"))['nidar'];print(c['launch_pad']['center']['x'])" 2>/dev/null || echo 0.0)
SPAWN_Y=$(python3 -c "import yaml;c=yaml.safe_load(open(\"$CFG\"))['nidar'];print(c['launch_pad']['center']['y'])" 2>/dev/null || echo -9.5)
ZDEF=$(python3 -c "import yaml;c=yaml.safe_load(open(\"$CFG\"))['nidar'];print(round(c['launch_pad']['thickness']+c['vehicle']['belly_clearance'],3))" 2>/dev/null || echo 0.25)
Z="${1:-$ZDEF}"
GUI="${2:-true}"
# Which vehicle model to spawn. Defaults to the X500 build (frame + VLP-16); pass
# iris_vlp16_cam to compare against the old airframe.
MODEL="${3:-x500_vlp16}"

echo "Checking SDF well-formedness first (catches the '--in-a-comment' mistake before a 10s wait)..."
python3 - "$ROOT_DIR" "$MODEL" <<'EOF'
import sys, os, xml.etree.ElementTree as ET
root = sys.argv[1]
sdf = os.path.join(root, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
                    'gazebo-classic', 'sitl_gazebo-classic', 'models', sys.argv[2],
                    sys.argv[2] + '.sdf')
try:
    ET.parse(sdf)
    print("  OK:", os.path.relpath(sdf, root))
except ET.ParseError as e:
    print("  FAIL:", os.path.relpath(sdf, root), "-", e)
    print("  (if the line is inside a <!-- comment -->, check for a stray '--')")
    sys.exit(1)
EOF

echo "Running scripts/check_mount_geometry.py..."
python3 "$ROOT_DIR/scripts/check_mount_geometry.py"

echo ""
echo "Starting bare Gazebo (world: nidar_competition.world, for the pad as a size reference)..."
roslaunch gazebo_ros empty_world.launch \
    world_name:="$ROOT_DIR/catkin_ws/src/nidar_sim/worlds/nidar_competition.world" \
    gui:="$GUI" paused:=false use_sim_time:=true &
GZPID=$!

# Wait for gzserver's spawn service rather than a fixed sleep.
for i in $(seq 1 40); do
    rosservice list 2>/dev/null | grep -q "/gazebo/spawn_sdf_model" && break
    sleep 0.5
done

echo "Spawning $MODEL at (${SPAWN_X}, ${SPAWN_Y}, ${Z}) - the real pad pose..."
rosrun gazebo_ros spawn_model -sdf \
    -file "$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/$MODEL/$MODEL.sdf" \
    -model "$MODEL" -x "$SPAWN_X" -y "$SPAWN_Y" -z "$Z"

echo ""
echo "Spawned. Vehicle is NOT armed and has no flight controller -- it will just sit at z=${Z}"
echo "(or fall/settle if under gravity near the ground) so you can inspect mounts freely."
echo ""
echo "Useful commands in another terminal (after: source scripts/setup_env.sh):"
echo "  rostopic echo /tfmini/range"
echo "  rosrun tf tf_echo world tfmini_lidar::link"
echo "  rosrun tf tf_echo world camera_link"
echo ""
echo "Ctrl+C here to tear down Gazebo."
wait $GZPID
