#!/bin/bash
# Component-placement rig: spawn each payload item as its OWN Gazebo model so the GUI's
# translate/rotate handles can move it.
#
# WHY THIS EXISTS
# Gazebo Classic's translate (T) and rotate (R) tools act on MODELS, never on individual links
# inside one. Everything in x500_vlp16.sdf is a link of a single model, so none of it can be
# dragged on its own, which is why the handles appear to do nothing useful. This rig republishes
# the frame and each payload item as separate static models. Then they ARE independently
# movable, and scripts/dump_gazebo_poses.py converts wherever you leave them back into
# base_link-relative <pose> values to paste into the real SDF.
#
# USAGE
#   Terminal 1:  scripts/pose_rig.sh
#   In the GUI:  click a component, press T to translate or R to rotate, drag the handles.
#                Hold Ctrl while dragging to disable snapping for fine placement.
#   Terminal 2:  python3 scripts/dump_gazebo_poses.py --rig --watch
#
# THE RIG IS NOT FLYABLE. Nothing here is jointed to the airframe and everything is static;
# it exists only to read out geometry. The flight vehicle stays x500_vlp16.sdf.
set -e
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/setup_env.sh"

MODELS="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
CFG="$ROOT_DIR/catkin_ws/src/nidar_config/config/mission_config.yaml"
SPAWN_X=$(python3 -c "import yaml;c=yaml.safe_load(open(\"$CFG\"))['nidar'];print(c['launch_pad']['center']['x'])" 2>/dev/null || echo 0.0)
SPAWN_Y=$(python3 -c "import yaml;c=yaml.safe_load(open(\"$CFG\"))['nidar'];print(c['launch_pad']['center']['y'])" 2>/dev/null || echo -9.5)
# Sit the frame at its natural resting height so the numbers you read match flight.
BASE_Z=0.2566

echo "Checking every SDF the rig touches..."
python3 - "$MODELS" "$ROOT_DIR" <<'EOF'
import sys, os, xml.etree.ElementTree as ET
models, root = sys.argv[1], sys.argv[2]
files = [os.path.join(models, n, n + '.sdf') for n in
         ('x500', 'x500_vlp16', 'rig_frame', 'rig_velodyne', 'rig_tfmini', 'rig_camera')]
files.append(os.path.join(root, 'catkin_ws', 'src', 'nidar_sim', 'models', 'tfmini_lidar', 'model.sdf'))
bad = False
for f in files:
    if not os.path.exists(f):
        print("  MISSING:", f); bad = True; continue
    try:
        ET.parse(f); print("  OK:", os.path.basename(f))
    except ET.ParseError as e:
        print("  FAIL:", os.path.basename(f), "-", e)
        print("        (if that line is in a comment, look for a stray '--')")
        bad = True
sys.exit(1 if bad else 0)
EOF

echo ""
echo "Starting Gazebo with the GUI..."
roslaunch gazebo_ros empty_world.launch \
    world_name:="$ROOT_DIR/catkin_ws/src/nidar_sim/worlds/nidar_competition.world" \
    gui:=true paused:=false use_sim_time:=true &
GZPID=$!

for i in $(seq 1 60); do
    rosservice list 2>/dev/null | grep -q "/gazebo/spawn_sdf_model" && break
    sleep 0.5
done

spawn () {  # name file x y z
    echo "  spawning $1"
    rosrun gazebo_ros spawn_model -sdf -file "$2" -model "$1" \
        -x "$3" -y "$4" -z "$5" >/dev/null 2>&1 || echo "    (spawn failed: $1)"
}

# The bare frame, pinned at its resting height. Every component pose is measured against this,
# so it must not settle or tip: un-pinned it tilted ~11 deg and corrupted every reading.
spawn rig_frame "$MODELS/rig_frame/rig_frame.sdf" "$SPAWN_X" "$SPAWN_Y" "$BASE_Z"
# Payload items at their CURRENT nominal offsets, so you start from the committed geometry.
spawn rig_velodyne "$MODELS/rig_velodyne/rig_velodyne.sdf" \
      "$SPAWN_X" "$SPAWN_Y" "$(python3 -c "print($BASE_Z + 0.20)")"
spawn rig_tfmini "$MODELS/rig_tfmini/rig_tfmini.sdf" \
      "$SPAWN_X" "$SPAWN_Y" "$(python3 -c "print($BASE_Z - 0.0305)")"
spawn rig_camera "$MODELS/rig_camera/rig_camera.sdf" \
      "$(python3 -c "print($SPAWN_X + 0.08)")" "$SPAWN_Y" \
      "$(python3 -c "print($BASE_Z - 0.0293)")"

cat <<'MSG'

================================================================================
RIG READY. Each component is now its own model, so the GUI handles work on it.

  click a component in the scene (or in the World tree on the left)
  press  T  translate handles      press  R  rotate handles
  hold Ctrl while dragging to turn OFF snapping for fine placement
  the left panel's "Pose" fields under each model can also be typed into directly

Read the result back, base_link-relative and SDF-ready:
    python3 scripts/dump_gazebo_poses.py --rig
    python3 scripts/dump_gazebo_poses.py --rig --watch     (live, as you drag)

Constraints the numbers must respect:
  velodyne  z > 0.138 above base_link, else the -15 deg ring is chopped by the prop tips
            (r = 0.270 m at z = 0.0625, after the 2026-09-06 airframe shrink)
  tfmini    parked range must stay above its 0.10 m floor; the skids rest at z = -0.2195,
            so keep the lens well above roughly -0.12
  camera    mounting plate is the mesh -Z face, so it needs roll = pi to sit plate-up

Ctrl+C here to tear the rig down.
================================================================================
MSG
wait $GZPID
