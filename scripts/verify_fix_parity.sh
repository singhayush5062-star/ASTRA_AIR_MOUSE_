#!/bin/bash
# Pre-flight check: are the fixes we think we are testing actually in the SOURCE and in the
# BINARY that roslaunch will run?
#
# This exists because on 2026-09-09 entry_detection_module.py and fast_exploration_fsm.cpp were
# both rewritten at 09:29:36 back to a state without the /mission/stop_exploration handshake.
# Three runs (093043, 094450, 101738) were then flown and analysed as if the fix were present.
# Run 101738 cost 136 s of its 164 s stall to the exact race the missing code prevents. Nothing
# in the launch, the build, or the flight log said the code was gone -- only the wording of one
# log line did. Run this before every test.
cd "$(dirname "$0")/.." || exit 1
WS=catkin_ws
NODE=$WS/devel/.private/exploration_manager/lib/exploration_manager/exploration_node
AP=$WS/devel/lib/libactive_perception.so
PM=$WS/devel/lib/libplan_manage.so
EDM=$WS/src/nidar_mission/scripts/mission_manager.py
FSM=$WS/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp
fail=0
chk() {  # chk <label> <needle> <file...>
    local label=$1 needle=$2; shift 2
    for f in "$@"; do
        if [ ! -e "$f" ]; then echo "MISSING FILE  $label: $f"; fail=1; continue; fi
        local n
        case "$f" in
            *.so|*/exploration_node) n=$(strings "$f" | grep -c -- "$needle") ;;
            *)                       n=$(grep -c -- "$needle" "$f") ;;
        esac
        if [ "$n" -lt 1 ]; then echo "ABSENT        $label  in  $f"; fail=1
        else                    echo "ok            $label  in  $f"; fi
    done
}
echo "--- launch XML well-formedness ---"
# A malformed <!-- --> block (e.g. a bare "--" inside a comment, which XML forbids) makes
# roslaunch refuse the WHOLE file with an RLException. exploration_node then never starts at
# all, but nothing else in the stack notices: PX4/gzserver/FAST-LIO are fine, the vehicle enters
# the arena and hovers, and the EDM just logs "FUEL has not subscribed" once a minute. From the
# outside that is indistinguishable from a stall -- run 20260909_120021 (2026-09-09) burned
# several minutes of wall clock looking like one before the actual cause (a "--" introduced by
# an edit to algorithm.xml) was found. Catch it before launch, not by staring at telemetry.
for xml in "$WS"/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml \
           "$WS"/src/fuel/fuel_planner/exploration_manager/launch/exploration.launch \
           "$WS"/src/nidar_mission/launch/nidar_mission.launch \
           "$WS"/src/nidar_perception/launch/detector.launch \
           "$WS"/src/nidar_bringup/launch/mission_only.launch \
           "$WS"/src/nidar_planner/launch/nidar_fuel_upstream.launch \
           "$WS"/src/nidar_slam/launch/nidar_mapping.launch; do
    # nidar_mission.launch went malformed twice in the 2026-09-11 session, both
    # times a bare "--" inside an XML comment which parses in vim but roslaunch
    # refuses with a top-level RLException that leaves the drone hovering while
    # FUEL sits in WAIT_TRIGGER (the mission_manager and EDM never launch, so
    # nothing publishes to /waypoint_generator/waypoints). See run
    # 20260911_094635.
    if [ -f "$xml" ]; then
        if python3 -c "import xml.dom.minidom as m; m.parse('$xml')" 2>/tmp/xmlcheck_err; then
            echo "ok            well-formed  $xml"
        else
            echo "MALFORMED     $xml"
            sed 's/^/              /' /tmp/xmlcheck_err
            fail=1
        fi
    fi
done

echo "--- source ---"
chk "stop-handshake publisher"  "pub_stop_exploration"                 "$EDM"
chk "stop-handshake helper"     "request_end_of_exploration"           "$EDM"
chk "stop-handshake subscriber" "stopExplorationCallback"              "$FSM"
echo "--- binaries ---"
chk "stop-handshake (FSM)"      "stop requested by mission layer"      "$NODE"
chk "trigger re-arm guard"      "ignoring trigger: exploration was stopped" "$NODE"
chk "degenerate-path retire"    "already standing on viewpoint"        "$NODE"
chk "viewpoint standoff"        "Standoff"                             "$AP"
chk "planExploreTraj guard"     "refusing a"                           "$PM"
chk "gain-weighted ATSP"        "FUEL GAIN"                            "$AP"
echo "--- staleness (binary older than source is a stale build) ---"
for pair in "$NODE:$FSM"; do
    b=${pair%%:*}; s=${pair##*:}
    if [ "$s" -nt "$b" ]; then echo "STALE         $b is older than $s -- rebuild"; fail=1
    else echo "ok            $b newer than $s"; fi
done

echo "--- phase 4: survivor detector chain ---"
DETECTOR=$WS/src/nidar_perception/scripts/survivor_detector.py
COVERAGE=$WS/src/nidar_mission/scripts/coverage_reporter.py
MAP2D=$WS/src/nidar_mission/scripts/map_2d_slicer.py
GRIDVIZ=$WS/src/nidar_mission/scripts/grid_visualizer.py
SURVIVOR_MSG_PY=$WS/devel/lib/python3/dist-packages/nidar_msgs/msg/_Survivor.py
MODEL_PT=$WS/src/nidar_perception/models/detection/PERSON_DETECTION_MODEL_V3/best.pt
# Model weights: an absent .pt turns the detector node into a FATAL, which
# under nidar_mission.launch (required=false by default for output=screen)
# would silently vanish and leave the run looking like a healthy exploration
# with zero detections -- the same failure mode the stop-handshake bug had.
if [ -f "$MODEL_PT" ]; then echo "ok            detector model  in  $MODEL_PT"
else echo "ABSENT        detector model at $MODEL_PT -- unzip PERSON_DETECTION_MODEL_V3"
     fail=1
fi
# Source: the strings we require the detector to log, so a silent rewrite that
# dropped e.g. the FAST-LIO smoke assertion or the survivor terminal line is
# caught before the flight, not by staring at logs after it.
chk "detector node file"        "class SurvivorDetector"           "$DETECTOR"
chk "detector: 5Hz timer"       "1.0 / self.detect_hz"             "$DETECTOR"
chk "detector: ground-plane BP" "_backproject_ground"              "$DETECTOR"
chk "detector: NN tracker"      "association_radius"               "$DETECTOR"
chk "detector: confirmation"    "confirmation_threshold"           "$DETECTOR"
chk "detector: [SURVIVOR] log"  "\[SURVIVOR\] id=%d"               "$DETECTOR"
chk "detector: FAST-LIO smoke"  "smoke assertion FAILED"           "$DETECTOR"
chk "coverage: 20% report"      "REACHED at t=%.1fs"               "$COVERAGE"
chk "coverage: 5 thresholds"    "20.0, 40.0, 60.0, 80.0, 95.0"     "$COVERAGE"

# Landing pad fix: DESCEND state between RETURN and LAND. A RETURN that
# overshoots or times out used to jump straight to AUTO.LAND, leaving PX4
# to blind-land mid-arena (see run 20260911_101520 t=527). DESCEND flies to
# (0, 0) camera_init first.
EDM_SRC=$WS/src/nidar_mission/scripts/mission_manager.py
chk "EDM: DESCEND state"        "DESCEND = \"DESCEND\""            "$EDM_SRC"
chk "EDM: run_descend"          "def run_descend"                   "$EDM_SRC"
chk "EDM: RETURN->DESCEND"      "MissionState.DESCEND"              "$EDM_SRC"

echo "--- phase 5: live 2D map + tagging ---"
# The OccupancyGrid publisher + grid overlay + survivor tag publisher.
# Absent = brief §5's "live 2D map generated during flight, legible as a
# floorplan, with survivors tagged on it" is not met.
chk "map_2d_slicer node file"   "class Map2DSlicer"                 "$MAP2D"
chk "map_2d: /map_2d publisher" "'/map_2d'"                         "$MAP2D"
chk "map_2d: OccupancyGrid"     "nav_msgs.msg import OccupancyGrid" "$MAP2D"
chk "grid_visualizer node file" "class GridVisualizer"              "$GRIDVIZ"
chk "grid_viz: /grid_markers"   "'/grid_markers'"                   "$GRIDVIZ"
chk "grid_viz: /survivor_tags"  "'/survivor_tags'"                  "$GRIDVIZ"
# Cell IDs use the phase plan A1..G7 convention. Missing means the grid
# overlay would render lines but no labels, and the GCS user cannot read
# off a cell to relay to rescue teams.
chk "grid_viz: cell label"      "def _cell_label"                   "$GRIDVIZ"
# Generated Python messages: catkin build outputs them under devel/lib/...;
# their absence means the message-generation stage never ran (bad CMake, or
# the workspace has not been rebuilt after adding msg files).
if [ -f "$SURVIVOR_MSG_PY" ]; then echo "ok            Survivor.msg python in $SURVIVOR_MSG_PY"
else echo "ABSENT        Survivor.msg python (rebuild nidar_msgs)"; fail=1
fi
# grid_visualizer.py swallows an ImportError on SurvivorArray and silently stops
# publishing /survivor_tags, so a stale import after the nidar_msgs split would
# look like "no survivors found" rather than a crash.
chk "detector: nidar_msgs import"  "from nidar_msgs.msg import"    "$DETECTOR"
chk "grid_viz: nidar_msgs import"  "from nidar_msgs.msg import"    "$GRIDVIZ"
# Ground truth: the world file has to actually contain the survivor blocks
# or the detector has nothing to detect. Accept either the earlier <actor>
# variant or the current static <model> variant; both mean "at least one
# survivor is spawned by the world at load".
WORLD=$WS/src/nidar_sim/worlds/nidar_competition.world
if [ -f "$WORLD" ]; then
    n_models=$(grep -c '<model name="survivor_' "$WORLD")
    n_actors=$(grep -c '<actor name="survivor_' "$WORLD")
    n=$((n_models + n_actors))
    if [ "$n" -ge 6 ]; then
        echo "ok            $n survivor block(s) in world (models=$n_models actors=$n_actors)"
    elif [ "$n" -ge 1 ]; then
        echo "WARN          only $n survivor block(s) in world (expected 6); re-run apply_mission_config.py"
    else
        echo "ABSENT        no survivor models in $WORLD -- rerun apply_mission_config.py"
        fail=1
    fi
fi
[ $fail -eq 0 ] && echo "PARITY OK" || echo "PARITY FAILED -- do not trust this run"
exit $fail
