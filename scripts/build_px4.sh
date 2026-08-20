#!/bin/bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Export Gazebo Model & Plugin paths
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$ROOT_DIR/simulation/custom_models:$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:+$GAZEBO_PLUGIN_PATH:}$ROOT_DIR/catkin_ws/devel/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ROOT_DIR/catkin_ws/devel/lib"

MODEL="${1:-gazebo-classic_iris_vlp16}"

PX4_DIR="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3"

# PX4's own build system hard-requires a .git directory at its root (Makefile aborts without one),
# and its version-header generator additionally shells out to `git rev-parse HEAD` inside a couple of
# nested submodule-like paths it checks (e.g. src/modules/mavlink/mavlink) if *anything* answering to
# `.git` is present there. This vendored copy of PX4-Autopilot has no real upstream git history of its
# own (it's committed as plain files in the NIDAR repo, not wired up as a proper git submodule), so a
# fresh clone has neither -- bootstrap minimal, self-contained git repos in exactly the places PX4's
# build tooling checks, so the build works without needing real PX4 git history.
ensure_git_bootstrap() {
    local dir="$1" tag="$2"
    if [ ! -e "$dir/.git" ]; then
        echo "Notice: $dir/.git is missing -- bootstrapping a minimal local git repo so PX4's build system's git checks pass..."
        (cd "$dir" && git init -q && git config user.email "build@nidar.local" \
            && git config user.name "NIDAR Build Bootstrap" && git commit -q -m "bootstrap" --allow-empty)
        if [ -n "$tag" ]; then
            (cd "$dir" && git tag "$tag" 2>/dev/null || true)
        fi
    fi
}
ensure_git_bootstrap "$PX4_DIR" "v1.14.3"
ensure_git_bootstrap "$PX4_DIR/src/modules/mavlink/mavlink" ""
ensure_git_bootstrap "$PX4_DIR/src/drivers/gps/devices" ""
ensure_git_bootstrap "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic" ""
ensure_git_bootstrap "$PX4_DIR/src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client" ""
ensure_git_bootstrap "$PX4_DIR/src/lib/events/libevents" ""

cd "$PX4_DIR"

echo "Building PX4 SITL for target: $MODEL..."

DONT_RUN=1 make px4_sitl "$MODEL"

echo "PX4 build completed successfully for $MODEL."
