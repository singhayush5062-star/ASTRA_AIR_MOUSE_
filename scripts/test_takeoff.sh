#!/bin/bash
# Kept at its historical path so `./scripts/test_takeoff.sh [GUI] [X Y Z YAW]` (and every doc that
# quotes it) keeps working after the package split. The orchestrator itself lives in the
# nidar_bringup package; this just hands over to it with the same arguments and environment.
exec "$(dirname "${BASH_SOURCE[0]}")/../catkin_ws/src/nidar_bringup/scripts/test_takeoff.sh" "$@"
