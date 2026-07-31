#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3"

echo "Building PX4 SITL..."

DONT_RUN=1 make px4_sitl gazebo-classic

echo "PX4 build completed."
