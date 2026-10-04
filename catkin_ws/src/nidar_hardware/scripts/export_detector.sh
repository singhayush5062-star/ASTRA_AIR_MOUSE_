#!/bin/bash
# Export a survivor-detector checkpoint to NCNN, the format that runs fast on the Jetson's CPU.
# Run it in the DEV container on the laptop/PC (x86, has ultralytics + internet for the pnnx
# converter), commit or copy the result, then point hardware.yaml perception.model at it.
#
#   catkin_ws/src/nidar_hardware/scripts/export_detector.sh                       # YOLO26s, 640
#   catkin_ws/src/nidar_hardware/scripts/export_detector.sh path/to/best.pt 480   # other model/size
#
# A smaller imgsz is faster and sees less: 640 is the trained size; try 480 if detect_hz cannot
# be held (the detector logs its latency; the GCS camera HUD shows it too).
set -e
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
MODEL=${1:-$ROOT_DIR/catkin_ws/src/nidar_perception/models/detection/YOLO26S_DRONE_PERSON_V1/best.pt}
IMGSZ=${2:-640}
python3 - "$MODEL" "$IMGSZ" <<'PY'
import sys
from ultralytics import YOLO
out = YOLO(sys.argv[1]).export(format="ncnn", imgsz=int(sys.argv[2]))
print("exported:", out)
PY
echo "Set catkin_ws/src/nidar_config/config/hardware.yaml perception.model to the *_ncnn_model directory above."
