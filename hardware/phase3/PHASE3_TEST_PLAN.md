# Phase 3: Perception, Mapping & GCS — Hardware Test Plan

**Scope:** Verify camera stream and YOLO survivor detection, 3D survivor localization via point cloud raycast, discrete grid tagging, 2D occupancy grid mapping, and Ground Control Station (GCS) telemetry on the physical vehicle.

---

## The Core Rule

> **Phase 2 MUST be fully PASSED before starting Phase 3.**  
> FAST-LIO SLAM, odometry relay, and position control must be verified working on real hardware before adding perception and mapping layers.

---

## Code Import & Reuse Strategy

All Phase 3 verification scripts in `hardware/phase3/scripts/` import directly from existing workspace code — zero new verification logic:

| Import Source | Functions / Modules Reused | Used In Test |
|---|---|---|
| [`scripts/verify_components.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_components.py) | `camera_aiming_vector()`, `lidar_pointcloud_validation()`, sensor validation math | 3.1 Camera, 3.2 3D Localization |
| [`scripts/check_mount_geometry.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/check_mount_geometry.py) | `rotated_camera_z_extent()`, camera-to-world transform, standoff geometry | 3.1 Camera Pose |
| [`scripts/verify_flight.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_flight.py) | `load_walls()`, `clearance_fn()` — wall cross-section at cruise height | 3.4 Occupancy Grid |
| [`scripts/verify_full_flight.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_full_flight.py) | `Verifier` class — full-flight verification stages | 3.4, 3.5 End-to-End |
| [`scripts/analyze_exploration.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/analyze_exploration.py) | `Analyzer` class — coverage tracking, `cov_hist` time-series | 3.4 Mapping Verification |
| [`scripts/strict_monitor.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/strict_monitor.py) | State/pose CSV logging, bandwidth/rate monitoring | 3.5 GCS Telemetry |
| [`scripts/mission_telemetry_logger.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/mission_telemetry_logger.py) | Mission telemetry recording — state, pose, completion tracking | 3.5 GCS Telemetry |
| [`catkin_ws/src/nidar_mission/scripts/entry_detection_module.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/catkin_ws/src/nidar_mission/scripts/entry_detection_module.py) | `MissionState`, `MultiCueEntryDetector` — state machine context | 3.5 Mission Context |
| [`catkin_ws/src/nidar_mission/config/mission_config.yaml`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/catkin_ws/src/nidar_mission/config/mission_config.yaml) | Arena dimensions, grid cell sizes, sensor parameters | 3.3 Grid Tagging, 3.4 Map |

## Persistent Execution & Implementation Logging

All execution runs and gating decisions are recorded in the common reports directory:
- [`hardware/reports/HARDWARE_STATUS_REPORT.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_STATUS_REPORT.md)
- [`hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md)

---

## 1. Camera & Survivor Detection — YOLO (Item 3.1)

### Objective
Verify the onboard camera publishes a usable video stream and that the YOLO detector runs at sufficient rate without starving SLAM of CPU/GPU resources.

### Test Procedure

```bash
# 1. Launch camera driver (adjust for your specific camera hardware)
roslaunch <camera_driver_pkg> camera.launch

# 2. Check frame rate (must be ≥ 15 FPS)
rostopic hz /camera/image_raw

# 3. Launch YOLO detector node
roslaunch nidar_mission yolo_detector.launch

# 4. Check detector rate (must be ≥ 5 Hz)
rostopic hz /detections

# 5. While YOLO is running, verify FAST-LIO is NOT degraded
rostopic hz /Odometry  # Must still be ≥ 9 Hz
```

### Imported Verification
```python
# Import camera aiming vector validation from verify_components
import sys; sys.path.insert(0, 'scripts/')
from verify_components import camera_aiming_vector

# Validate camera mount orientation matches expected FOV
from check_mount_geometry import rotated_camera_z_extent
```

### False Positive Test
```bash
# Point camera at plain walls, floor, and background obstacles
# Record detections for 60 seconds
# Count false positive detections — must be 0
rostopic echo /detections --filter "len(m.bounding_boxes) > 0"
```

### Pass Criteria
- Camera stream publishing at ≥ 15 FPS
- YOLO detector runs at ≥ 5 Hz onboard
- FAST-LIO localization rate remains ≥ 9 Hz (no CPU starvation)
- Zero false positives on plain walls and background obstacles

---

## 2. 3D Survivor Localization (Item 3.2)

### Objective
Verify that bounding box centroids are correctly raycasted against the registered point cloud to produce 3D world positions within 0.30 m of ground truth.

### Test Procedure

```bash
# 1. Place known test targets (mannequins / heat signatures) at surveyed positions
#    Ground truth positions measured with tape / laser to ± 0.01 m

# 2. Launch full perception stack (FAST-LIO + Camera + YOLO + 3D Localizer)

# 3. Record detected 3D positions
rostopic echo /survivor_positions

# 4. Compare detected positions against surveyed ground truth
#    Compute Euclidean distance error for each detection
```

### Imported Verification
```python
# Import from verify_components for point cloud validation
import sys; sys.path.insert(0, 'scripts/')
from verify_components import lidar_pointcloud_validation

# Validate registered point cloud quality before raycast
# Reuse camera_aiming_vector() to verify camera-to-lidar alignment
from verify_components import camera_aiming_vector
```

### Pass Criteria
- Bounding box centroid correctly raycasts against registered point cloud
- 3D position error ≤ 0.30 m against surveyed ground-truth targets
- All test targets detected (no false negatives on known targets)

---

## 3. Discrete Grid Tagging (Item 3.3)

### Objective
Verify that 3D world coordinates are correctly mapped to the Discrete Arena Grid format (e.g., A1 – N14), including cell center and boundary edge cases.

### Test Procedure

```bash
# 1. Load arena grid configuration from mission_config.yaml
rosparam load $(rospack find nidar_mission)/config/mission_config.yaml

# 2. Test known coordinate → grid cell mappings
#    Place targets at:
#      - Cell center of A1 → must tag as A1
#      - Cell center of N14 → must tag as N14
#      - Cell boundary between B2 and B3 → must tag consistently
#      - Corner of the grid (edge case)

# 3. Publish test survivor positions and verify grid tags
rostopic echo /survivor_grid_tags
```

### Imported Verification
```python
# Import arena configuration for grid dimension validation
import yaml
import sys; sys.path.insert(0, 'catkin_ws/src/nidar_mission/scripts/')
from apply_mission_config import REPO, CONFIG
with open(CONFIG) as f:
    cfg = yaml.safe_load(f)
# Validate arena bounds: x_min, x_max, y_min, y_max match grid layout
arena = cfg['nidar']['arenas'][cfg['nidar']['arena']['active']]
```

### Pass Criteria
- 3D coordinates map correctly to Discrete Arena Grid format (A1 – N14)
- Cell center targets tagged correctly
- Cell boundary targets tagged consistently (no ambiguous results)
- Corner / edge grid cells handled without IndexError

---

## 4. 2D Occupancy Grid Mapping (Item 3.4)

### Objective
Verify that the 3D point cloud is correctly sliced, published as `nav_msgs/OccupancyGrid`, and that corridors, rooms, and obstacles are clearly distinguished.

### Test Procedure

```bash
# 1. Launch full SLAM + mapping stack
roslaunch fast_lio mapping_velodyne.launch

# 2. Verify point cloud Z-slicing (0.3 m ≤ Z ≤ 1.9 m)
#    Only points within this band should appear in the 2D grid

# 3. Check /map_2d publication rate (must be ≥ 2 Hz)
rostopic hz /map_2d

# 4. Verify message type
rostopic info /map_2d  # Must be nav_msgs/OccupancyGrid

# 5. Visual verification in RViz
#    - Open RViz with map display
#    - Walk/carry drone through known corridors and rooms
#    - Verify corridors, rooms, and obstacles appear correctly
```

### Imported Verification
```python
# Import wall geometry for map accuracy comparison
import sys; sys.path.insert(0, 'scripts/')
from verify_flight import load_walls

# Load known wall segments at cruise height for comparison against occupancy grid
wall_A, wall_B = load_walls(z=1.75)

# Import coverage analyzer to track mapping progress
from analyze_exploration import Analyzer
# Analyzer.cov_hist tracks (time, free_area, percentage) over the run
```

### Pass Criteria
- 3D point cloud correctly sliced between 0.3 m ≤ Z ≤ 1.9 m
- `/map_2d` publishes as `nav_msgs/OccupancyGrid` at ≥ 2 Hz
- Corridors, rooms, and obstacles clearly distinguished in RViz
- Map accuracy: ≥ 90% of known wall segments appear as occupied cells

---

## 5. Ground Control Station (Item 3.5)

### Objective
Verify the GCS displays live video, 2D map, survivor tags, drone pose, and mission clock, all matching onboard telemetry within bandwidth constraints.

### Test Procedure

```bash
# 1. Launch GCS on the operator laptop
# (connected via WiFi to the drone's companion computer)

# 2. Launch mission telemetry logger to record onboard values
python3 scripts/mission_telemetry_logger.py

# 3. Verify each GCS display element:
#    a) Live video stream from camera
#    b) 2D occupancy grid map
#    c) Survivor detection tags with grid cell labels
#    d) Drone pose (position + heading)
#    e) Mission clock / elapsed time

# 4. Cross-reference GCS display against onboard topics:
rostopic echo /mavros/local_position/pose   # Compare with GCS pose display
rostopic echo /survivor_positions            # Compare with GCS survivor markers
rostopic echo /map_2d                        # Compare with GCS map display

# 5. Measure wireless bandwidth
#    Monitor network interface throughput during full operation
iftop -i wlan0  # or equivalent network monitoring tool
```

### Imported Verification
```python
# Import telemetry logger for recording onboard ground truth
import sys; sys.path.insert(0, 'scripts/')
from mission_telemetry_logger import *  # Reuse existing telemetry recording

# Import strict_monitor for state/pose CSV logging (used for comparison)
from strict_monitor import state_cb, pose_cb
```

### Pass Criteria
- GCS displays live video, 2D map, survivor tags, drone pose, and mission clock
- Displayed values match onboard telemetry (position within 0.1 m, no stale data > 1 s)
- Total wireless bandwidth ≤ 4 Mbps
- GCS update rate ≥ 2 Hz for map and pose displays
- No dropped frames in video stream for > 2 consecutive seconds

---

## Phase 3 Sign-Off Sheet

| # | Item | Status | Verified By | Notes |
|---|---|:---:|---|---|
| 1 | Camera & Survivor Detection (YOLO) | [ ] PASS | | |
| 2 | 3D Survivor Localization | [ ] PASS | | |
| 3 | Discrete Grid Tagging | [ ] PASS | | |
| 4 | 2D Occupancy Grid Mapping | [ ] PASS | | |
| 5 | Ground Control Station (GCS) | [ ] PASS | | |

> **GATING DECISION:** If all 5 items are marked **PASS**, Phase 3 is officially complete. You may now proceed to **Phase 4 (Full Mission & Failsafes)**.
