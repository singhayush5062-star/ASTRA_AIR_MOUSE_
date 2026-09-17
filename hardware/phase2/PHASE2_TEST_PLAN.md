# Phase 2: Autonomy & Flight Control — Hardware Test Plan

**Scope:** Verify FAST-LIO2 SLAM localization, odometry relay to PX4 EKF, flight envelope guard, position setpoint control, and progressive FUEL exploration on the physical vehicle.

---

## The Core Rule

> **Phase 1 MUST be fully PASSED before starting Phase 2.**  
> First flight tests MUST be performed in a clear, open area with safety nets or tethered flight.

---

## Code Import & Reuse Strategy

All Phase 2 verification scripts in `hardware/phase2/scripts/` import directly from existing workspace code — zero new verification logic:

| Import Source | Functions / Modules Reused | Used In Test |
|---|---|---|
| [`scripts/relay_odometry.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/relay_odometry.py) | `odometry_callback()`, origin anchoring, jump rejection, healthy streak tracking | 2.2 Odometry Relay |
| [`scripts/flight_envelope_guard.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/flight_envelope_guard.py) | `camera_to_world()`, `world_to_camera()`, boundary clamping, HOLD state streaming | 2.3 Flight Envelope Guard |
| [`scripts/verify_flight.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_flight.py) | `load_walls()`, `clearance_fn()`, `collision_radius()` | 2.4 Position Control |
| [`scripts/verify_full_flight.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_full_flight.py) | `Verifier` class — placement, takeoff, altitude, entry, explore stage checks | 2.4, 2.5 Full Flight |
| [`scripts/analyze_exploration.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/analyze_exploration.py) | `Analyzer` class — coverage tracking, revisit scoring, repeat target detection | 2.5 FUEL Exploration |
| [`scripts/strict_monitor.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/strict_monitor.py) | State/pose/completion CSV logging, setpoint publisher audit | 2.4, 2.5 Monitoring |
| [`catkin_ws/src/nidar_mission/scripts/entry_detection_module.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/catkin_ws/src/nidar_mission/scripts/entry_detection_module.py) | `MultiCueEntryDetector`, `MissionState` state machine | 2.3, 2.5 Entry/Mission |
| [`config/flight_envelope_guard.yaml`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/config/flight_envelope_guard.yaml) | World frame boundaries, spawn pose, margin parameters | 2.3 Config Verification |

## Persistent Execution & Implementation Logging

All execution runs and gating decisions are recorded in the common reports directory:
- [`hardware/reports/HARDWARE_STATUS_REPORT.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_STATUS_REPORT.md)
- [`hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md)

---

## 1. FAST-LIO2 SLAM Localization (Item 2.1)

### Objective
Verify that FAST-LIO2 produces stable, accurate localization on real hardware at ≥ 9 Hz without NaN/Inf or coordinate jumps.

### Test Procedure

```bash
# 1. Launch FAST-LIO with hardware LiDAR
roslaunch fast_lio mapping_velodyne.launch

# 2. Monitor output rate (must be ≥ 9 Hz)
rostopic hz /Odometry

# 3. Check for NaN/Inf values in pose
rostopic echo /Odometry | grep -i "nan\|inf"

# 4. Displacement test: carry drone 1.0 m forward on a measured rail/tape
#    Record before and after position from /Odometry
rostopic echo -n 1 /Odometry  # BEFORE
# <physically move drone 1.0 m forward>
rostopic echo -n 1 /Odometry  # AFTER
```

### Imported Verification
```python
# Import from existing scripts — no new math
from scripts.verify_full_flight import Verifier
# Verifier.check_placement() and Verifier's odometry tracking validates FAST-LIO pose
```

### Pass Criteria
- Output rate sustained ≥ 9 Hz on companion CPU
- Zero NaN/Inf values in position or orientation
- Displacement test: 1.0 m forward → FAST-LIO estimates 1.0 m ± 0.05 m

---

## 2. Odometry Relay & PX4 EKF (Item 2.2)

### Objective
Verify `relay_odometry.py` correctly forwards FAST-LIO odometry to MAVROS vision pose, and PX4 EKF fuses it without divergence.

### Test Procedure

```bash
# 1. Launch MAVROS + FAST-LIO + relay
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600"
roslaunch fast_lio mapping_velodyne.launch
rosrun scripts relay_odometry.py

# 2. Compare three pose streams simultaneously
rostopic echo /Odometry                      # FAST-LIO raw
rostopic echo /mavros/vision_pose/pose        # Relay output
rostopic echo /mavros/local_position/pose     # PX4 EKF fused

# 3. Check EKF innovation residuals
rostopic echo /mavros/estimator_status
```

### Imported Verification
```python
# Import relay_odometry.py to verify its jump rejection and origin anchoring
import sys; sys.path.insert(0, 'scripts/')
from relay_odometry import odometry_callback, state_callback
# Verify healthy_streak counter increments and rejected_count stays at 0
```

### Pass Criteria
- FAST-LIO pose ≈ Relay pose ≈ `/mavros/local_position/pose` (within 0.05 m)
- EKF innovation residuals remain within safe bounds (no EKF reset warnings)
- `rejected_count` = 0 during normal operation
- `healthy_streak` continuously increasing

---

## 3. Flight Envelope Guard (Item 2.3)

### Objective
Verify the flight envelope guard correctly accepts in-bounds commands, rejects out-of-bounds commands, and triggers fault states.

### Test Procedure

```bash
# 1. Launch the guard node with config
rosrun scripts flight_envelope_guard.py

# 2. Send a valid in-envelope setpoint
rostopic pub /planning/pos_cmd quadrotor_msgs/PositionCommand \
  "{position: {x: 0.0, y: 0.0, z: 1.5}}"
# Expected: ACCEPT → forwarded to /mavros/setpoint_raw/local

# 3. Send an out-of-bounds command
rostopic pub /planning/pos_cmd quadrotor_msgs/PositionCommand \
  "{position: {x: 8.0, y: 0.0, z: 1.5}}"
# Expected: REJECT → blocked, rejection reason published

# 4. Verify diagnostic output
rostopic echo /guard/rejection_reason
rostopic echo /guard/state
```

### Imported Verification
```python
# Import guard's coordinate transforms and boundary logic directly
import sys; sys.path.insert(0, 'scripts/')
from flight_envelope_guard import camera_to_world, world_to_camera

# Verify boundary config matches flight_envelope_guard.yaml
import yaml
with open('config/flight_envelope_guard.yaml') as f:
    cfg = yaml.safe_load(f)
guard_cfg = cfg['flight_envelope_guard']
# Validate world_x_min, world_x_max, world_y_min, world_y_max, world_z_min, world_z_max
```

### Pass Criteria
- Valid in-envelope setpoint → `ACCEPT` → forwarded to MAVROS
- Out-of-bounds command → `REJECT` → blocked with reason published
- Physical drone outside boundary → `FAULT_STATE_OUT_OF_ENVELOPE` triggers
- HOLD state streams last safe position at ≥ 20 Hz

---

## 4. Position Setpoint Control (Item 2.4)

### Objective
Verify the drone tracks position commands with acceptable steady-state error and holds altitude without oscillation.

### Test Procedure

```bash
# 1. Launch full stack (MAVROS + FAST-LIO + relay + guard)
# 2. Arm and switch to OFFBOARD mode
rosrun mavros mavsafety arm
rosrun mavros mavsys mode -c OFFBOARD

# 3. Command a 0.5 m step forward
#    Monitor position tracking via strict_monitor.py
python3 scripts/strict_monitor.py

# 4. Hold altitude at 1.2 m - 1.5 m for 30 seconds
#    Record z-axis oscillation from /mavros/local_position/pose
```

### Imported Verification
```python
# Import strict_monitor for state/pose/completion logging
import sys; sys.path.insert(0, 'scripts/')
from strict_monitor import state_cb, pose_cb

# Import verify_flight for wall clearance checks
from verify_flight import clearance_fn, load_walls
```

### Pass Criteria
- 0.5 m forward step → drone holds position with ≤ 0.1 m steady-state error
- Altitude hold at 1.2 m – 1.5 m stable without oscillation (Z σ ≤ 0.03 m)
- No PX4 failsafe triggers during 30 s hold test

---

## 5. Progressive FUEL Exploration (Item 2.5)

### Objective
Verify FUEL exploration planner generates valid trajectories, avoids obstacles, and achieves ≥ 98% arena coverage.

### Test Procedure

Progressive 3-stage test (each must pass before the next):

#### Stage A: Single Target
```bash
# Launch FUEL with a single viewpoint target
# Import analyze_exploration.py to track coverage
python3 scripts/analyze_exploration.py 120
```
- FUEL generates B-spline trajectory → drone tracks smoothly

#### Stage B: Small Room
```bash
# Restrict FUEL to a small room (e.g. 3 m × 3 m) area
python3 scripts/analyze_exploration.py 300
```
- Drone explores space, retires unreachable frontiers, no oscillation between same targets

#### Stage C: Full Arena
```bash
# Full arena exploration with mission launch
roslaunch nidar_mission nidar_mission.launch
python3 scripts/analyze_exploration.py 1800
```
- Gain-weighted ATSP active, coverage plateau triggers at ≥ 98%

### Imported Verification
```python
# Import analyze_exploration for live coverage and revisit analysis
import sys; sys.path.insert(0, 'scripts/')
from analyze_exploration import Analyzer
# Analyzer tracks: coverage %, revisit scoring, repeat target detection

# Import verify_full_flight for end-to-end stage checks
from verify_full_flight import Verifier
# Verifier checks: yaw churn ratio, peak yaw rate, command jumps, path efficiency

# Import entry detection for mission state machine validation
sys.path.insert(0, 'catkin_ws/src/nidar_mission/scripts/')
from entry_detection_module import MissionState
```

### Pass Criteria
- **Stage A:** B-spline trajectory generated and tracked smoothly
- **Stage B:** Unreachable frontiers retired, no oscillation
- **Stage C:** ≥ 98% coverage, recovers from stale targets
- Yaw churn ratio < 22.4 (better than simulation baseline)
- Peak yaw rate < 60 deg/s
- Command position jumps > 0.30 m: 0

---

## Phase 2 Sign-Off Sheet

| # | Item | Status | Verified By | Notes |
|---|---|:---:|---|---|
| 1 | FAST-LIO2 SLAM Localization | [ ] PASS | | |
| 2 | Odometry Relay & PX4 EKF | [ ] PASS | | |
| 3 | Flight Envelope Guard | [ ] PASS | | |
| 4 | Position Setpoint Control | [ ] PASS | | |
| 5 | Progressive FUEL Exploration | [ ] PASS | | |

> **GATING DECISION:** If all 5 items are marked **PASS**, Phase 2 is officially complete. You may now proceed to **Phase 3 (Perception, Mapping & GCS)**.
