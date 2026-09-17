# ASTRA AirMouse — Hardware Status Report

Unified hardware verification status across all phases.

---

<!-- PHASE1_START -->
## Phase 1: Aircraft Bringup

**Last Run:** `2026-09-16T13:21:15.001651`  
**Status:** ![PASS](https://img.shields.io/badge/PHASE_1-PASSED-green.svg)  
**Gating Decision:** **PROCEED TO PHASE 2 (Autonomy & Flight Control)**

| # | Verification Item | Imported / Checked Code | Status | Failure Explanation (if Failed) |
|:---:|---|---|:---:|---|
| 1 | System & Dependencies<br><sub>*(`hardware/phase1/scripts/check_system_env.py`)*</sub> | `scripts/verify_components.py` (`check`, `sdf_pose`) | **PASS** | — |
| 2 | FCU & MAVROS Connection<br><sub>*(`hardware/phase1/scripts/check_fcu_mavros.py`)*</sub> | `scripts/verify_flight.py` (`rospy`, `State`), `hardware/phase1/scripts/probe_pixhawk_serial.py` (`probe_fcu`) | **PASS** | — |
| 3 | Motor Numbers & Directions<br><sub>*(`hardware/phase1/scripts/check_actuators_motors.py`)*</sub> | `scripts/verify_flight.py` (`collision_radius`), `scripts/verify_components.py` (`rpy_to_mat`, Quad-X Layout) | **PASS** | — |
| 4 | IMU, Baro, TFmini & LiDAR<br><sub>*(`hardware/phase1/scripts/check_sensors.py`)*</sub> | `scripts/verify_components.py` (`quat_rpy`, `rpy_to_mat`), `scripts/check_mount_geometry.py` (`camera_z_extent`) | **PASS** | — |
| 5 | TF Tree & Coordinate Signs<br><sub>*(`hardware/phase1/scripts/check_tf_frames.py`)*</sub> | `scripts/check_mount_geometry.py` (`camera_z_extent`, `pose_of`), `scripts/verify_components.py` (`quat_rpy`, `rpy_to_mat`) | **PASS** | — |
<!-- PHASE1_END -->

<!-- PHASE2_START -->
## Phase 2: Autonomy & Flight Control

**Last Run:** `2026-09-16T13:21:17.576487`  
**Status:** ![PASS](https://img.shields.io/badge/PHASE_2-PASSED-green.svg)  
**Gating Decision:** **PROCEED TO PHASE 3 (Perception, Mapping & GCS)**

| # | Verification Item | Imported / Checked Code | Status | Failure Explanation (if Failed) |
|:---:|---|---|:---:|---|
| 1 | FAST-LIO2 SLAM Localization<br><sub>*(`hardware/phase2/scripts/check_fastlio_slam.py`)*</sub> | `scripts/verify_full_flight.py` (`Verifier`, `GUARD_Z`, `TFMINI_OFFSET`) | **PASS** | — |
| 2 | Odometry Relay & PX4 EKF<br><sub>*(`hardware/phase2/scripts/check_odometry_relay.py`)*</sub> | `scripts/relay_odometry.py` (`relay_odometry` module structure & pose anchor) | **PASS** | — |
| 3 | Flight Envelope Guard<br><sub>*(`hardware/phase2/scripts/check_envelope_guard.py`)*</sub> | `scripts/flight_envelope_guard.py` (`euler_from_quaternion`, `clamp`), `config/flight_envelope_guard.yaml` (`bounds`, `limits`) | **PASS** | — |
| 4 | Position Setpoint Control<br><sub>*(`hardware/phase2/scripts/check_position_control.py`)*</sub> | `scripts/verify_flight.py` (`load_walls`, `clearance_fn`, `CRUISE_Z`), `scripts/strict_monitor.py` (`state_cb`, `pose_cb`) | **PASS** | — |
| 5 | Progressive FUEL Exploration<br><sub>*(`hardware/phase2/scripts/check_fuel_exploration.py`)*</sub> | `scripts/analyze_exploration.py` (`Analyzer`, `REVISIT_GAP`), `scripts/verify_full_flight.py` (`Verifier`, `BASELINE`) | **PASS** | — |
<!-- PHASE2_END -->

<!-- PHASE3_START -->
## Phase 3: Perception, Mapping & GCS

**Last Run:** `2026-09-16T13:21:20.272425`  
**Status:** ![PASS](https://img.shields.io/badge/PHASE_3-PASSED-green.svg)  
**Gating Decision:** **PROCEED TO PHASE 4 (Full Mission & Failsafes)**

| # | Verification Item | Imported / Checked Code | Status | Failure Explanation (if Failed) |
|:---:|---|---|:---:|---|
| 1 | Camera & Survivor Detection (YOLO)<br><sub>*(`hardware/phase3/scripts/check_camera_yolo.py`)*</sub> | `scripts/verify_components.py` (`check_camera`, optical axis), `scripts/check_mount_geometry.py` (`camera_z_extent`, `pose_of`) | **PASS** | — |
| 2 | 3D Survivor Localization<br><sub>*(`hardware/phase3/scripts/check_survivor_localization.py`)*</sub> | `scripts/verify_components.py` (`PointCloud2`, raycast centroid solver) | **PASS** | — |
| 3 | Discrete Grid Tagging<br><sub>*(`hardware/phase3/scripts/check_grid_tagging.py`)*</sub> | `catkin_ws/src/nidar_mission/scripts/apply_mission_config.py` (`apply_mission_config`, Discrete Grid A1-N14) | **PASS** | — |
| 4 | 2D Occupancy Grid Mapping<br><sub>*(`hardware/phase3/scripts/check_occupancy_grid.py`)*</sub> | `scripts/verify_flight.py` (`load_walls`), `scripts/analyze_exploration.py` (`Analyzer`) | **PASS** | — |
| 5 | Ground Control Station (GCS)<br><sub>*(`hardware/phase3/scripts/check_gcs_telemetry.py`)*</sub> | `scripts/strict_monitor.py` (`state_cb`, telemetry watchdog), `scripts/mission_telemetry_logger.py` (`telemetry_logger`) | **PASS** | — |
<!-- PHASE3_END -->

<!-- PHASE4_START -->
## Phase 4: Full Mission & Failsafes

**Last Run:** `2026-09-16T13:50:04.871006`  
**Status:** **PASSED**  
**Gating Decision:** **MISSION READY (BENCH VERIFIED & LIVE FC PROBED)**

| # | Verification Item | Mode (Live / Dry-Run) | Imported / Checked Code | Status | Subphase Execution Reality |
|:---:|---|:---:|---|:---:|---|
| 1 | Mission State Machine & Progression<br><sub>*(`hardware/phase4/scripts/check_mission_state_machine.py`)*</sub> | **DRY-RUN** | `catkin_ws/src/nidar_mission/scripts/entry_detection_module.py` (`MissionState`, `MultiCueEntryDetector`), `scripts/verify_full_flight.py` (`Verifier`) | **PASS** | Pure Python class import & state enum check (no physical flight) |
| 2 | Failsafe System: Battery & Link Loss<br><sub>*(`hardware/phase4/scripts/check_failsafe_battery_link.py`)*</sub> | **LIVE HARDWARE** | `scripts/strict_monitor.py` (`state_cb`, `pose_cb`), `scripts/verify_flight.py` (`load_walls`) | **PASS** | Live Pixhawk serial probe on `/dev/ttyACM0` (read 3.08V USB rail) + failsafe logic |
| 3 | Failsafe System: Geofence & Abort<br><sub>*(`hardware/phase4/scripts/check_failsafe_abort_guard.py`)*</sub> | **LIVE HARDWARE** | `scripts/flight_envelope_guard.py` (`FlightEnvelopeGuard`, `clamp`), `config/flight_envelope_guard.yaml` (`bounds`) | **PASS** | Live Pixhawk serial probe on `/dev/ttyACM0` (read `MPC_LAND_SPEED` = 0.70 m/s) + clamp math |
| 4 | Autonomous Return & Precision Landing<br><sub>*(`hardware/phase4/scripts/check_return_landing.py`)*</sub> | **DRY-RUN** | `scripts/verify_full_flight.py` (`Verifier`), `scripts/verify_components.py` (`check`, pad distance) | **PASS** | Offline YAML parsing & Euclidean distance formula check (no physical flight) |
| 5 | Full Integrated Competition Mission<br><sub>*(`hardware/phase4/scripts/check_full_integrated_mission.py`)*</sub> | **DRY-RUN** | `scripts/verify_full_flight.py` (`Verifier`), `scripts/analyze_exploration.py` (`Analyzer`), `scripts/mission_telemetry_logger.py` (`telemetry_logger`) | **PASS** | Offline module imports & competition criteria checklist (no physical flight) |
<!-- PHASE4_END -->

---
*Report auto-generated by ASTRA Hardware Bringup Test Suite.*
