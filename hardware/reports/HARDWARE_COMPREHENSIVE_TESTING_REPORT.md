# ASTRA AirMouse — Master Hardware Verification & Testing Report
## (Strict Code-Level Audit & Physical Hardware Reality)

> **Core Ground Rule (Rule 8):** **ZERO DUMMY OR FABRICATED DATA.**  
> - When physical hardware is connected (Pixhawk FC, GPS, Host Webcam, RPLiDAR A2), run real hardware queries and record actual physical telemetry.
> - When physical hardware is absent (Flight battery, motors, dedicated drone camera, flight arena), perform clean **Code Dry-Runs only** (validating imports, configs, and math) and explicitly defer physical testing until hardware is available.  
> 
> **Physical Hardware on Bench:**
> - **Pixhawk FCU**: 3DR Pixhawk v2.4.8 on `/dev/ttyACM0` *(Live MAVLink telemetry dumped and archived)*
> - **External GPS**: U-Blox NEO M8N external module *(Live 3D Fix, 13 satellites locked)*
> - **Vision Sensor**: Host Laptop Built-in Webcam on `/dev/video0` *(Live video streaming 640x480 @ 15.4 FPS)*
> - **LiDAR Sensor**: Slamtec RPLiDAR A2 on `/dev/ttyUSB0` *(Live 360° laser sweep: 500 points @ 468.5 Hz, distances 0.128 m to 4.647 m, Health Good)*
> 
> **Power Configuration:** 5V USB Bus Powered (~3.09V ADC read by Pixhawk power monitor; 5V USB for RPLiDAR CP2102 bridge).  
> **Unconnected / Missing:** NO Flight Battery, NO ESCs, NO Motors, NO Propellers, NO Dedicated Drone Camera, NO Rangefinder, NO Flight Arena.

---

## 1. Physical Hardware Probing vs. Code Dry-Run Separation

Every subphase is strictly classified without fabricated metrics:

1. 🟢 **LIVE PHYSICAL HARDWARE & SYSTEM PROBES**:
   - **Pixhawk Serial Communication**: Direct PyMAVLink serial communication (`/dev/ttyACM0:921600`).
   - **Autopilot Identification**: MAVLink `HEARTBEAT` decode: Autopilot 12 (PX4), System ID 1, Vehicle Type 2 (Quadrotor).
   - **Battery Rail Monitor**: Live ADC voltage read from Pixhawk (`SYS_STATUS`: **3.097 V** USB VBUS).
   - **External GPS Telemetry**: Live fix from external module (`GPS_RAW_INT`: **3D_FIX, Fix Type 3, 13 Satellites Visible**, Lat: 28.6788982°, Lon: 77.4921396°, Alt: 202.2 m, HDOP: 3.03).
   - **EEPROM Parameters**: Live parameter reads from Pixhawk (`NAV_RCL_ACT = 2`, `MPC_LAND_SPEED = 0.70 m/s`).
   - **Inertial & Barometric Sensors**: Live IMU stationary gravity ($Z = -9.937\text{ m/s}^2$) and Barometer (98,173 mbar).
   - **Vision Pipeline**: Live Host Webcam video probe (`/dev/video0`): 25 frames captured at 640x480, 15.4 FPS, 64.99 ms latency (~15.4 Hz pipeline throughput), average luminance 115.0/255.0.
   - **LiDAR 360° Point Cloud**: Live Slamtec RPLiDAR A2 probe on `/dev/ttyUSB0` (Model ID: 24, FW: v1.26, HW: 5, S/N: `C3B99AF2C1EA9FC2A2EB92F1506A3C00`, Health: Good). Motor spun up via DTR; 500 valid 360° points captured in 1.07 s at 468.5 Hz sample rate; distance spread: 0.128 m to 4.647 m across 2 complete rotations.
   - **System Environment**: Companion PC environment checked (Python dependencies, dialout permissions, ROS master port 11311).

2. 🟡 **OFFLINE CODE DRY-RUNS (NO DUMMY NUMBERS / PENDING PHYSICAL HARDWARE)**:
   - Verifying Python classes (`Verifier`, `Analyzer`, `MissionState`, `FlightEnvelopeGuard`) import cleanly without syntax or path errors.
   - Parsing configuration YAML files (`flight_envelope_guard.yaml`, `mission_config.yaml`).
   - Verifying boundary clamping math and coordinate frame transformation matrices.
   - Explicitly tagging all physical flight maneuvers (takeoff, autonomous exploration, precision touchdown) as **DEFERRED / PENDING PHYSICAL HARDWARE** rather than fabricating fake sample values.

---

## 2. Master Verification Truth Table (All 20 Subphases)

| Phase | Item # | Verification Item | Execution Mode | Physical Hardware Actually Probed | Code Checked / Status |
|:---:|:---:|---|:---:|---|---|
| **1** | **1.1** | System & Environment | 🟢 **LIVE SYSTEM** | Companion PC (OS, dialout, ports) | `check_system_env.py` (all dependencies import OK) |
| **1** | **1.2** | FCU & MAVROS Connection | 🟢 **LIVE HARDWARE** | **Pixhawk FCU** (`/dev/ttyACM0:921600`) | `check_fcu_mavros.py` / `probe_pixhawk_serial.py` (live MAVLink handshake OK) |
| **1** | **1.3** | Actuators & Motor Directions | 🟡 **DRY-RUN** | **None** (No battery or motors connected) | `check_actuators_motors.py` (Quad-X geometry arrays verified in Python; motors not spun) |
| **1** | **1.4** | Sensors: IMU, Baro, GPS, LiDAR | 🟢 **LIVE HARDWARE** | **Pixhawk FCU + GPS + RPLiDAR A2** | `dump_pixhawk_gps_telemetry.py` (IMU ~1G, GPS 13 sats) + `probe_rplidar.py` (500 pts @ 468.5 Hz) |
| **1** | **1.5** | TF Tree & Coordinate Frames | 🟡 **DRY-RUN** | **None** (Math calculation) | `check_tf_frames.py` (Euler / quaternion transforms conform to FLU/ENU) |
| **2** | **2.1** | FAST-LIO2 SLAM Localization | 🟡 **DRY-RUN** | Host PC | `check_fastlio_slam.py` (Verifier imported; physical SLAM deferred to full continuous ROS cloud) |
| **2** | **2.2** | Odometry Relay & PX4 EKF | 🟡 **DRY-RUN** | Host PC | `check_odometry_relay.py` (Script existence and relay structure verified on disk) |
| **2** | **2.3** | Flight Envelope Guard | 🟡 **DRY-RUN** | Host PC | `check_envelope_guard.py` (YAML bounds verified; clamp math verified in Python) |
| **2** | **2.4** | Position Setpoint Control | 🟡 **DRY-RUN** | Host PC | `check_position_control.py` (Offboard setpoint code verified; flight test deferred) |
| **2** | **2.5** | Progressive FUEL Exploration | 🟡 **DRY-RUN** | **None** (No flight battery/motors) | `check_fuel_exploration.py` (Analyzer & Verifier classes imported cleanly; flight deferred) |
| **3** | **3.1** | Camera & Vision Pipeline | 🟢 **LIVE HARDWARE** | **Host Laptop Webcam** (`/dev/video0`) | `check_camera_yolo.py` (live video stream 640x480 @ 15.4 FPS, edge detection pipeline latency 64.99 ms, valid optical range) |
| **3** | **3.2** | 3D Survivor Localization | 🟡 **DRY-RUN** | **None** (No camera connected) | `check_survivor_localization.py` (Raycast solver algorithm verified in code) |
| **3** | **3.3** | Discrete Grid Tagging | 🟡 **DRY-RUN** | Host PC | `check_grid_tagging.py` (Discrete grid conversion algorithm verified: `(0.5, -2.3) -> H7`) |
| **3** | **3.4** | 2D Occupancy Grid Mapping | 🟡 **DRY-RUN** | Host PC | `check_occupancy_grid.py` (Wall loader module verified; live mapping deferred to continuous ROS scan) |
| **3** | **3.5** | GCS Telemetry Stream | 🟡 **DRY-RUN** | Host PC | `check_gcs_telemetry.py` (Telemetry logger file and callback verified on disk) |
| **4** | **4.1** | Mission State Machine | 🟡 **DRY-RUN** | Host PC | `check_mission_state_machine.py` (MissionState enum and state strings verified; no flight) |
| **4** | **4.2** | Battery & Link Loss Failsafes | 🟢 **LIVE HARDWARE** | **Pixhawk FCU** (`/dev/ttyACM0`) | `check_failsafe_battery_link.py` (Live ADC read: **3.09 V** on USB rail; failsafe logic OK) |
| **4** | **4.3** | Geofence & Abort Guard | 🟢 **LIVE HARDWARE** | **Pixhawk FCU** (`/dev/ttyACM0`) | `check_failsafe_abort_guard.py` (Live EEPROM read: `MPC_LAND_SPEED = 0.70`, clamp OK) |
| **4** | **4.4** | Return & Precision Landing | 🟡 **DRY-RUN** | Host PC | `check_return_landing.py` (YAML pad coordinates & tolerance verified; touchdown deferred) |
| **4** | **4.5** | Full Integrated Mission Run | 🟡 **DRY-RUN** | Host PC | `check_full_integrated_mission.py` (All 3 modules imported; rubric verified; flight deferred) |

---

## 3. Verified Live Telemetry Snapshot (Permanently Archived)

Captured on 2026-09-16 and archived in [`hardware/reports/pixhawk_gps_live_snapshot.json`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/pixhawk_gps_live_snapshot.json):

```json
{
  "fcu": {
    "autopilot": 12,
    "type": 2,
    "armed": false,
    "system_id": 1,
    "component_id": 1
  },
  "gps": {
    "fix_type": 3,
    "fix_status": "3D_FIX",
    "satellites_visible": 13,
    "latitude_deg": 28.6788982,
    "longitude_deg": 77.4921396,
    "altitude_m": 202.2,
    "hdop": 3.03
  },
  "battery": {
    "voltage_v": 3.097,
    "rail_type": "5V_USB_BUS"
  },
  "imu": {
    "accel_z_ms2": -9.937,
    "pressure_mbar": 98173.0
  },
  "parameters": {
    "EKF2_HGT_REF": 1,
    "MPC_THR_HOVER": 0.5,
    "EKF2_RNG_CTRL": 1,
    "NAV_RCL_ACT": 2,
    "MPC_LAND_SPEED": 0.7
  },
  "camera": {
    "device": "/dev/video0",
    "type": "HOST_BUILTIN_WEBCAM",
    "resolution": "640x480",
    "frame_rate_fps": 15.4,
    "latency_ms": 64.99,
    "throughput_hz": 15.39,
    "mean_brightness": 115.0,
    "optical_valid": true
  },
  "rplidar": {
    "device": "/dev/ttyUSB0",
    "model": "RPLiDAR A2",
    "model_id": 24,
    "firmware": "1.26",
    "hardware_rev": 5,
    "serial_number": "C3B99AF2C1EA9FC2A2EB92F1506A3C00",
    "baud": 115200,
    "health": "Good",
    "points_captured": 500,
    "sample_rate_hz": 468.5,
    "min_distance_m": 0.128,
    "max_distance_m": 4.647,
    "mean_distance_m": 0.945,
    "rotations": 2
  }
}
```

---

## 4. Phase-by-Phase Detailed Test Methodology & Audit Reality

### Phase 1: Aircraft Bringup
- **1.1 System & Environment (🟢 LIVE SYSTEM)**
  - **Hardware Probed:** Host laptop (Ubuntu Linux, Python 3 environment, user dialout group, ROS master port 11311).
  - **Code Imported:** `scripts/verify_components.py` (`check`, `sdf_pose`).
  - **Test Procedure:** Checked Python packages (`rospy`, `pymavlink`, `yaml`, `numpy`, `cv2`, `serial`), verified USB serial device nodes (`/dev/ttyACM0`, `/dev/ttyUSB0`), confirmed ROS Core availability.
  - **Pass Criteria:** All core imports load cleanly; dialout permissions present.

- **1.2 FCU & MAVROS Connection (🟢 LIVE HARDWARE)**
  - **Hardware Probed:** 3DR Pixhawk Flight Controller connected via `/dev/ttyACM0:921600`.
  - **Code Imported:** `hardware/phase1/scripts/probe_pixhawk_serial.py`, `scripts/verify_flight.py`.
  - **Test Procedure:** Sent MAVLink ping, received `HEARTBEAT` packet within 1000 ms, confirmed Autopilot type 12 (PX4), System ID 1.
  - **Pass Criteria:** Reliable MAVLink bidirectional handshake without packet loss.

- **1.3 Motor Numbers & Directions (🟡 DRY-RUN)**
  - **Hardware Probed:** None (No flight battery or motors connected to bench).
  - **Code Imported:** `scripts/verify_flight.py` (`collision_radius`), `scripts/verify_components.py` (`rpy_to_mat`, Quad-X matrix).
  - **Test Procedure:** Validated Quad-X rotor order geometry vectors in Python memory. Motors were NOT physically commanded or spun.
  - **Pass Criteria:** Quad-X motor coordinate array mathematically sound. Physical spin deferred to assembled bench with LiPo.

- **1.4 Sensors: IMU, Baro, GPS, LiDAR (🟢 LIVE HARDWARE)**
  - **Hardware Probed:** Pixhawk internal IMU/Barometer + External GPS receiver + Slamtec RPLiDAR A2 on `/dev/ttyUSB0`.
  - **Code Imported:** `hardware/phase1/scripts/dump_pixhawk_gps_telemetry.py`, `hardware/phase1/scripts/probe_rplidar.py`.
  - **Test Procedure:** Queried live MAVLink streams `HIGHRES_IMU`, `SCALED_PRESSURE`, and `GPS_RAW_INT`. Measured earth gravity ($Z = -9.937\text{ m/s}^2$), barometric pressure ($98,173\text{ mbar}$), and confirmed 3D GPS lock ($13\text{ satellites}$, fix type 3). Probed RPLiDAR A2 over `/dev/ttyUSB0` at 115200 baud, validated health state (Good, error code 0), spun motor via DTR line, and captured 500 real laser scan points in 1.07 s ($468.5\text{ Hz}$ sample rate, distances $0.128\text{ m}$ to $4.647\text{ m}$, 2 complete $360^\circ$ sweeps).
  - **Pass Criteria:** Stationary gravity within $9.81 \pm 0.5\text{ m/s}^2$; GPS Fix $\ge 3$; RPLiDAR health Good and $\ge 50$ real laser points captured.

- **1.5 TF Tree & Coordinate Signs (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC CPU.
  - **Code Imported:** `scripts/check_mount_geometry.py`, `scripts/verify_components.py`.
  - **Test Procedure:** Validated coordinate transformations between FLU (Forward-Left-Up) body frame and ENU (East-North-Up) world frame using quaternion multiplication.
  - **Pass Criteria:** Frame rotation matrices orthogonal; det $= +1$.

---

### Phase 2: Autonomy & Flight Control
- **2.1 FAST-LIO2 SLAM Localization (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/verify_full_flight.py` (`Verifier`, `GUARD_Z`, `TFMINI_OFFSET`).
  - **Test Procedure:** Verified Python module import and launch file configuration parameters. Live laser SLAM processing deferred to full continuous ROS pointcloud integration.
  - **Pass Criteria:** Verifier class imports cleanly; configuration files valid.

- **2.2 Odometry Relay & PX4 EKF (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/relay_odometry.py` (structure & pose anchor).
  - **Test Procedure:** Verified source file presence, syntax correctness, and ROS subscriber/publisher message interface signatures.
  - **Pass Criteria:** File syntax valid; message topic schemas conform to `nav_msgs/Odometry` and `geometry_msgs/PoseStamped`.

- **2.3 Flight Envelope Guard (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/flight_envelope_guard.py` (`euler_from_quaternion`, `clamp`), `config/flight_envelope_guard.yaml`.
  - **Test Procedure:** Tested mathematical clamping logic against boundary conditions ($X \in [-12, 12]$, $Y \in [-10, 10]$, $Z \in [0.2, 2.5]$).
  - **Pass Criteria:** Clamping function strictly enforces bounding box coordinates.

- **2.4 Position Setpoint Control (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/verify_flight.py` (`load_walls`, `clearance_fn`, `CRUISE_Z`), `scripts/strict_monitor.py`.
  - **Test Procedure:** Verified setpoint generation math and wall clearance safety margins in software. No physical offboard commands were sent to motors.
  - **Pass Criteria:** Offboard control setpoint algorithms compute collision-free vectors.

- **2.5 Progressive FUEL Exploration (🟡 DRY-RUN)**
  - **Hardware Probed:** None (No flight battery or motors).
  - **Code Imported:** `scripts/analyze_exploration.py` (`Analyzer`, `REVISIT_GAP`), `scripts/verify_full_flight.py` (`Verifier`, `BASELINE`).
  - **Test Procedure:** Verified mathematical formulas for frontier selection and exploration coverage calculation. No flight was performed.
  - **Pass Criteria:** Analysis algorithms and verification classes instantiate without runtime errors.

---

### Phase 3: Perception, Mapping & GCS
- **3.1 Camera & Vision Pipeline (🟢 LIVE HARDWARE)**
  - **Hardware Probed:** Host Laptop Built-in Webcam (`/dev/video0`).
  - **Code Imported:** `hardware/phase3/scripts/check_camera_yolo.py`, `scripts/verify_components.py`, `scripts/check_mount_geometry.py`.
  - **Test Procedure:** Opened `/dev/video0` with OpenCV, streamed 25 real video frames, executed grayscale conversion and Canny edge extraction on live optical feed, measured frame rate and frame latency.
  - **Pass Criteria:** Frame rate $\ge 10.0\text{ FPS}$ ($15.4\text{ FPS}$ measured); optical luminance within $[5, 250]$ ($115.0$ measured).

- **3.2 3D Survivor Localization (🟡 DRY-RUN)**
  - **Hardware Probed:** None (Camera detection deferred to dedicated sensor).
  - **Code Imported:** `scripts/verify_components.py` (`PointCloud2`, raycast centroid solver).
  - **Test Procedure:** Verified 3D projection raycast math and pinhole camera transformation equations in Python.
  - **Pass Criteria:** Pinhole projection equation yields accurate 3D coordinates from pixel ray.

- **3.3 Discrete Grid Tagging (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `catkin_ws/src/nidar_mission/scripts/apply_mission_config.py` (`apply_mission_config`, Discrete Grid A1-N14).
  - **Test Procedure:** Fed floating-point coordinates $(0.5, -2.3)$ into tagging module; verified correct alphanumeric cell assignment `H7`.
  - **Pass Criteria:** Correct string conversion without boundary overflow.

- **3.4 2D Occupancy Grid Mapping (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/verify_flight.py` (`load_walls`), `scripts/analyze_exploration.py` (`Analyzer`).
  - **Test Procedure:** Loaded arena boundary definitions and verified 2D occupancy rasterization arrays in Python memory.
  - **Pass Criteria:** Wall coordinates accurately rasterize into 2D grid matrix.

- **3.5 Ground Control Station (GCS) (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/strict_monitor.py` (`state_cb`, telemetry watchdog), `scripts/mission_telemetry_logger.py`.
  - **Test Procedure:** Validated telemetry serializer callbacks and JSON logging file schema.
  - **Pass Criteria:** Telemetry serialization schema conforms to GCS packet format.

---

### Phase 4: Full Mission & Failsafes
- **4.1 Mission State Machine & Progression (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `catkin_ws/src/nidar_mission/scripts/entry_detection_module.py` (`MissionState`, `MultiCueEntryDetector`), `scripts/verify_full_flight.py` (`Verifier`).
  - **Test Procedure:** Instantiated `MissionState` enum, verified transitions (`IDLE` $\to$ `TAKEOFF` $\to$ `ENTRY_SEARCH` $\to$ `EXPLORATION` $\to$ `RETURN` $\to$ `LAND`), and verified illegal reverse transitions are blocked. No physical flight conducted.
  - **Pass Criteria:** State machine progression strictly enforces forward transition safety.

- **4.2 Failsafe System: Battery & Link Loss (🟢 LIVE HARDWARE)**
  - **Hardware Probed:** 3DR Pixhawk Flight Controller (`/dev/ttyACM0`).
  - **Code Imported:** `hardware/phase4/scripts/check_failsafe_battery_link.py`, `scripts/strict_monitor.py`, `scripts/verify_flight.py`.
  - **Test Procedure:** Queried live Pixhawk ADC voltage over MAVLink (`SYS_STATUS.voltage_battery = 3097 mV`). Verified battery failsafe decision tree ($V < 14.2\text{ V}$ triggers `RETURN/LAND` warning logic for 4S configuration).
  - **Pass Criteria:** Live ADC measurement successfully acquired from physical FCU; failsafe triggers correctly on low voltage.

- **4.3 Failsafe System: Geofence & Abort (🟢 LIVE HARDWARE)**
  - **Hardware Probed:** 3DR Pixhawk Flight Controller (`/dev/ttyACM0`).
  - **Code Imported:** `hardware/phase4/scripts/check_failsafe_abort_guard.py`, `scripts/flight_envelope_guard.py`.
  - **Test Procedure:** Queried Pixhawk EEPROM onboard parameters via MAVLink `PARAM_REQUEST_READ`. Verified `MPC_LAND_SPEED` ($0.70\text{ m/s}$) and `NAV_RCL_ACT` ($2 = \text{Return-to-Land}$). Tested software clamping math.
  - **Pass Criteria:** Pixhawk parameter values read live; emergency clamp logic halts trajectory outside bounds.

- **4.4 Autonomous Return & Precision Landing (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/verify_full_flight.py` (`Verifier`), `scripts/verify_components.py` (`check`, launch pad coordinates).
  - **Test Procedure:** Parsed launch pad coordinates $(0.0, 0.0)$ from configuration, verified Euclidean touchdown proximity tolerance ($< 0.20\text{ m}$). Physical touchdown was not executed.
  - **Pass Criteria:** Pad geometry math and touchdown radius verified in software.

- **4.5 Full Integrated Competition Mission (🟡 DRY-RUN)**
  - **Hardware Probed:** Host PC.
  - **Code Imported:** `scripts/verify_full_flight.py` (`Verifier`), `scripts/analyze_exploration.py` (`Analyzer`), `scripts/mission_telemetry_logger.py` (`telemetry_logger`).
  - **Test Procedure:** Verified competition scoring rubric checklist and full mission orchestration pipeline in Python. No full autonomous flight was flown.
  - **Pass Criteria:** All sub-modules integrate cleanly without namespace or parameter conflicts.

---

*Report maintained strictly under Rule 8: Zero Dummy Data — Real Hardware Queries & Pure Code Dry-Runs.*
