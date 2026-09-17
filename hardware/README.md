# ASTRA Hardware Verification Checklist

Complete phase-by-phase hardware verification guide for the **ASTRA AirMouse** autonomous drone.
---

## The 1 Golden Rule

> **Do not move to the next phase until the current phase passes on real hardware.**

```text
INPUT ──► INTERFACE ──► HARDWARE RESPONSE ──► TELEMETRY CONFIRMATION ──► PASS / FAIL
```

---

## Phase 1: Aircraft Bringup (Bench / Props OFF)

### 1.1 System & Environment
- [ ] Companion computer OS running (Ubuntu / Docker container)
- [ ] ROS Master active (`roscore`)
- [ ] Python dependencies present (`rospy`, `mavros`, `numpy`, `scipy`, `yaml`, `cv2`)
- [ ] Serial ports accessible (`/dev/ttyACM*` or `/dev/ttyUSB*`) with dialout permissions
- [ ] LiDAR Ethernet pingable via static IP

### 1.2 PX4 Flight Controller & MAVROS
- [ ] FCU boots with normal status LED
- [ ] `/mavros/state` reports `connected: true` and stable heartbeat
- [ ] Critical parameters match configuration:
  - `EKF2_EV_CTRL = 11` (external vision position + yaw fusion)
  - `EKF2_HGT_REF = 2` (rangefinder primary height)
  - `MPC_THR_HOVER` matched to physical vehicle mass
- [ ] Arm and Disarm commands execute cleanly

### 1.3 Actuators & Motors (Props OFF!)
- [ ] Motor numbering (1, 2, 3, 4) matches physical Quad X frame geometry
- [ ] Motor rotation directions match CW / CCW designations
- [ ] ESC response is smooth from idle to full range

### 1.4 Sensors
- [ ] **IMU:** Drone tilt (pitch/roll/yaw) matches signs in `/mavros/imu/data`
- [ ] **Barometer:** Stable altitude reading on bench
- [ ] **TFmini Rangefinder:** Actual distance matches `/mavros/distance_sensor/*`
- [ ] **LiDAR:** Point cloud publishing at $\ge 10\text{ Hz}$ with valid timestamps and frame ID

### 1.5 TF & Coordinate Frames
- [ ] Valid TF tree: `world` $\to$ `camera_init` $\to$ `base_link` $\to$ `lidar`, `camera`, `tfmini`
- [ ] Moving drone forward $+X \implies$ TF reports $+X$ (not $-X$ or $Y$)
- [ ] Moving drone left $+Y \implies$ TF reports $+Y$
- [ ] Lifting drone up $+Z \implies$ TF reports $+Z$

---

## Phase 2: Autonomy & Flight Control

### 2.1 FAST-LIO2 SLAM Localization
- [ ] Output rate $\ge 9\text{ Hz}$ sustained on companion CPU
- [ ] No NaN/Inf values, no sudden coordinate jumps
- [ ] **Displacement test:** Carrying drone $1.0\text{ m}$ forward $\implies$ FAST-LIO estimates $1.0\text{ m} \pm 0.05\text{ m}$

### 2.2 Odometry Relay & PX4 EKF
- [ ] `relay_odometry.py` forwards `/Fast_LIO/odometry` $\to$ `/mavros/vision_pose/pose`
- [ ] FAST-LIO pose $\approx$ Relay pose $\approx$ `/mavros/local_position/pose`
- [ ] EKF innovation residuals remain within safe bounds

### 2.3 Flight Envelope Guard
- [ ] Valid in-envelope setpoint $\to$ `ACCEPT` $\to$ forwarded to MAVROS
- [ ] Out-of-bounds command $\to$ `REJECT` $\to$ blocked
- [ ] Physical drone outside boundary $\to$ `FAULT_STATE_OUT_OF_ENVELOPE` triggers

### 2.4 Position Setpoint Control
- [ ] Step command of $0.5\text{ m}$ forward $\implies$ drone holds position with $\le 0.1\text{ m}$ steady-state error
- [ ] Altitude hold stable at $1.2\text{ m} - 1.5\text{ m}$ without oscillation

### 2.5 Progressive FUEL Exploration
- [ ] **Single Target:** Generates B-spline trajectory $\to$ tracks smoothly
- [ ] **Small Room:** Explores space, retires unreachable frontiers, avoids oscillation
- [ ] **Full Arena:** Gain-weighted ATSP active, recovers from stale targets, coverage completion triggers at $\ge 98\%$

---

## Phase 3: Perception, Mapping & GCS

### 3.1 Camera & Survivor Detection (YOLO)
- [ ] Camera stream publishing at $\ge 15\text{ FPS}$
- [ ] YOLO detector runs at $\ge 5\text{ Hz}$ onboard without starving SLAM
- [ ] Zero false positives on plain walls and background obstacles

### 3.2 3D Survivor Localization
- [ ] Bounding box centroid raycasts against registered point cloud
- [ ] 3D position error $\le 0.30\text{ m}$ against surveyed ground-truth targets

### 3.3 Discrete Grid Tagging
- [ ] 3D coordinates map correctly to Discrete Arena Grid format (e.g. A1 – N14)
- [ ] Cell center and boundary test targets correctly tagged

### 3.4 2D Occupancy Grid Mapping
- [ ] 3D point cloud sliced between $0.3\text{ m} \le Z \le 1.9\text{ m}$
- [ ] `/map_2d` publishes as `nav_msgs/OccupancyGrid` at $\ge 2\text{ Hz}$
- [ ] Corridors, rooms, and obstacles clearly distinguished

### 3.5 Ground Control Station (GCS)
- [ ] GCS displays live video, 2D map, survivor tags, drone pose, and mission clock
- [ ] Displayed values match onboard telemetry
- [ ] Total wireless bandwidth $\le 4\text{ Mbps}$

---

## Phase 4: Full Mission & Failsafes

### 4.1 Mission State Machine
- [ ] Telemetric state progression:
  $$\text{PREFLIGHT} \to \text{TAKEOFF} \to \text{ENTRY} \to \text{EXPLORE} \to \text{RETURN} \to \text{EXIT} \to \text{LAND}$$
- [ ] Transitions depend on physical confirmation (e.g. altitude reached, door crossed, coverage plateau reached), never blind timers

### 4.2 Failsafe Injections (Test Each Individually)
- [ ] **Low Battery ($< 20\%$):** Aborts exploration and returns home
- [ ] **Link Loss:** GCS heartbeat timeout triggers autonomous return
- [ ] **Companion Software Halt:** PX4 failsafe transitions to altitude hold / descent
- [ ] **Manual Abort:** Single switch triggers immediate landing sequence
- [ ] **30-Minute Timeout:** Returns home when remaining time equals return cost $+$ margin

### 4.3 Full Integrated Competition Run
- [ ] Single operator "START" command
- [ ] Autonomous takeoff, entry, and exploration
- [ ] $\ge 98\%$ area mapped
- [ ] $\ge 5/6$ survivors detected and tagged in correct grid cells
- [ ] Live map and video streamed to GCS throughout
- [ ] Autonomous return, exit through doorway, and landing on pad ($\le 1.0\text{ m}$ of center)
- [ ] Total mission time $\le 25\text{ minutes}$
