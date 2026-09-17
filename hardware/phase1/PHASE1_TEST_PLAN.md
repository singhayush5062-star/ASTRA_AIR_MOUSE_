# Phase 1: Aircraft Bringup — Hardware Test Plan

**Scope:** Bench-level verification of the Companion Computer, PX4 Flight Controller, Actuators (Props OFF), Sensors (IMU, Baro, TFmini, LiDAR), and ROS Coordinate Frames before any flight.

---

## The Core Rule

> **Props MUST be removed from all motors for Phase 1 tests.**  
> Do not advance to Phase 2 (Autonomy & Flight Control) until every section below passes.

---

## Code Import & Reuse Strategy

To ensure zero duplicate code creation and exact parity with system simulation/verification math:
- All Phase 1 bringup scripts in `hardware/phase1/scripts/` import directly from existing root workspace scripts:
  - [`scripts/verify_components.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_components.py): Reused for prop clearance math, quaternion/RPY transformations, TFmini beam elevation, camera aiming vectors, and LiDAR pointcloud validation.
  - [`scripts/check_mount_geometry.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/check_mount_geometry.py): Reused for rotated 3D camera mesh z-extents and standoff leg clearance calculations.
  - [`scripts/verify_flight.py`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_flight.py): Reused for prop collision radius math (`collision_radius()`).

## Persistent Execution & Implementation Logging

All execution runs and gating decisions are automatically recorded in:
- [`hardware/reports/HARDWARE_STATUS_REPORT.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_STATUS_REPORT.md)
- [`hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md`](file:///c:/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md)

---

## 1. System & Environment Test

### Objective
Ensure the companion computer OS, ROS Noetic environment, Python modules, and communication interfaces are active.

### Test Procedure

```bash
# 1. Verify ROS Core
roscore &

# 2. Check Python Dependencies
python3 -c "import rospy, mavros, numpy, scipy, yaml, cv2; print('PYTHON DEPS OK')"

# 3. Check Serial Device Permissions
ls -l /dev/ttyACM* /dev/ttyUSB*
# User must be in 'dialout' group: sudo usermod -aG dialout $USER

# 4. Check LiDAR Network Connection (if Ethernet)
ping -c 3 192.168.1.102  # Replace with actual LiDAR static IP
```

### Pass Criteria
- `PYTHON DEPS OK` is printed with no `ImportError`.
- Serial device has read/write permissions for current user.
- 0% packet loss on LiDAR ping.

---

## 2. PX4 Flight Controller & MAVROS Connection

### Objective
Verify reliable two-way telemetry and command link between Companion Computer and FCU.

### Test Procedure

```bash
# 1. Launch MAVROS (adjust port/baudrate to match your physical link, e.g., /dev/ttyACM0:921600)
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600"

# 2. In a second terminal, verify state and heartbeat
rostopic echo -n 1 /mavros/state
rostopic hz /mavros/state
```

### Expected Output
- `connected: True`
- `mode: "MANUAL"` (or `POSCTL` / `OFFBOARD` as set)
- Frequency $\ge 2.0\text{ Hz}$

### Parameter Verification
Run MAVROS parameter service checks or QGroundControl:
```bash
# Query key parameters
rosrun mavros mavparam get EKF2_EV_CTRL   # Must be 11 (Vision position + yaw fusion)
rosrun mavros mavparam get EKF2_HGT_REF   # Must be 2 (Rangefinder) or 3 (Vision)
rosrun mavros mavparam get MPC_THR_HOVER  # Matched to physical vehicle thrust-to-weight
```

### Arm / Disarm Software Test
```bash
# Arm vehicle (PROPS MUST BE OFF!)
rosrun mavros mavsafety arm

# Disarm vehicle
rosrun mavros mavsafety disarm
```

### Pass Criteria
- FCU connects with no dropped heartbeats.
- Arm and Disarm succeed with audible/telemetric confirmation.

---

## 3. Actuators, Motors & ESC Test (PROPS OFF!)

### Objective
Confirm motor order, rotation direction (CW/CCW), and ESC throttle response.

### Quad X Layout Reference
```text
      Front
   (4) CW   (2) CCW
        \   /
         \ /
         / \
        /   \
  (3) CCW   (1) CW
       Back
```

### Test Procedure (QGroundControl / MAVROS Actuator Test)
1. In QGroundControl $\to$ **Vehicle Setup** $\to$ **Actuators / Motors**:
   - Slide **Motor 1** $\to$ verify **Rear-Right** spins **Clockwise (CW)**.
   - Slide **Motor 2** $\to$ verify **Front-Right** spins **Counter-Clockwise (CCW)**.
   - Slide **Motor 3** $\to$ verify **Rear-Left** spins **Counter-Clockwise (CCW)**.
   - Slide **Motor 4** $\to$ verify **Front-Left** spins **Clockwise (CW)**.
2. Slowly ramp throttle from 10% to 50% on all motors:
   - Check for smooth acceleration without stuttering or desync.

### Pass Criteria
- Every motor matches its assigned physical corner and spin direction.
- Zero motor stutter or unexpected vibrations.

---

## 4. Sensor Validation

### 4.1 IMU Verification
```bash
rostopic echo /mavros/imu/data
```
- **Pitch up (nose up):** `orientation.y` / angular velocity registers positive change.
- **Roll right (right wing down):** `orientation.x` registers positive change.
- **Yaw right (clockwise from top):** `orientation.z` registers positive/correct heading change.
- **Z-Acceleration stationary:** $\approx +9.81\text{ m/s}^2$ (or $-9.81\text{ m/s}^2$ depending on standard).

### 4.2 Barometer Verification
```bash
rostopic echo /mavros/altitude
```
- Stationary altitude reading on bench must remain within $\pm 0.1\text{ m}$ without drifting wildly.

### 4.3 TFmini Rangefinder Verification
```bash
rostopic echo /mavros/distance_sensor/rangefinder_sub # Or your configured TFmini topic
```
- Place a flat cardboard target at measured distances:
  - At **0.20 m:** reported distance is $0.20 \pm 0.02\text{ m}$
  - At **0.50 m:** reported distance is $0.50 \pm 0.03\text{ m}$
  - At **1.00 m:** reported distance is $1.00 \pm 0.05\text{ m}$

### 4.4 LiDAR Point Cloud Verification
```bash
# Launch your LiDAR driver (e.g. Livox / Velodyne / RPLidar)
rostopic hz /livox/lidar  # Or /velodyne_points / /scan
rostopic echo -n 1 /livox/lidar | grep frame_id
```
- Point cloud publishes at $\ge 10.0\text{ Hz}$.
- `frame_id` matches the TF tree definition (e.g., `lidar_link` or `livox_frame`).
- Visual confirmation in RViz shows clear geometric room contours.

---

## 5. TF & Coordinate Frames Verification

### Objective
Ensure the robot kinematics and sensor offsets are correctly aligned without axis inversion.

### Test Procedure
```bash
# 1. View the TF tree
rosrun tf tf_echo camera_init base_link
# Or generate visual PDF:
rosrun tf view_frames && evince frames.pdf
```

### Physical Displacement Verification
Hold the drone and manually translate it along known axes:
1. **Move +1.0 m Forward:** TF $X$ translation increases by $+1.0\text{ m}$ (not negative, not $Y$).
2. **Move +1.0 m Left:** TF $Y$ translation increases by $+1.0\text{ m}$.
3. **Lift +0.5 m Up:** TF $Z$ translation increases by $+0.5\text{ m}$.

### Pass Criteria
- No broken or disconnected frames in TF tree.
- Motion along all 3 Cartesian axes strictly adheres to the right-hand convention without inversion.

---

## Phase 1 Sign-Off Sheet

| # | Item | Status | Verified By | Notes |
|---|---|:---:|---|---|
| 1 | System & Dependencies | [ ] PASS | | |
| 2 | FCU & MAVROS Connection | [ ] PASS | | |
| 3 | Motor Numbers & Directions | [ ] PASS | | |
| 4 | IMU, Baro, TFmini & LiDAR | [ ] PASS | | |
| 5 | TF Tree & Coordinate Signs | [ ] PASS | | |

> **GATING DECISION:** If all 5 items are marked **PASS**, Phase 1 is officially complete. You may now proceed to **Phase 2 (Autonomy & Flight Control)**.
