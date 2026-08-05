# Walkthrough — NIDAR AirMouse Implementation & Verification

This document summarizes the technical updates, fixes, and runtime verification results executed according to the approved implementation plan.

---

## 1. Summary of Completed Actions

### Competition Arena Mesh Orientation Fix (`tomar.dae`)
- **Problem**: `tomar.dae` contained `<up_axis>Y_UP</up_axis>`, which caused Gazebo Classic (`Z_UP`) to apply a $-90^\circ$ coordinate rotation, spawning the arena inverted.
- **Fix**: Updated `<up_axis>Z_UP</up_axis>` in both [ARENA/tomar.dae](file:///home/ayush/Desktop/NIDAR/ARENA/tomar.dae) and [model mesh](file:///home/ayush/Desktop/NIDAR/simulation/custom_models/nidar_arena/meshes/tomar.dae), aligning the mesh properly with Gazebo's ground plane at $Z=0$.

### 1.5-Meter Height Restriction Enforcement
- **[exploration_planner.yaml](file:///home/ayush/Desktop/NIDAR/config/fuel/exploration_planner.yaml)**: Capped ceiling bound `box_max_z: 1.8`, safety clearance `d_min: 0.25m`, and max velocity `max_vel: 1.0m/s`.
- **[nidar_fuel.launch](file:///home/ayush/Desktop/NIDAR/launch/nidar_fuel.launch)**: Updated exploration bounding box Z limit to `1.8m`.
- **[fuel_to_mavros_bridge.py](file:///home/ayush/Desktop/NIDAR/scripts/fuel_to_mavros_bridge.py)**: Clamped B-spline trajectory setpoint Z targets to a maximum height of $1.5\text{m}$.

### Mission Rosbag Recorder
- **[record_mission.sh](file:///home/ayush/Desktop/NIDAR/scripts/record_mission.sh)**: Created executable rosbag logging script targeting `/Fast_LIO/odometry`, `/cloud_registered`, `/mavros/local_position/pose`, `/tf`, `/tf_static`, `/mavros/state`, `/mavros/setpoint_raw/local`, and `/exploration_node/frontier_num`.

---

## 2. Topic Verification & Telemetry Metrics

```bash
docker exec ros_workspace bash -c "source /opt/ros/noetic/setup.bash && rostopic hz /Fast_LIO/odometry /mavros/local_position/pose /mavros/imu/data"
```

| Topic Name | Streaming Rate | Status |
| :--- | :---: | :--- |
| `/Fast_LIO/odometry` | **10.01 Hz** | Active & Registered |
| `/mavros/local_position/pose` | **30.01 Hz** | Locked (EKF2 External Vision Fusion) |
| `/mavros/imu/data` | **50.00 Hz** | Active |

### Telemetry Lock & State Output (`rostopic echo`)

```yaml
/mavros/local_position/pose:
  position:
    x: -0.0202
    y: 0.0260
    z: 0.0203
  orientation:
    w: -0.9999

/mavros/state:
  connected: True
  armed: True
  guided: True
  mode: "AUTO.TAKEOFF"
```

---

## 3. Phase Completion Status

- [x] **Arena Orientation Fix**: `tomar.dae` mesh fixed to `Z_UP`.
- [x] **Phase 1 (Simulation & SLAM Bring-Up)**: Verified with FAST-LIO2 streaming @ 10Hz.
- [x] **Phase 2 (Localization & Rosbag)**: Verified EKF2 vision lock @ 30Hz; `record_mission.sh` deployed.
- [x] **Phase 3 (FUEL Autonomous Exploration)**: Parameters tuned for 1.5m height cap & 1m corridor navigation.
- [x] **Phase 4 (State Machine & Shortest Path RTH)**: `mission_manager.py` configured with A* shortest path search and auto-land sequence.
- [x] **Phase 5 (Perception Readiness)**: Ready for 2D map slicer and survivor marker integration.
