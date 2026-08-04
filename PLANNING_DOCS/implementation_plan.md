# Plan to Integrate `nidar_world_arena.dae` & Unblock Autonomous Flight (Phase 4)

## Problem Overview
The project is currently blocked on two main fronts:
1. **Preflight Arming Error ("Yaw Estimate Error")**: PX4's EKF2 rejects vision-based odometry feedback (`/mavros/vision_pose/pose`) coming from FAST-LIO2 via `relay_odometry.py`, preventing the drone from arming and switching to `OFFBOARD` mode.
2. **Arena & Autonomy Integration (Phase 4)**: The new arena mesh (`nidar_world_arena.dae`) needs to be wrapped into a Gazebo `.world` file with proper collision/visual tags, and the **FUEL** (Fast UAV Exploration) framework needs to be cloned, compiled, and bridged to MAVROS setpoints.

---

## User Review Required

> [!IMPORTANT]
> **1. Gazebo World Mesh Collision Model**:
> Using complex `.dae` meshes directly for Gazebo physics collisions can be computationally expensive or cause raycast physics bugs with 3D LiDAR. We will configure `nidar_world_arena.dae` with `<uri>` model links in SDF/Gazebo world format, ensuring visual and collision geometries are properly specified.
>
> **2. PX4 EKF2 Heading Alignment**:
> Resolving the "Yaw estimate error" requires forcing PX4 EKF2 to accept External Vision (EV) heading and resetting internal yaw heading to match FAST-LIO2 initial frame alignment.

---

## Proposed Changes

### Component 1: Gazebo Arena Mesh Integration (`nidar_competition.world`)

#### [NEW] `simulation/custom_models/nidar_arena/model.config`
* Define Gazebo model metadata for the `nidar_world_arena.dae` arena.

#### [NEW] `simulation/custom_models/nidar_arena/model.sdf`
* Wrap `nidar_world_arena.dae` into an SDF model with `<visual>` and `<collision>` mesh geometries scaled correctly (1:1 scale).

#### [NEW] `nidar_competition.world`
* Create a Gazebo world file placing `nidar_arena` at origin `(0, 0, 0)`, along with sun lighting, physics solver settings, and spawning point for `iris_vlp16`.

---

### Component 2: PX4 EKF2 & Vision Odometry Stabilization

#### [MODIFY] `relay_odometry.py`
* Update coordinate frame transformations and covariance parameters published to `/mavros/vision_pose/pose`.
* Ensure high-rate publishing (>= 30 Hz) before PX4 attempts arming.
* Add timestamp header alignment using ROS sim time to prevent EKF2 latency rejection.

#### [MODIFY] `simulation/PX4-Autopilot-v1.14.3/ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16`
* Update EKF2 parameters for pure External Vision (EV) navigation:
  * `EKF2_EV_CTRL = 15` (Enable EV position, velocity, and yaw fusion)
  * `EKF2_GPS_CTRL = 0` (Disable GPS)
  * `EKF2_HGT_REF = 3` (Vision height reference)
  * `EKF2_EV_DELAY = 0` (Low latency simulation offset)
  * `EKF2_YAW_NOISE = 0.1`

---

### Component 3: Phase 4 Autonomous Exploration (FUEL Stack)

#### [NEW] `catkin_ws/src/fuel` (Submodule / Repository Clone)
* Clone and build `FUEL` planner and `FIESTA` voxel grid mapping package into `catkin_ws/src/`.

#### [NEW] `scripts/fuel_to_mavros_bridge.py`
* Create a ROS node to convert FUEL B-spline trajectory position and velocity outputs into MAVROS setpoints (`/mavros/setpoint_raw/local`).

#### [MODIFY] `config/fuel/exploration_planner.yaml`
* Tune safety clearance `d_min = 0.25m` for tight 1-meter corridor navigation inside `nidar_world_arena.dae`.
* Cap max velocity to `1.5 m/s` and max acceleration to `0.3 m/s²`.

---

## Verification Plan

### Automated Simulation Verification
1. **Launch World with New Arena**:
   ```bash
   docker exec -it ros_workspace bash -c "source /home/developer/NIDAR/scripts/setup_env.sh && roslaunch px4 mavros_posix_sitl.launch world:=/home/developer/NIDAR/nidar_competition.world vehicle:=iris_vlp16"
   ```
2. **Verify FAST-LIO & Odometry Relay**:
   ```bash
   rostopic echo -n 1 /mavros/vision_pose/pose
   rostopic echo -n 1 /mavros/state
   ```
   *Expectation*: `connected: True`, vision pose updating continuously.
3. **Arm and Takeoff Test**:
   ```bash
   rosrun mavros mavcmd cmd takeoff 0 0 0 0 1.5
   ```
   *Expectation*: Drone arms without EKF2 yaw errors and hovers at 1.5m in the new arena.

### Manual Verification
1. **RViz Visualization**: Inspect point cloud registration (`/cloud_registered`) against `nidar_world_arena.dae` bounds.
2. **Autonomous Exploration Sweep**: Trigger FUEL exploration node and observe collision-free B-spline trajectories inside the arena corridors.
