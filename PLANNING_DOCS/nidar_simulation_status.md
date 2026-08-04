# NIDAR Drone Simulation Project Audit and Task List

This document provides a comprehensive summary of the current state of the NIDAR drone simulation project, details the findings from our diagnostic session, and outlines a structured task list for subsequent development phases.

## Status Summary

The NIDAR drone simulation environment is built on ROS Noetic, Gazebo Classic, and PX4 Autopilot v1.14.3 inside the `ros_workspace` Docker container. 

### Completed Tasks
- **Environment and Repository Layout**: Sourced/setup of the workspace structure. The custom `catkin_ws` contains `FAST_LIO`, `velodyne_simulator`, and `livox_ros_driver`.
- **PX4 Target Build**: The target SITL firmware (`gazebo-classic_iris_vlp16`) is built and ready in the container.
- **ROS Workspace Build**: The `catkin build` compiles successfully and links all packages correctly.
- **Drone Model Integration**: The custom model `iris_vlp16` incorporating the quadcopter, an IMU, and a Velodyne VLP-16 LiDAR sensor is properly integrated and spawns in Gazebo.
- **LiDAR Sensor Streaming**: Velodyne LiDAR plugin loads and streams data to the `/velodyne_points` topic at 10Hz.
- **IMU Sensor Streaming**: The integrated IMU streams data to `/mavros/imu/data` at 50Hz.

### Key Diagnostics: MAVROS Connection Issue Resolved
During our diagnostic run, we found that:
1. **The Issue**: Sourcing `scripts/run_sim.sh` spawned the simulation successfully, but MAVROS was stuck reporting `connected: False` in `/mavros/state`.
2. **Root Cause**: Gazebo failed to load the PX4 plugins (`libgazebo_mavlink_interface.so`, `libgazebo_motor_model.so`, etc.) because the build directory `/home/developer/NIDAR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic` was missing from `GAZEBO_PLUGIN_PATH` and `LD_LIBRARY_PATH` inside the container's environment setup (`scripts/setup_env.sh`).
3. **The Block**: Without the MAVLink interface plugin, Gazebo did not connect to PX4's simulation TCP port `4560`. PX4 stayed stuck waiting for the simulator, preventing telemetry forwarding to MAVROS on UDP port `14540`.
4. **Resolution**: Manually appending `/home/developer/NIDAR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic` to both `GAZEBO_PLUGIN_PATH` and `LD_LIBRARY_PATH` fully resolves this. MAVROS immediately connects (`connected: True`), enters the `AUTO.LOITER` flight mode, and streams telemetry successfully.

---

## Detailed Task List

Here is the proposed checklist for the upcoming steps. No code was modified during this audit phase.

### Phase 1: Environment & Setup Fixes
- [ ] **Update `setup_env.sh` inside the container**:
  Add the PX4 build paths to the environment setup script:
  ```bash
  export GAZEBO_PLUGIN_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${GAZEBO_PLUGIN_PATH}"
  export LD_LIBRARY_PATH="$ROOT_DIR/simulation/PX4-Autopilot-v1.14.3/build/px4_sitl_default/build_gazebo-classic:${LD_LIBRARY_PATH}"
  ```
- [ ] **Verify `run_sim.sh` runs cleanly**:
  Confirm that launching `scripts/run_sim.sh gui:=false` automatically connects MAVROS and starts streaming all sensor data without manual exports.

### Phase 2: Autonomous Navigation and Takeoff Validation
- [ ] **Update Takeoff Test Script (`scripts/test_takeoff.sh`)**:
  - Implement a clean shutdown sequence using `trap` in bash to kill spawned processes (rosmaster, gzserver, mavros, px4) on script exit.
  - Review timing and add checking loops for `/mavros/state` connection state before attempting commands.
- [ ] **Verify Takeoff Commands**:
  - Arm the drone via `rosrun mavros mavsafety arm`.
  - Trigger takeoff command `rosrun mavros mavcmd takeoff 0 0 0 0 3.0` and verify the drone ascends in Gazebo (checking altitude topic `/mavros/global_position/rel_alt` or `/mavros/local_position/pose`).
  - Land the drone via `rosrun mavros mavcmd land 0 0 0 0`.

### Phase 3: FAST-LIO Mapping and Coordinate Frames Verification
- [ ] **Verify Transform Tree (TF)**:
  - Check the TF tree to ensure the transformation between `/base_link`, `/velodyne_link` (LiDAR), and mapping coordinate frames is active and correct.
- [ ] **Launch FAST-LIO Mapping**:
  - Source the environment and launch FAST-LIO using `launch/fast_lio/nidar_mapping.launch`.
  - Monitor mapping output in the terminal to verify state estimation.
- [ ] **Validate Mapping Extrinsics**:
  - Review extrinsic parameters (rotation/translation from LiDAR to IMU) configured in `config/fast_lio/nidar_sim.yaml` to ensure they match the sensor positions in `iris_vlp16.sdf`.

### Phase 4: Full End-to-End Simulation & RViz Visualization
- [ ] **Ensure X11 Display Authorization Config is Robust**:
  - Fix host-container X11 permissions (`xhost +local:docker`) to allow Gazebo GUI and RViz display rendering on the host.
- [ ] **Launch Simulation with RViz Visualization**:
  - Run the takeoff test and FAST-LIO mapping with the Gazebo GUI enabled.
  - Observe mapping registration and point cloud rendering in RViz to verify that the coordinate frame transformation is stable and mapping is accurate.
