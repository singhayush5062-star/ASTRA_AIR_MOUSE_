# NIDAR Project Progress Report

This document summarizes the development progress, integrated packages, and technical milestones achieved in the NIDAR (GPS-Denied Autonomous Drone) project up to Phase 3.

## Overview of Completed Phases

We have successfully built a fully functioning software-in-the-loop (SITL) simulation stack that allows the Iris drone to boot up in a GPS-denied environment, build a 3D LiDAR map, localize itself within that map, and execute an autonomous takeoff command purely using vision-based odometry.

### 1. Core Simulation Environment (Gazebo & PX4)
*   **Package**: `px4`, `gazebo_ros`, `mavros`
*   **Status**: Fully Integrated
*   **Details**: 
    *   Deployed the `iris_vlp16` quadrotor model, which equips the standard Iris drone with a Velodyne VLP-16 3D LiDAR sensor.
    *   Resolved X11 GUI forwarding issues to allow Gazebo and RViz to render correctly from within the Docker container to the host machine.
    *   Hard-coded PX4 Extended Kalman Filter (EKF2) parameters (`EKF2_EV_CTRL = 15`, `EKF2_GPS_CTRL = 0`, `EKF2_HGT_REF = 3`) into the `1023_gazebo-classic_iris_vlp16` airframe file. This guarantees the drone defaults to External Vision fusion on boot, bypassing fragile runtime MAVLink parameter adjustments.

### 2. SLAM Pipeline (FAST-LIO2)
*   **Package**: `fast_lio`
*   **Status**: Fully Integrated
*   **Details**:
    *   Successfully integrated the FAST-LIO2 package to consume the high-rate LiDAR (`/velodyne_points`) and IMU (`/mavros/imu/data`) streams.
    *   Fixed namespace issues in `nidar_mapping.launch` to ensure the `laserMapping` node correctly reads global parameters and initializes the LiDAR in Velodyne mode (`lidar_type = 2`).
    *   The system now successfully processes point clouds and generates a highly accurate local odometry estimate (`/Fast_LIO/odometry`).

### 3. Autonomy Bridge & Flight Control
*   **Package**: Custom Python scripts (`relay_odometry.py`, `test_takeoff.sh`)
*   **Status**: Fully Integrated & Verified
*   **Details**:
    *   **Odometry Relay**: Created `relay_odometry.py` to continuously transform and pipe the FAST-LIO odometry messages into the `/mavros/vision_pose/pose` topic at high frequency.
    *   **State Machine Initialization**: Built `test_takeoff.sh`, which automatically orchestrates the startup sequence: launching the world, waiting for MAVROS, starting SLAM, initiating the relay, monitoring the EKF for a local position lock, switching the flight controller to `AUTO.LOITER`, arming, and commanding a takeoff to a defined altitude.
    *   **Successful Verification**: The pipeline was verified to correctly arm and elevate the drone to 1.75 meters completely independent of GPS data.

### 4. Visualization & Debugging (RViz)
*   **Package**: `robot_state_publisher`, `rviz`
*   **Status**: Fully Integrated
*   **Details**:
    *   Authored a custom `iris_vlp16.urdf` file to link the physical drone meshes with the ROS TF tree.
    *   Configured `nidar_lidar.rviz` to render the `RobotModel` at the center of the SLAM map (`camera_init` frame), providing a live, 3D visualization of the drone flying through the mapped point cloud.

---

## Current Workspace Architecture

| Component | Technology / ROS Node | Topic / Interface |
| :--- | :--- | :--- |
| **Physics Engine** | Gazebo Classic | Simulated IMU, LiDAR, Motors |
| **Flight Controller** | PX4 SITL (v1.14.3) | UDP (port 14540) |
| **ROS Bridge** | MAVROS | `/mavros/state`, `/mavros/cmd/*` |
| **Mapping & Odometry** | FAST-LIO2 | Sub: `/velodyne_points`, `/mavros/imu/data`<br>Pub: `/Fast_LIO/odometry` |
| **Vision Fusion Bridge**| `relay_odometry.py` | Sub: `/Fast_LIO/odometry`<br>Pub: `/mavros/vision_pose/pose` |
| **Visualization** | RViz & `robot_state_publisher` | `/robot_description`, `/tf` |

---

## Next Steps (Phase 4: Autonomous Exploration)

I see that you have just created `nidar_competition.world` with a walled maze environment! This perfectly sets us up for the next phase. 

Our upcoming objectives are:
1.  **Map Conversion**: Integrate an `octomap_server` or voxel grid node to convert the dense 3D point cloud from FAST-LIO into a 2D/3D occupancy grid for collision checking.
2.  **Exploration Logic**: Integrate the **FUEL** (Fast UAV Exploration) package to subscribe to the occupancy grid, identify frontiers, and generate collision-free 3D trajectories through the maze.
3.  **Trajectory Execution**: Bridge FUEL's trajectory outputs into MAVROS position/velocity setpoints to physically drive the drone through the maze.
