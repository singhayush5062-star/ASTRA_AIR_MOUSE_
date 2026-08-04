# NIDAR 2026 AIR MOUSE: Phase-Wise Execution Master Plan

**Current Status:** Phases 1 through 3 are successfully completed. The SLAM-to-Flight-Controller loop is closed (`relay_odometry.py` is actively feeding FAST-LIO2 data to PX4), and the custom Gazebo `nidar_competition.world` (15m x 15m maze, 1m corridors) is built and operational.

The following phases outline the transition from *Localization* to *Full Autonomy & Payload Integration*.

---

## Phase 4: Volumetric Mapping & Autonomous Exploration
**Objective:** Integrate the FUEL hierarchical planner to autonomously explore the maze without manual intervention or wall collisions.
**Primary Package:** `FUEL` (HKUST-Aerial-Robotics)

### Step 4.1: Connect FAST-LIO2 to FUEL's Mapping Engine (FIESTA)
* **Action:** Modify FUEL’s launch files to subscribe to your existing SLAM topics. 
* **Topic Remapping:**
    * Remap FUEL's expected odometry to `/Fast_LIO/odometry`.
    * Remap FUEL's expected point cloud to `/cloud_registered` (output from FAST-LIO2).
* **Validation:** Open RViz. You should see FUEL generating a 3D Voxel map and visualizing "Frontier" bounding boxes (the edges of unexplored space) inside your simulated maze.

### Step 4.2: Parameter Tuning for 1-Meter Corridors
* **Action:** Open FUEL’s `exploration_planner.yaml` and `bspline_opt.yaml` to override the default outdoor/large-scale settings.
* **Critical Modifications:**
    * **LiDAR FOV:** Change from `80.0` (RealSense default) to `360.0` (Velodyne/Livox).
    * **Safety Clearance (`d_min` / `clearance`):** Set to **0.25m** to prevent the drone from deadlocking in 1m passages.
    * **Kinodynamic Limits:** Cap max velocity ($v_{max}$) to **1.0 m/s** and acceleration to **1.0 m/s²**.
    * **Repulsive Penalty (`lambda_c`):** Increase this weight so the B-spline optimizer aggressively pushes the drone to the center of the corridor during 90-degree turns.

### Step 4.3: The MAVROS Offboard Control Bridge
* **Action:** FUEL computes smooth B-spline trajectories, but PX4 needs MAVLink setpoints.
* **Development:** Write a ROS node (e.g., `fuel_to_mavros_bridge.py`) that translates FUEL's trajectory position/velocity outputs into `/mavros/setpoint_raw/local` or `/mavros/setpoint_position/local` messages.
* **Validation:** Arm the drone, switch PX4 to `OFFBOARD` mode, and trigger FUEL. The drone must autonomously sweep the maze and stop only when fully explored.

---

## Phase 5: The Competition Payload (Perception & 2D Mapping)
**Objective:** Fulfill the NIDAR scoring rules by streaming a 2D map and tagging survivors.
**Primary Packages:** Custom YOLO ROS Node, Custom 2D Slicer Node

### Step 5.1: Live 2D Occupancy Grid Slicer
* **Action:** The Ground Control Station (GCS) operator needs a lightweight 2D map, not a dense 3D point cloud, to satisfy bandwidth and rule constraints.
* **Development:** Create a ROS node (`map_2d_slicer_node`) that subscribes to FUEL's/FIESTA's 3D voxel grid.
* **Logic:** Slice the map horizontally (e.g., between $Z = 0.3m$ and $Z = 1.8m$). Flatten this data and publish it as a standard ROS `nav_msgs/OccupancyGrid` at a throttled rate (5 Hz).

### Step 5.2: YOLOv8 Survivor Localizer
* **Action:** Integrate a TensorRT-accelerated YOLOv8/v11 object detection node subscribing to the downward/forward simulated camera (`/camera/color/image_raw`).
* **Logic:**
    1.  YOLO detects a "survivor" and outputs a 2D pixel bounding box.
    2.  Raycast the center of this box against the 3D LiDAR point cloud (or depth map) to find the real-world $(X, Y, Z)$ coordinate.
    3.  Convert $(X, Y)$ into the maze's grid coordinate system.
    4.  Publish a `visualization_msgs/Marker` (e.g., a colored dot/box) to overlay onto the 2D map.

---

## Phase 6: Mission State Machine & Auto-Exit
**Objective:** Automate the mission lifecycle to guarantee the drone returns to the entry point before the 30-minute timer expires.
**Primary Package:** Custom State Machine Node (e.g., `nidar_mission_commander`)

### Step 6.1: Build the Mission Controller
* **Action:** Create a master Python node utilizing `smach` (State Machine) or a robust loop architecture.
* **Lifecycle States:**
    * **[INIT]:** Drone takes off autonomously inside the 2ft x 2ft launch box. Records its starting position $(X_0, Y_0, Z_0)$.
    * **[EXPLORE]:** Triggers FUEL. Continuously monitors the active frontier cluster count (`N_cls`) from FUEL and the total count of tagged survivors from YOLO.
    * **[RETURN]:** Triggered automatically if `Survivors == 6` OR `N_cls == 0` (maze fully mapped). The node preempts FUEL's exploration, sends the saved $(X_0, Y_0, Z_0)$ coordinate as an absolute goal to the B-spline optimizer, and routes the drone back.
    * **[LAND]:** Once within 0.5m of $(X_0, Y_0)$, triggers the MAVROS auto-land command.

---

## Phase 7: Ground Control Station (GCS) & Hardware Transition
**Objective:** Move from the Docker simulator to physical Jetson hardware and establish the offline telemetry link.

### Step 7.1: Offline Telemetry Bridge (No Internet Rule)
* **Action:** Configure a standalone 5GHz Wi-Fi router (e.g., Ubiquiti). Connect the physical companion computer and the GCS operator laptop to this local subnet.
* **Throttled Stream:** Use `rosbridge_suite` or `image_transport/compressed` to stream *only* the compressed camera feed, the 2D Occupancy Grid, and the Survivor Markers to the GCS. 
* **Dashboard:** Set up Foxglove Studio or a custom RViz layout on the GCS laptop to display the live dashboard.

### Step 7.2: Hardware Porting & Extrinsic Calibration
* **Action:** Flash the Nvidia Jetson Orin/Xavier. Compile the workspace utilizing CUDA/TensorRT.
* **CPU Pinning:** Use `taskset` to pin heavy ROS nodes (like YOLO and FIESTA) to specific CPU cores. This prevents them from starving the MAVROS/PX4 offboard heartbeat (which would trigger an emergency failsafe).
* **Calibration:** Rigorously calibrate the physical transform (TF) between the Livox LiDAR, the Camera, and the IMU. (A 2-degree misalignment in real life will cause YOLO targets to project inside solid walls on your map).

### Step 7.3: Physical Mock-up Flights
* **Action:** Construct a physical test arena with cardboard/wood walls simulating a 1m corridor and a 2x2m room.
* **Validation:** Run the complete end-to-end test: Autonomous Takeoff -> Map Generation -> Target Detection -> Auto Return to Base -> Landing.