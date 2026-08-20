# ASTRA AIR MOUSE 2026: Autonomous GPS-Denied Exploration & Mapping

This repository contains the complete autonomous exploration, volumetric mapping, and flight simulation stack for the **NIDAR AirMouse** competition. It is designed for **zero-friction team deployment** using Docker, ROS Noetic, Gazebo Classic, FAST-LIO2, FUEL, and PX4 Autopilot SITL.

---

## System Prerequisites & Requirements

Before setting up the environment, ensure your host system meets the following software and hardware requirements:

### 1. Host Operating System
* **Recommended:** Ubuntu 20.04 LTS or Ubuntu 22.04 LTS (or any Linux distribution running an X11 display server).

### 2. Docker Engine & User Permissions
* **Docker Engine Version:** Docker Engine `v20.10.0` or higher (`v24.0+` recommended).
* **Non-Root Docker Privileges:** Ensure your user account is added to the `docker` group so container management scripts execute without requiring `sudo`:
  ```bash
  sudo usermod -aG docker $USER
  newgrp docker
  ```

### 3. Display Authorization & Graphical Forwarding (X11)
* **X11 Utilities:** Install `xhost` on the host to allow the Docker container to render Gazebo and RViz GUIs on your host display:
  ```bash
  sudo apt-get update && sudo apt-get install -y x11-xserver-utils
  ```
* **Graphics Drivers:** OpenGL 3.3+ hardware acceleration (NVIDIA proprietary drivers or Intel/AMD Mesa drivers).

### 4. Recommended Hardware Specifications
* **CPU:** Quad-core 2.5GHz+ processor (8+ threads recommended for concurrent SITL, SLAM, and FUEL planning loops).
* **RAM:** Minimum **8 GB** (16 GB recommended for parallel `catkin build` compilation).
* **Disk Storage:** **25 GB** free storage space (for Docker image, PX4 SITL build targets, and ROS workspace build files).

---

## Quickstart & Team Onboarding (3-Step Setup)

To ensure consistency across different hardware and operating systems, the entire compilation and simulation environment is dockerized. Any teammate can launch an exact replica of the verified baseline in three commands:

### Step 1: Clone the Repository
Clone this repository with all submodules to your host machine:
```bash
git clone --recurse-submodules https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
```

### Step 2: Launch the Automated Dev Container
Run the included Docker dev starter script. This will automatically set up host X11 socket authorization, build the ROS Noetic + Livox-SDK + FUEL container image if needed, and create/attach a persistent development container named `ros_workspace`:
```bash
chmod +x scripts/docker_dev_start.sh
./scripts/docker_dev_start.sh
```

> **Note on Container Lifecycle:** The dev container is persistent (`ros_workspace`). After exiting the container shell, you can re-attach at any time by running `./scripts/docker_dev_start.sh` or:
> ```bash
> docker start ros_workspace && docker exec -it ros_workspace /bin/bash
> ```

### Step 3: Build & Run Simulation
Inside the Docker interactive shell, execute the build scripts to compile the PX4 firmware and ROS packages:
```bash
# 1. Compile PX4 SITL target
./scripts/build_px4.sh

# 2. Compile custom ROS packages (FAST-LIO2, FUEL, Drivers)
cd catkin_ws && catkin build
source devel/setup.bash && cd ~/NIDAR

# 3. Launch automated flight & autonomous exploration (with GUI)
./scripts/test_takeoff.sh true
```

---

## Repository Structure

```directory
NIDAR/
├── docker/                        # Docker environment definition for reproducible onboarding
│   └── Dockerfile                 # Full Ubuntu 20.04 + ROS Noetic + MAVROS + Livox-SDK + FUEL toolchain
├── catkin_ws/                     # Active ROS Workspace (built inside Docker)
│   └── src/
│       ├── FAST_LIO/              # Real-time LiDAR-inertial SLAM odometry & cloud registration
│       ├── fuel/                  # Fast UAV Exploration (FUEL) hierarchical planner & FIESTA mapping
│       ├── livox_ros_driver/      # Livox ROS driver for 3D LiDAR sensors
│       └── velodyne_simulator/    # Velodyne VLP-16 gazebo plugins and descriptive meshes
├── config/                        # Extrinsic calibrations and RViz visualization profiles
│   ├── fast_lio/nidar_sim.yaml
│   └── nidar_lidar.rviz
├── scripts/                       # Autonomous mission managers and utility scripts
│   ├── docker_dev_start.sh        # Team onboarding container launcher
│   ├── flight_envelope_guard.py   # Trajectory-to-MAVROS offboard command bridge & safety envelope (v_max = 0.6 m/s)
│   ├── relay_odometry.py          # Relays SLAM estimates to PX4 vision pose fusion
│   └── test_takeoff.sh            # End-to-end automated mission execution script
└── simulation/                    # Simulation models & PX4 Autopilot environment
    ├── custom_models/iris_vlp16/  # Custom quadcopter integrated with VLP-16 LiDAR & IMU
    └── PX4-Autopilot-v1.14.3/     # PX4 firmware pre-configured for GPS-denied EKF external vision
```

---

## Key Technical Profiles
* **EKF External Vision Fusion:** PX4 ROMFS defaults are hard-coded (`EKF2_EV_CTRL = 11`) to enable robust GPS-denied state estimation driven by `/Fast_LIO/odometry`, with `EKF2_BARO_CTRL = 1` fusing barometric height as a cross-check.
* **Exploration Safety Bounds:** Maximum exploration velocity is capped at **$0.6\text{ m/s}$** (`max_vel` in `launch/nidar_fuel_upstream.launch`), with safety clearances optimized for narrow indoor corridors and warehouse obstacles.

---

## Troubleshooting & Known Issues

### `ERROR: cannot launch node of type [fast_lio/fastlio_mapping]: fast_lio`
* **Cause:** `catkin_ws/devel/setup.bash` was deleted or unlinked by a partial package build, leaving ROS package environment variables (`ROS_PACKAGE_PATH`) unaware of `fast_lio` or other nodes.
* **Fix:** Relink the merged devel space and regenerate setup files by running a full workspace build:
  ```bash
  cd catkin_ws && catkin build
  source devel/setup.bash
  cd ~/NIDAR
  ```
* **Auto-Healing:** `./scripts/setup_env.sh` automatically detects if `catkin_ws/devel/setup.bash` is missing and triggers `catkin build` to repair workspace linking.

### `Makefile:39: *** YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.` (during `./scripts/build_px4.sh`)
* **Cause:** `simulation/PX4-Autopilot-v1.14.3` is vendored into this repository as plain files rather than
  a git submodule, so a fresh clone has no `.git` there — but PX4's own build system requires one (both at
  its root, and inside a couple of nested paths its version-header generator checks, e.g.
  `src/modules/mavlink/mavlink`).
* **Auto-Healing:** `./scripts/build_px4.sh` automatically detects and bootstraps a minimal, self-contained
  local git repo in each place PX4's build tooling needs one — no action required, this runs automatically
  on every invocation and is a no-op once already bootstrapped.

### `Unable to register with master node [http://localhost:11311]: master may not be running yet.` (during `./scripts/test_takeoff.sh`)
* **Cause:** `roslaunch px4 mavros_posix_sitl.launch` died immediately on startup due to missing runtime prerequisites, causing `roscore` (ROS master) to shut down:
  1. **Missing GeographicLib Dataset:** MAVROS failed with `[FATAL] UAS: GeographicLib exception: File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm`. Since MAVROS is a required node, its crash kills `roslaunch` and `roscore`.
  2. **Unbuilt PX4 SITL Binary / Missing GStreamer Dev Libraries:** The PX4 SITL target (`build/px4_sitl_default/bin/px4`) was missing or failed CMake configuration due to missing GStreamer dev packages (`libgstreamer1.0-dev`).
* **Fix:** Install the missing GeographicLib geoid dataset and GStreamer dev libraries, then build PX4 SITL:
  ```bash
  # 1. Install missing GeographicLib geoid dataset
  sudo /usr/sbin/geographiclib-get-geoids egm96-5

  # 2. Install GStreamer development packages
  sudo apt-get update && sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

  # 3. Build PX4 SITL binary target
  ./scripts/build_px4.sh gazebo-classic_iris_vlp16
  ```


