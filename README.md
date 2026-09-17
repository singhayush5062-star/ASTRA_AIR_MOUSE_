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

### 4. GPU Access for Onboard Inference (optional, for YOLO detection work)
* **NVIDIA Driver:** Host driver `>= 525` (CUDA 12.1 compatible; the image ships a self-contained CUDA 12.1 PyTorch build, no host CUDA toolkit needed).
* **NVIDIA Container Toolkit:** Required so Docker can pass the GPU through to the container:
  ```bash
  distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker
  ```
* **Verify:** `docker info | grep -i nvidia` should list the `nvidia` runtime. `scripts/docker_dev_start.sh` auto-detects this and adds `--gpus all` when launching the container; it falls back to CPU-only with a warning if not found.
* **Test trial:** once inside the container, run `python3 scripts/test_gpu_inference.py` to confirm the container sees the GPU and can run a real YOLOv8 forward pass on it.

### 5. Recommended Hardware Specifications
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

Regenerated 2026-09-04 from the actual runtime call graph of `scripts/test_takeoff.sh`, not from
directory listings. Every path below is loaded or executed by a live run; see
`PLANNING_DOCS/repo_cleanup_and_mind_map_2026-09-03.md` for the full trace.

```directory
NIDAR/
├── docker/Dockerfile              # Ubuntu 20.04 + ROS Noetic + MAVROS + Livox-SDK + FUEL toolchain
├── nidar_competition.world        # Gazebo world loaded by test_takeoff.sh (-> model://nidar_arena)
├── catkin_ws/src/
│   ├── FAST_LIO/                  # LiDAR-inertial SLAM; publishes /Fast_LIO/odometry (Z pinned to TFmini)
│   ├── fuel/                      # FUEL exploration planner (exploration_manager, plan_manage,
│   │                              #   bspline_opt, plan_env, path_searching, active_perception, ...)
│   ├── ikd-Tree/                  # Incremental k-d tree used by FAST-LIO
│   ├── livox_ros_driver/          # Livox LiDAR ROS driver
│   ├── velodyne_simulator/        # Gazebo VLP-16 plugin + meshes
│   └── PX4-Autopilot/             # NOT firmware: 12-file shim carrying velodyne_vlp16 meshes for rospack
├── config/
│   ├── fast_lio/nidar_sim.yaml    # FAST-LIO tuning
│   ├── iris_vlp16.urdf            # robot_description for TF
│   ├── nidar_lidar.rviz           # RViz profile (only loaded when RVIZ=1)
│   └── flight_envelope_guard.yaml # Safety envelope: XY bounds, Z band, range/vision watchdogs
├── launch/
│   ├── fast_lio/nidar_mapping.launch
│   └── nidar_fuel_upstream.launch # FUEL stack + traj_server + waypoint_generator
├── scripts/                       # 8 live scripts, all reachable from test_takeoff.sh
│   ├── test_takeoff.sh            # End-to-end mission entry point (GUI arg; RVIZ=1 for RViz)
│   ├── setup_env.sh               # ROS/Gazebo/PX4 env + model & plugin paths
│   ├── relay_odometry.py          # /Fast_LIO/odometry -> /mavros/vision_pose/pose
│   ├── flight_envelope_guard.py   # /planning/pos_cmd -> MAVROS setpoints + safety envelope
│   ├── cpu_repin_loop.sh          # Re-pins worker threads spawned after initial affinity set
│   ├── mission_telemetry_logger.py
│   ├── docker_dev_start.sh
│   └── build_px4.sh
├── simulation/
│   ├── custom_models/
│   │   ├── arina_nidar/           # ACTIVE arena mesh loaded by nidar_competition.world
│   │   ├── launch_pad/            # Spawn + return pad at world (0, -9.5)
│   │   ├── iris_vlp16/            # Legacy airframe, superseded by x500_vlp16; not spawned
│   │   ├── tfmini_lidar/          # Downward TFmini rangefinder (fixed-altitude 2D flight)
│   │   ├── velodyne_vlp16/
│   │   └── nidar_arena/           # Previous arena, kept for regression runs
│   └── PX4-Autopilot-v1.14.3/     # Vendored PX4 firmware (96% of tracked files; see cleanup plan Sec 4.4)
│       └── .../models/x500_vlp16/x500_vlp16.sdf   # <- the model Gazebo ACTUALLY spawns
└── PLANNING_DOCS/                 # Active engineering plans; archive/ holds resolved ones
```

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

