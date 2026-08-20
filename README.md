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

## End-to-End Build & Execution Workflow

Follow these step-by-step instructions to clone, build, compile, troubleshoot, and run the simulation stack from scratch.

### Step 1: Clone Repository & Verify Workspace (Host)
Execute the following commands on your host Linux terminal:

```bash
cd ~
mkdir -p ~/NIDAR_TEST
cd ~/NIDAR_TEST

# Clone repository with all submodules
git clone --recurse-submodules https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_

# Verify repository status, active branch, and submodules
git status
git branch --show-current
git submodule status
find . -maxdepth 2 -type f | sort | head -100
```

### Step 2: Build & Run Docker Container (Host)
Build the clean Docker development image and launch an interactive container with full X11 display forwarding and host networking privileges:

```bash
# 1. Build the Docker image
docker build \
    -t astra-air-mouse-test1:clean \
    -f docker/Dockerfile \
    docker

# 2. Verify image creation
docker images | grep -E 'astra-air-mouse-test1'

# 3. Grant X11 display access permissions to Docker containers
xhost +local:docker

# 4. Launch interactive Docker container
docker run -it --rm \
    --name astra_air_mouse_test1 \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e LIBGL_ALWAYS_SOFTWARE=0 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$HOME/NIDAR_TEST/ASTRA_AIR_MOUSE_:/home/developer/NIDAR" \
    -w /home/developer/NIDAR \
    astra-air-mouse-test1:clean \
    /bin/bash
```

---

### Step 3: Compile ROS Workspace & PX4 SITL (Inside Container)

Once inside the interactive Docker container shell (`/home/developer/NIDAR`), inspect the directory structure and follow the specific build locations:

```bash
cd /home/developer/NIDAR

# Inspect directory structure
ls
ls scripts
ls launch
ls catkin_ws/src
```

#### A. Build ROS Catkin Workspace
> **Build Location:** MUST be executed inside `/home/developer/NIDAR/catkin_ws`

```bash
cd /home/developer/NIDAR/catkin_ws

# Clean previous build artifacts and compile custom ROS packages (FAST-LIO2, FUEL, Drivers)
catkin clean -y 2>/dev/null || true
catkin build --no-status

# Source the compiled workspace setup bash file
source devel/setup.bash
```

#### B. Build PX4 SITL Firmware Target
> **Build Location:** MUST be executed inside `/home/developer/NIDAR` (Project Root)

```bash
cd /home/developer/NIDAR

# Ensure script permissions and compile PX4 SITL for gazebo-classic_iris_vlp16
chmod +x scripts/build_px4.sh
./scripts/build_px4.sh gazebo-classic_iris_vlp16
```

---

### Step 4: Final Testing & Execution

After completing compilation of both `catkin_ws` and PX4 SITL, launch the full end-to-end automated mission execution script with GUI:

```bash
cd /home/developer/NIDAR

# Execute autonomous takeoff, SLAM mapping, and FUEL exploration test (with GUI visualization)
./scripts/test_takeoff.sh true
```

---

## Repository Structure

```directory
NIDAR/
├── docker/                        # Docker environment definition for reproducible onboarding
│   └── Dockerfile                 # Full Ubuntu 20.04 + ROS Noetic + MAVROS + Livox-SDK + FUEL toolchain
├── catkin_ws/                     # Active ROS Workspace (built inside /home/developer/NIDAR/catkin_ws)
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
│   ├── build_px4.sh               # PX4 SITL firmware build script (run from /home/developer/NIDAR)
│   ├── flight_envelope_guard.py   # Trajectory-to-MAVROS offboard command bridge & safety envelope (v_max = 0.6 m/s)
│   ├── relay_odometry.py          # Relays SLAM estimates to PX4 vision pose fusion
│   └── test_takeoff.sh            # End-to-end automated mission execution script
└── simulation/                    # Simulation models & PX4 Autopilot environment
    ├── custom_models/iris_vlp16/  # Custom quadcopter integrated with VLP-16 LiDAR & IMU
    └── PX4-Autopilot-v1.14.3/     # PX4 firmware pre-configured for GPS-denied EKF external vision
```

---

## Technical Features & Configuration

* **EKF External Vision Fusion:** PX4 ROMFS defaults are pre-configured (`EKF2_EV_CTRL = 11`) for robust GPS-denied state estimation driven by `/Fast_LIO/odometry`, with `EKF2_BARO_CTRL = 1` fusing barometric height as a cross-check.
* **Exploration Safety Bounds:** Maximum exploration velocity is capped at **$0.6\text{ m/s}$** (`max_vel` in `launch/nidar_fuel_upstream.launch`), with safety clearances optimized for narrow indoor corridors and warehouse obstacles.

---

## Troubleshooting & Known Issues

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

  # 3. Build PX4 SITL binary target (from /home/developer/NIDAR)
  ./scripts/build_px4.sh gazebo-classic_iris_vlp16
  ```

### `ERROR: cannot launch node of type [fast_lio/fastlio_mapping]: fast_lio`
* **Cause:** `catkin_ws/devel/setup.bash` was unlinked or not sourced, leaving ROS package environment variables (`ROS_PACKAGE_PATH`) unaware of `fast_lio`.
* **Fix:** Rebuild the catkin workspace inside `catkin_ws` and source the setup script:
  ```bash
  cd /home/developer/NIDAR/catkin_ws
  catkin build
  source devel/setup.bash
  cd /home/developer/NIDAR
  ```

### `Makefile:39: *** YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.` (during `./scripts/build_px4.sh`)
* **Cause:** `simulation/PX4-Autopilot-v1.14.3` has no `.git` directory in a fresh unzipped state, but PX4's build system requires git tags for version generation.
* **Auto-Healing:** `./scripts/build_px4.sh` automatically detects and bootstraps a minimal, self-contained local git repo in each required directory before triggering `make px4_sitl`.
