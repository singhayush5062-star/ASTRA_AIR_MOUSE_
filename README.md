# ASTRA AIR MOUSE 2026: Autonomous GPS-Denied Exploration & Mapping

This repository contains the complete autonomous exploration, volumetric mapping, and flight simulation stack for the **NIDAR AirMouse** competition. It is designed for **zero-friction team deployment** using Docker, ROS Noetic, Gazebo Classic, FAST-LIO2, FUEL, and PX4 Autopilot SITL.

---

## Quickstart & Team Onboarding (3-Step Setup)

To ensure consistency across different hardware and operating systems, the entire compilation and simulation environment is dockerized. Any teammate can launch an exact replica of the verified baseline in three commands:

### Step 1: Clone the Repository
Clone this repository to your Ubuntu host machine:
```bash
git clone https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
```

### Step 2: Launch the Automated Dev Container
Run the included Docker dev starter script. This will automatically set up host X11 socket authorization, build the ROS Noetic + FUEL container image if needed, and drop you into an interactive bash shell with your workspace mounted live at `/home/developer/NIDAR`:
```bash
chmod +x scripts/docker_dev_start.sh
./scripts/docker_dev_start.sh
```

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
│   └── Dockerfile                 # Full Ubuntu 20.04 + ROS Noetic + MAVROS + FUEL toolchain
├── catkin_ws/                     # Active ROS Workspace (built inside Docker)
│   └── src/
│       ├── FAST_LIO/              # Real-time LiDAR-inertial SLAM odometry & cloud registration
│       ├── fuel/                  # Fast UAV Exploration (FUEL) hierarchical planner & FIESTA mapping
│       └── velodyne_simulator/    # Velodyne VLP-16 gazebo plugins and descriptive meshes
├── config/                        # Extrinsic calibrations and RViz visualization profiles
│   ├── fast_lio/nidar_sim.yaml
│   └── nidar_lidar.rviz
├── scripts/                       # Autonomous mission managers and utility scripts
│   ├── docker_dev_start.sh        # Team onboarding container launcher
│   ├── fuel_to_mavros_bridge.py   # Trajectory-to-MAVROS offboard command bridge (v_max = 1.0 m/s)
│   ├── relay_odometry.py          # Relays SLAM estimates to PX4 vision pose fusion
│   └── test_takeoff.sh            # End-to-end automated mission execution script
└── simulation/                    # Simulation models & PX4 Autopilot environment
    ├── custom_models/iris_vlp16/  # Custom quadcopter integrated with VLP-16 LiDAR & IMU
    └── PX4-Autopilot-v1.14.3/     # PX4 firmware pre-configured for GPS-denied EKF external vision
```

---

## Key Technical Profiles
* **EKF External Vision Fusion:** PX4 ROMFS defaults are hard-coded (`EKF2_EV_CTRL = 15`) to enable robust GPS-denied state estimation driven by `/Fast_LIO/odometry`.
* **Exploration Safety Bounds:** Maximum exploration velocity is capped at **$1.0\text{ m/s}$**, with safety clearances optimized for narrow indoor corridors and warehouse obstacles.
