# Hardware Deployment, Installation & Tuning Guide

Real NIDAR AirMouse drone: **Jetson Orin Nano Super** (onboard computer) + **PX4** flight controller
+ **Livox Mid-360** LiDAR + **TFmini Plus** rangefinder + camera, flown from the **GCS** on a separate
Linux laptop over the **T12** RC/data link, a **SiK** radio, **USB**, or **Wi-Fi**.

This guide is the companion of the bring-up checklist in [`hardware/README.md`](README.md): this file
says *how to install and configure*, the checklist says *what must pass before the next step*.

> **The frame is not built yet.** Every placement value below is a placeholder in one file,
> [`catkin_ws/src/nidar_config/config/hardware.yaml`](../catkin_ws/src/nidar_config/config/hardware.yaml).
> When the frame exists: measure (§5.2), edit that file, run the generator (§5.4), reload the PX4
> parameters (§7.2). Nothing else needs editing for a different sensor placement.

## Contents

1. [What runs where](#1-what-runs-where)
2. [Wiring](#2-wiring)
3. [Jetson Orin Nano setup](#3-jetson-orin-nano-setup)
4. [Build](#4-build)
5. [Sensor placement and `hardware.yaml`](#5-sensor-placement-and-hardwareyaml)
6. [Sensors](#6-sensors)
7. [PX4 flight controller](#7-px4-flight-controller)
8. [GCS laptop](#8-gcs-laptop)
9. [Bring-up and first flights](#9-bring-up-and-first-flights)
10. [Tuning guide](#10-tuning-guide)
11. [Troubleshooting](#11-troubleshooting)
12. [File map](#12-file-map)

---

## 1. What runs where

```
 ┌──────────────────────────── DRONE ─────────────────────────────┐        ┌──────── GCS LAPTOP (Linux) ────────┐
 │                                                                │        │                                    │
 │  Mid-360 ──Ethernet──┐                                         │        │  start_gcs.sh  (FastAPI :8000)     │
 │  camera ──USB/CSI────┤  JETSON ORIN NANO (Docker: nidar-onboard)│  Wi-Fi │   ├─ MAVLink link ◄── UDP 14550 ◄──┼── MAVROS gcs_url
 │  (TFmini) ─optional──┤   hw_bringup.sh                         │◄──────►│   │                  serial ◄─────┼── T12 / SiK ground
 │                      │   MAVROS, livox driver2 + livox_bridge  │        │   └─ ROS link (bridge) ◄── Jetson   │
 │                      │   FAST-LIO2 → relay → PX4 EKF2          │        │       ROS master: map, survivors,  │
 │                      │   guard, FUEL, mission manager,         │        │       mission, camera, health,     │
 │                      │   detector, 2D map, mission_commander   │        │       TAKEOFF/RTL/LAND             │
 │                      └──UART (TELEM2, 921600)──┐               │        │  Browser: http://localhost:8000    │
 │  TFmini ──UART (GPS2)──────────► PX4 FC ◄──────┘               │        │           → HARDWARE               │
 │  T12 air unit ──TELEM1 (MAVLink) + SBUS (RC)──► PX4 FC          │  T12   │  T12 ground unit ── USB serial    │
 │                                                                │◄──────►│   (+ video, if it outputs any)     │
 └────────────────────────────────────────────────────────────────┘        └────────────────────────────────────┘
```

* **The whole autonomy stack runs on the Jetson.** It is the simulation stack unchanged
  (FAST-LIO2, FUEL, flight envelope guard, mission manager, survivor detector, 2D map); only the
  sensor sources differ (`nidar_hardware` package instead of Gazebo).
* **The Jetson never arms on its own.** `hw_bringup.sh` brings everything up and reports READY;
  the operator presses **TAKEOFF** on the GCS. The onboard `mission_commander` checks the stack,
  then runs the same sequence `test_takeoff.sh` runs in SITL (AUTO.TAKEOFF → arm → OFFBOARD), and
  the mission manager flies the mission (climb → door → explore → return → land on the pad).
* **The GCS** (separate Linux laptop) shows telemetry, the live 2D map, survivors, mission phase,
  health and video, and sends **TAKEOFF / RTL / LAND / EMERGENCY ABORT**.

| GCS gets ... | over MAVLink only (T12 / SiK / USB) | + ROS link (Wi-Fi to the Jetson) |
|---|---|---|
| mode, armed, attitude, battery, position | ✓ (position converted to the arena frame) | ✓ (exact, from tf) |
| mission phase, survivors | ✓ via onboard STATUSTEXT (needs §7.3 forwarding) | ✓ |
| live 2D map, onboard camera with detections, per-subsystem health | — | ✓ |
| LAND, ABORT | ✓ (straight to PX4) | ✓ |
| TAKEOFF, RTL | ✓ only with MAVLink forwarding (§7.3) | ✓ |

## 2. Wiring

| From | To | Notes |
|---|---|---|
| FC **TELEM2** (TX, RX, GND) | Jetson 40-pin header **pin 8** (UART TX), **pin 10** (UART RX), **pin 6** (GND) | 3.3 V logic on both sides. TX→RX, RX→TX. **Do not** connect the 5 V pin. 921600 baud. Device: `/dev/ttyTHS1` on JetPack 6 (`ls -l /dev/ttyTHS*`; some JetPack 5 images name it `ttyTHS0`). |
| FC **TELEM1** | **T12 air unit** data port (or SiK air radio) | MAVLink to the GCS. Baud must equal the air unit's data-link setting (default here 57600 → `fcu.telem1_baud`). |
| FC **RC IN** | T12 air unit SBUS/PPM out | The T12 transmitter is the safety pilot (§9). |
| **TFmini Plus** | FC **GPS2** (or TELEM3/4) UART | 5 V supply, 3.3 V TTL. TX→RX crossed. Or to the Jetson via a USB-UART (`rangefinder.source: serial`). |
| **Mid-360** | Jetson **Ethernet** | The Mid-360 cable splits into power (9–27 V DC, ~6.5 W) and RJ45. Power it from the battery through a regulator, not from the Jetson. |
| Camera | Jetson USB 3 (or CSI / Ethernet IP camera) | Detection needs the frames **on the Jetson**. |
| Jetson power | Battery → regulator → Jetson DC jack | Use the input range in NVIDIA's carrier-board spec (the kit ships with a 19 V supply). Never the raw pack. |
| Bench option | FC **USB** → Jetson USB | `/dev/ttyACM0`; set `fcu.url: /dev/ttyACM0:115200`. Fine for the bench; prefer TELEM2 in flight. |

**Video to the GCS.** Two independent paths, use either or both:
* *Onboard stream over Wi-Fi* (default): the Jetson sends a throttled JPEG of the detection camera
  with the detector's boxes drawn by the GCS. Needs the ROS link (§8.3).
* *T12 video downlink*: whatever camera feeds the T12 air unit appears at the T12 ground unit. If it
  reaches the laptop as a USB video device or through an HDMI capture dongle, enter `/dev/videoN` in
  the Hardware page camera panel (**CONFIGURE STREAM**); if it is an RTSP stream, enter its URL.
  A camera wired only to the T12 air unit is **not** seen by the Jetson's detector.

## 3. Jetson Orin Nano setup

### 3.1 JetPack and power mode

The Orin Nano **Super** needs JetPack 6.x (Ubuntu 22.04). ROS Noetic needs Ubuntu 20.04, so the
flight stack runs in a container (§3.3) — nothing ROS-related is installed on the host.

```bash
cat /etc/nv_tegra_release        # L4T version (R36.x = JetPack 6, R35.x = JetPack 5)
sudo nvpmodel -q                 # current power mode
# Pick the highest mode (MAXN SUPER on the Super kit) from the desktop power menu, or
# sudo nvpmodel -m <id> with the id listed in /etc/nvpmodel.conf, then:
sudo jetson_clocks               # lock clocks at max (repeat after each boot, or make a service)
sudo usermod -aG docker,dialout $USER   # log out and back in
```

### 3.2 Network

Two networks on the Jetson:

| Interface | Purpose | Address (`hardware.yaml`) |
|---|---|---|
| Ethernet | Mid-360 only | static `lidar.host_ip` (192.168.1.50/24); the LiDAR is `lidar.lidar_ip` |
| Wi-Fi / T12 network | GCS laptop | `network.jetson_ip`; the laptop is `network.gcs_ip` |

```bash
sudo nmcli con add type ethernet ifname eth0 con-name livox \
     ipv4.method manual ipv4.addresses 192.168.1.50/24
sudo nmcli con up livox
ping 192.168.1.112               # the Mid-360 (factory IP 192.168.1.1XX, XX = last 2 serial digits)
```

Give the Jetson and the laptop **fixed** addresses on the GCS network (DHCP reservation or static), and
put them in `network.jetson_ip` / `network.gcs_ip` (MAVROS sends MAVLink to `gcs_ip`; ROS nodes
advertise `jetson_ip`). `scripts/jetson_onboard.sh check` verifies all of this.

### 3.3 Onboard container

```bash
git clone --recurse-submodules https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
git checkout hardware_deployment_test    # until the hardware PR is merged
scripts/jetson_onboard.sh check      # JetPack, docker, serial devices, cameras, IPs, LiDAR ping
scripts/jetson_onboard.sh image      # docker/Dockerfile.jetson -> image nidar-onboard (~30-60 min)
scripts/jetson_onboard.sh start      # persistent container nidar_onboard (--net=host --privileged, /dev)
scripts/jetson_onboard.sh build      # catkin build (profile "jetson": no Gazebo packages)
```

The image (`docker/Dockerfile.jetson`) contains ROS Noetic, MAVROS (+ geoid data), the FUEL/FAST-LIO
build dependencies, Livox-SDK (v1, for the message package FAST-LIO uses), **Livox-SDK2 +
livox_ros_driver2** (the Mid-360 driver, in `/opt/livox_ws`), and ultralytics + NCNN for the detector.
The repository is bind-mounted at `/home/developer/NIDAR`, so `git pull` on the host + `build` updates it.

### 3.4 GPU detector (optional)

The default image (`ros:noetic-ros-base-focal`) runs on any JetPack but cannot use the GPU: CUDA
libraries are mounted from the host, and JetPack 6's are built for Ubuntu 22.04. The detector then
runs on the CPU with an **NCNN** model (`perception.model` = a `*_ncnn_model` directory), which keeps
up at the configured `detect_hz` on the Orin's six cores.

For GPU inference flash **JetPack 5.1.x** (Ubuntu 20.04; this gives up the Super power mode) and build:

```bash
BASE_IMAGE=nvcr.io/nvidia/l4t-base:r35.4.1 \
TORCH_WHEEL=https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl \
scripts/jetson_onboard.sh image
```

then set `perception.model` to the `.pt` and `perception.device: cuda`.

## 4. Build

### 4.1 Workspace

`scripts/jetson_onboard.sh build` runs, inside the container:

```bash
catkin config --profile jetson --extend /opt/livox_ws/devel \
    --skiplist velodyne_gazebo_plugins velodyne_simulator velodyne_description nidar_sim nidar_gcs
catkin build --profile jetson
python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py --check
```

The `jetson` profile is local to the Jetson (git-ignored); the laptop keeps the `default` profile.

### 4.2 Livox driver

`livox_ros_driver2` is built into the image (`/opt/livox_ws`) and the workspace **extends** it, so
`rospack find livox_ros_driver2` works after `source catkin_ws/devel/setup.bash`. If it does not, the
workspace was built before the extend was configured: `catkin clean --profile jetson -y` and build again.

FAST-LIO is unchanged: it reads `livox_ros_driver/CustomMsg`. `nidar_hardware/livox_bridge.py`
republishes the driver2 scans under that type (the two definitions are byte-identical) and puts LiDAR
time onto ROS time (§6.1). Topics: `/livox/lidar`, `/livox/imu` (driver) → `/nidar/livox/lidar`,
`/nidar/livox/imu` (FAST-LIO).

## 5. Sensor placement and `hardware.yaml`

### 5.1 Frames

```
world ──(launch pad pose, mission_config.yaml)── map ── camera_init ──(FAST-LIO)── body ── base_link ─┬─ livox_frame
                                                                                                    ├─ tfmini_link
                                                                                                    └─ camera_link
```

* **`base_link` = the flight controller's IMU.** Every mount in `hardware.yaml` is measured from the
  centre of the FC, in **ROS axes: +x forward, +y LEFT, +z UP**, metres; angles in degrees.
* `body` is the IMU FAST-LIO integrates: the Mid-360's internal IMU (`lidar.imu_source: livox`) or
  the FC's (`fcu`). `hw_static_tf.py` publishes `body → base_link` and the three sensor frames.
* `camera_init` is planted where the drone stands when FAST-LIO starts. **Always start the stack with
  the drone on the launch pad centre, nose toward the arena door** (mission_config.yaml
  `launch_pad.spawn_yaw`): that is what places the map, the grid and the survivors correctly in the arena.

### 5.2 Measuring each mount

| Sensor | Origin to measure to | Rules |
|---|---|---|
| Mid-360 (`lidar.mount`) | centre of the flat bottom of the LiDAR (the Livox frame origin; the internal IMU offset is added automatically) | With `imu_source: livox`: **upright, cable to the back, square to the body: roll = pitch = yaw = 0** (the generator refuses anything else). Put it straight above the FC (x = y = 0) so the vision lever arm is zero. Mount it high enough that the props are out of its field of view, or raise `lidar.blind` past the farthest visible prop tip. |
| TFmini (`rangefinder.mount`) | centre of the lens face | `z` is **negative** (below the FC). Pointing straight down, with nothing (legs, battery strap) in its 3.6° beam. |
| Camera (`camera.mount`) | optical centre (lens) | `pitch` **positive = tilted down** (ROS convention). Measure the tilt with a phone inclinometer on the camera body. |

Typical accuracy needed: ±5 mm for the TFmini z (it *is* the altitude), ±1 cm / ±1° for the others.

### 5.3 Parameter reference

| Key | Default | What it does / when to change |
|---|---|---|
| `network.jetson_ip` | 192.168.144.50 | Jetson address the GCS reaches; ROS nodes advertise it. |
| `network.gcs_ip` | 192.168.144.100 | Laptop address; used in `fcu.gcs_url`. |
| `fcu.url` | `/dev/ttyTHS1:921600` | MAVROS → PX4. Baud = `fcu.telem2_baud` = PX4 `SER_TEL2_BAUD`. |
| `fcu.telem2_baud` | 921600 | → `SER_TEL2_BAUD`. Lower (460800) if the wires are long and you see MAVLink errors. |
| `fcu.gcs_url` | `udp://@<gcs_ip>:14550` | MAVROS forwards all FC MAVLink to the laptop over Wi-Fi. `""` = off. |
| `fcu.telem1_baud` | 57600 | → `SER_TEL1_BAUD`; must match the T12 air unit / SiK radio. |
| `fcu.mavlink_forwarding` | true | → `MAV_0_FORWARD`/`MAV_1_FORWARD`. Needed for TAKEOFF/RTL and mission reports over the radio. Costs ~1.5 kB/s of radio bandwidth (§7.3). |
| `lidar.host_ip` / `lidar.lidar_ip` | 192.168.1.50 / .112 | Written into the Livox driver config. |
| `lidar.imu_source` | livox | `livox` (Mid-360 IMU, hardware-synced; mount must be square) or `fcu` (any mount angle; needs `time_sync: ptp`). |
| `lidar.mount` | (0, 0, 0.12, 0, 0, 0) | §5.2. |
| `lidar.blind` | 0.30 m | Points closer than this are dropped (props/frame). Too large indoors → "No Effective Points" near walls. |
| `lidar.det_range` | 40 m | FAST-LIO's map radius. |
| `lidar.time_sync` | restamp | `restamp` (no setup) or `ptp` (§6.1). |
| `rangefinder.source` | fcu | `fcu` (wired to PX4, read through MAVROS) or `serial` (wired to the Jetson). |
| `rangefinder.px4_port` | GPS2 | → `SENS_TFMINI_CFG`. |
| `rangefinder.mavros_topic` | `/mavros/distance_sensor/hrlv_ez4_pub` | MAVROS's topic for distance sensor id 0 (PX4's first rangefinder). |
| `rangefinder.mount` | (0, 0, -0.06) | **The altitude datum** → FAST-LIO `tfmini_mount_offset`, PX4 `EKF2_RNG_POS_*`. |
| `rangefinder.range_min/max`, `min_strength` | 0.1 / 12 / 100 | Readings outside are dropped (a silent sensor trips the guard's range watchdog instead of faking an altitude). |
| `camera.source` | `/dev/video0` | `/dev/videoN`, `rtsp://…`, or `gst:<pipeline>` (CSI). |
| `camera.width/height/fps` | 1280×720@30 | Requested capture. |
| `camera.publish_fps` | 10 | `/camera/image_raw` rate (the detector samples it at `perception.detect_hz`). |
| `camera.calibration_file` | "" | ost.yaml from §6.4. Empty = intrinsics from `horizontal_fov_deg`. |
| `camera.mount`, `rotate_180` | §5.2 | `rotate_180: true` if mounted upside down. |
| `camera.gcs_stream` | 5 fps, 640 px, q60 | Onboard video to the GCS (~1–1.5 Mbit/s). |
| `perception.model` | V3 NCNN | Detector weights (§6.5). |
| `perception.detect_hz` | 3 | Inference rate. Raise only if the latency allows (GCS camera HUD). |
| `perception.confidence_threshold` | 0.55 | Lower = more detections and more false ones. |
| `perception.target_height_m` | 0.35 | Height of a survivor's body centre; the pixel ray is intersected with this plane. |
| `commander.min_battery_pct` | 60 | TAKEOFF refused below. |
| `commander.auto_rtl_battery_pct` | 30 | In flight below this: one mission RTL. |
| `commander.offboard_delay_s` | 1.0 | AUTO.TAKEOFF → arm → wait → OFFBOARD (as in SITL). |
| `px4.ev_delay_ms` | 60 | → `EKF2_EV_DELAY` (§10.2). |
| `px4.mpc_thr_hover` | 0.45 | → `MPC_THR_HOVER` (initial guess; PX4 learns it in flight). |
| `px4.max_climb_speed`, `takeoff_speed`, `land_speed`, `max_xy_speed` | 1.0, 0.8, 0.5, 1.0 | → `MPC_*` limits. The planner flies at `mission_config.yaml flight.max_vel` (0.6). |

The **arena and mission** (arena bounds, launch pad position, cruise altitude, speeds, guard limits,
entry/return behaviour) stay in `mission_config.yaml` and are shared with the simulation — see
`catkin_ws/src/nidar_bringup/README.md` and §10.

### 5.4 Regenerate after editing

```bash
python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py          # write + print frame chain
python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py --check  # drift only (hw_bringup runs it)
```

It writes `nidar_slam/config/fast_lio/nidar_hw.yaml`, `nidar_hardware/config/px4/nidar_hw.params`
and `nidar_hardware/config/livox/MID360_config.json`, and refuses impossible combinations (rotated
LiDAR with its own IMU, rangefinder above the FC, mismatched bauds). **If the `.params` file changed,
reload it into PX4 (§7.2).** Read the printed frame chain: a sign error shows up there.

## 6. Sensors

### 6.1 Mid-360

* Bench check: `ping <lidar_ip>`, then in the container
  `roslaunch nidar_hardware hw_drivers.launch` (or `LIDAR=1 CAMERA=0 hw_bringup.sh`) and
  `rostopic hz /nidar/livox/lidar /nidar/livox/imu` → 10 Hz / ~200 Hz.
* **Time sync.** Without sync the Mid-360 stamps data with time-since-power-on. `livox_bridge.py`
  maps that onto ROS time with one offset (the minimum delay over 10 s) for scans and IMU alike
  (log: `re-stamping scans + IMU onto ROS time`). For hardware-grade stamps run PTP instead:
  `sudo ptp4l -i eth0 -m -S` on the Jetson host (as a service), then set `lidar.time_sync: ptp`.
  Required if `lidar.imu_source: fcu`.
* `/nidar/livox/status` reports rates and the clock offset once a second.

### 6.2 TFmini Plus

* On the FC (`source: fcu`): PX4 `SENS_TFMINI_CFG` (in the params file) starts the driver; check
  `rostopic echo /mavros/distance_sensor/hrlv_ez4_pub` and `/tfmini/range`.
* On the Jetson (`source: serial`): set `serial_port` (prefer `/dev/serial/by-id/...` names); the
  node parses the 9-byte frames itself (115200 baud, cm output — the TFmini Plus factory default).
* Verify against a tape: 0.20 / 0.50 / 1.00 m (checklist Phase 1 §4.3).

### 6.3 Camera

`v4l2-ctl --list-devices` and `v4l2-ctl -d /dev/video0 --list-formats-ext` show what the camera can
deliver. USB cameras need MJPG for 720p30 (the node requests it). Check
`rostopic hz /camera/image_raw` and look at it on the GCS camera panel.

### 6.4 Camera calibration

Survivor positions are computed from the camera intrinsics; calibrate once per camera and lens:

```bash
# laptop dev container with the ROS link to the Jetson (ROS_MASTER_URI=http://<jetson_ip>:11311, ROS_IP=<laptop_ip>),
# hardware.yaml camera.calibration_file still "" (raw, distorted frames):
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 \
       image:=/camera/image_raw camera:=/camera
# SAVE -> /tmp/calibrationdata.tar.gz -> extract ost.yaml to
#   catkin_ws/src/nidar_hardware/config/camera/ost.yaml
```

Set `camera.calibration_file: catkin_ws/src/nidar_hardware/config/camera/ost.yaml` (absolute paths
also work); the node then undistorts frames and publishes the matching pinhole model.

### 6.5 Detector model

`perception.model` default is `PERSON_DETECTION_MODEL_V3/best_ncnn_model` (already NCNN, runs on the
CPU). To fly the newer YOLO26s model on the CPU, export it once on the dev PC:

```bash
catkin_ws/src/nidar_hardware/scripts/export_detector.sh   # -> YOLO26S_DRONE_PERSON_V1/best_ncnn_model
```

and point `perception.model` at the result. Test on real footage of the arena dummies before a run:
the simulation-trained thresholds are a starting point only.

## 7. PX4 flight controller

### 7.1 Firmware, airframe, calibration (QGroundControl, FC on USB)

1. Flash **PX4 v1.14** (the SITL in this repo is v1.14.3).
2. Airframe: the closest match (e.g. *Holybro X500 V2*, or *Generic Quadcopter*), then set motor
   positions/directions in **Actuators** and run the motor test (**props off**).
3. Calibrate accelerometer, gyro, level horizon, RC (T12), ESCs, power module (voltage + current).
   Skip the compass (§7.3: no magnetometer indoors).
4. Battery: `BAT1_N_CELLS`, charged/empty voltage.
5. RC switches: **Position**, **Land**, and a **Kill switch**. The safety pilot takes over with
   Position mode or kills the motors (§9).

### 7.2 Load the NIDAR parameters

`nidar_hardware/config/px4/nidar_hw.params` (generated, §5.4):

* QGroundControl → **Parameters → Tools → Load from file…** → reboot the FC; or
* from the Jetson: `rosrun mavros mavparam load catkin_ws/src/nidar_hardware/config/px4/nidar_hw.params`,
  then reboot (`rosrun mavros mavcmd long 246 1 0 0 0 0 0 0`).

Re-load whenever the generator says the file changed. The file lists *why* each value is set.

### 7.3 What the parameters do

* **Links:** TELEM1 = radio to the GCS (`MAV_0_*`), TELEM2 = Jetson in Onboard mode (`MAV_1_*`,
  921600), `UXRCE_DDS_CFG 0` (PX4 v1.14 can start a ROS 2 DDS client on TELEM2; it must not).
* **Forwarding** (`MAV_0_FORWARD = MAV_1_FORWARD = 1`): PX4 relays MAVLink between the two links, so
  the GCS's TAKEOFF/RTL (sent over the radio to MAVROS's component 240) reach `mission_commander.py`
  and its replies and mission reports reach the GCS. It also forwards the Jetson's 10 Hz vision pose
  to the radio (~1.5 kB/s): fine at ≥ 57600 baud. With `fcu.mavlink_forwarding: false` TAKEOFF/RTL
  need the ROS link (Wi-Fi); LAND and ABORT always work.
* **Estimator** — same choices as the validated SITL airframe
  (`simulation/PX4-Autopilot-v1.14.3/ROMFS/.../1025_gazebo-classic_x500_vlp16`, which explains each):
  external vision for position + yaw (`EKF2_EV_CTRL 11`), height reference vision (`EKF2_HGT_REF 3`,
  FAST-LIO's Z is the obstacle-gated TFmini), no GPS, no magnetometer (`EKF2_MAG_TYPE 5`,
  `SYS_HAS_MAG 0`), baro as backup, `EKF2_EV_POS_*` = where FAST-LIO's IMU sits.
* **Failsafes — indoor rules:** never PX4 Return (its straight line home crosses the arena walls).
  RC loss → Land, except in Offboard (the autonomous mission continues; `COM_RCL_EXCEPT 4`);
  GCS link loss → nothing (`NAV_DLL_ACT 0`, the mission is autonomous); Jetson/offboard loss → Land
  (`COM_OBL_RC_ACT 4`); critical battery → Land (the commander's mission RTL at 30% comes first);
  attitude failure (> 60°) → flight termination (`CBRK_FLIGHTTERM 0`, as in SITL).

## 8. GCS laptop

### 8.1 Install (once)

The GCS needs **ROS Noetic** for the ROS link (map, survivors, onboard camera, TAKEOFF/RTL over
Wi-Fi). Two ways, both Docker:

**A. Dedicated GCS container (recommended for the field laptop).** Its own image `nidar-gcs` and
container `nidar_gcs`; it does not touch the dev container `ros_workspace` or its catkin build
(the web UI and `nidar_msgs` are built inside the image). Needs only Docker on the host.

```bash
scripts/gcs_docker.sh image      # docker/Dockerfile.gcs -> image nidar-gcs (~5-10 min once)
scripts/gcs_docker.sh start      # container nidar_gcs -> http://localhost:8000 -> HARDWARE
scripts/gcs_docker.sh logs       # follow the GCS log;  restart | stop | shell | rm
```

It restarts with Docker (`--restart unless-stopped`), uses the host network (UDP 14550, ROS to the
Jetson) and sees `/dev` with the `dialout` group (USB radios). Backend code and `backend/.env` are
read from the repository: `restart` after editing them. After frontend changes: `image`, `rm`,
`start`. Port 8000 already taken (a GCS running in `ros_workspace`)? `GCS_PORT=8001 scripts/gcs_docker.sh start`.
The SIMULATION page's START button needs Gazebo and stays in the dev container; the Hardware page's
SIMULATOR link connects to a SITL running there.

**B. Inside the dev container**, next to the simulation:

```bash
./scripts/docker_dev_start.sh            # dev container (has ROS Noetic); --net=host, /dev, dialout
# inside it:
catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh     # backend deps (incl. pymavlink) + UI build
catkin_ws/src/nidar_gcs/scripts/start_gcs.sh     # http://localhost:8000 -> HARDWARE
```

* **USB radios from inside the container** need `/dev` mounted and the `dialout` group: containers
  created by an older `docker_dev_start.sh` lack both — the script warns; recreate once with
  `docker rm -f ros_workspace && ./scripts/docker_dev_start.sh` (the repo is a bind mount).
* On the host: `sudo usermod -aG dialout $USER`; `sudo systemctl disable --now ModemManager` (it
  grabs new USB serial devices); on Ubuntu 22.04 `sudo apt remove brltty` (it steals CH340 adapters).
  `start_gcs.sh` warns about the first two.
* Without ROS (plain Python on the laptop) the Hardware page still works over MAVLink only.

### 8.2 Connect (Hardware page → CONNECT DRONE)

| Link | Dialog | Drone side |
|---|---|---|
| **T12** data link | SERIAL: **Scan Ports**, pick the T12 ground unit's port, baud = its data-link baud (57600 default) | Air unit on FC TELEM1 at `fcu.telem1_baud` |
| **SiK** radio | SERIAL (preset *SiK Telemetry @57600*) | Air radio on TELEM1 |
| **USB** to the FC | SERIAL (preset *PX4 USB*), the `/dev/ttyACM*` port | — (bench only) |
| **Wi-Fi** | UDP, bind 0.0.0.0, port **14550** (preset *Wi-Fi Drone UDP*) | `fcu.gcs_url: udp://@<gcs_ip>:14550` |
| **Simulator** | SIMULATOR | The NIDAR SITL on this laptop (`./scripts/test_takeoff.sh`); exercises the Hardware page end to end |

**Scan Ports** lists every serial device on the laptop with what it is (flight controller, FTDI/CP210x/
CH34x radio adapter, ...) and flags **NO PERMISSION** or **IN USE by <program>**. CONNECT only
succeeds once a **flight controller heartbeat** arrives; otherwise the dialog says why (port busy,
permission, no data, data but wrong baud, only the companion heard, ...). **Force Connect** keeps the
link open without a heartbeat (bench: radio on, vehicle not yet).

### 8.3 ROS link to the Jetson

With **UDP** the GCS finds the Jetson by itself: MAVLink arrives from the Jetson's IP, and the backend
starts its ROS bridge against `http://<that ip>:11311` (event: *ROS LINK ONLINE*). With a **serial**
link plus Wi-Fi, set the master explicitly in `catkin_ws/src/nidar_gcs/backend/.env`:

```bash
DRONE_ROS_MASTER_URI=http://192.168.144.50:11311   # "auto" = from the UDP sender, "" = MAVLink only
DRONE_ROS_IP=                                     # laptop IP for ROS; empty = detected
```

Both machines must reach each other on all ports (ROS uses random TCP ports): same subnet, no
firewall between them. The bridge's "hardware" profile pulls only light topics (the map at 1 Hz,
JPEG video at `camera.gcs_stream.fps`, a 2 Hz status summary) — no point clouds over the air.

### 8.4 Commands

| Button | What happens |
|---|---|
| **TAKEOFF** (press twice) | `mission_commander` checks: FC connected and disarmed, FAST-LIO tracking, LiDAR data, vision relay healthy, PX4 local position, rangefinder, guard streaming setpoints, mission manager in TAKEOFF, FUEL up, vehicle on the pad, battery ≥ `min_battery_pct`. Refusals name the failing check. Then AUTO.TAKEOFF → arm → OFFBOARD; the mission manager flies the mission. |
| **RTL** | Mission return: FUEL stops, the vehicle retraces its explored path out of the door and lands on the pad. Over the pad (still climbing) it simply lands. |
| **LAND** | PX4 AUTO.LAND where it is. |
| **EMERGENCY ABORT** (press twice) | AUTO.LAND over every link that is up. The RC kill switch remains the last resort. |

## 9. Bring-up and first flights

Follow [`hardware/README.md`](README.md) phase by phase. In practice:

1. **Bench, props off, FC + Jetson only:** `LIDAR=0 CAMERA=0 scripts/jetson_onboard.sh bringup`
   → MAVROS connects, `/tfmini/range` correct. GCS: connect over **each** link you will use;
   telemetry moves when you tilt the drone; LAND changes the mode.
2. **Bench, props off, full stack:** `scripts/jetson_onboard.sh bringup` → READY (or the reason).
   Carry the drone 1 m forward / left / up: FAST-LIO, `/mavros/local_position/pose` and the GCS map
   agree, with the right signs (checklist Phase 1 §5, Phase 2 §2.1–2.2).
3. **Manual hover on vision** (safety pilot, Position mode on the T12): proves EKF2 holds with FAST-LIO
   before any autonomy. Tune `EKF2_EV_DELAY` here (§10.2).
4. **Autonomous takeoff and land:** TAKEOFF, watch the climb and the hold over the pad, then **RTL**
   (lands on the pad) before it heads for the door.
5. **Arena entry, then full mission**, with the safety pilot ready on Position mode and the kill switch.

Each `hw_bringup.sh` run logs every process to `logs/hw/<run>/` and records a rosbag of the mission
topics (map, survivors, poses, states) there.

## 10. Tuning guide

### 10.1 Mapping / localisation (FAST-LIO)

| Symptom | Knob (file) |
|---|---|
| "No Effective Points" near walls | lower `lidar.blind` (hardware.yaml) |
| Self-returns (props) in the map | raise `lidar.blind`, or mount the LiDAR higher |
| Odometry < 9 Hz, Jetson CPU saturated | `point_filter_num` 4 → 6, `filter_size_surf/map` 0.5 → 0.6 (`nidar_slam/launch/nidar_mapping.launch`); lower `perception.detect_hz` |
| Drift / jumps in small rooms | `filter_size_surf/map` 0.5 → 0.3 (more detail, more CPU) |
| Map rotated against the arena | the drone did not start on the pad facing the door (§5.1), or `lidar.mount.yaw` ≠ physical |
| Altitude wrong by a constant | `rangefinder.mount.z` |

Check: `rostopic hz /Fast_LIO/odometry` ≥ 9 Hz; 1 m carry → 1.00 ± 0.05 m.

### 10.2 PX4 estimator

* **`px4.ev_delay_ms`** — the delay between the LiDAR scan and PX4 receiving its pose. Too small or
  too large makes the vision innovations oscillate during motion. Fly a manual hover with small
  stick inputs, then compare `vehicle_visual_odometry` with `vehicle_local_position` in the log
  (PX4 Flight Review / PlotJuggler) and adjust in 10 ms steps until they line up.
* **`EKF2_EVP_NOISE`** (params file) — raise if PX4 logs vision position resets; lower if the
  position estimate is sluggish.
* **`px4.mpc_thr_hover`** — after a hover, set it to the logged `hover_thrust_estimate`.

### 10.3 Flight envelope and mission (`mission_config.yaml`, shared with the simulation)

* **`vehicle.collision_radius`** — measure the **real** frame: centre to the outermost prop tip.
  Then **`planner.obstacles_inflation`** must stay ≥ it plus the localisation error (keep ≥ 0.1 m
  margin); also `entry.min_clearance`. Re-run `apply_mission_config.py` after changing them.
* `flight.max_vel` / `max_acc` (0.6 / 0.3): raise only after clean runs.
* `vehicle.cruise_altitude_world` and `guard.z_min/z_max`: the flight altitude band.
* `guard.mission` / `guard.explore` boxes: must match the real arena and pad layout.

Run `python3 catkin_ws/src/nidar_config/scripts/apply_mission_config.py` after editing.

### 10.4 Detection

`perception.detect_hz`, `confidence_threshold`, `target_height_m` (hardware.yaml); the tracker's
`association_radius` / `confirmation_threshold` (`nidar_perception/launch/detector.launch`). Survivor
position errors that grow toward the image edges mean the camera needs calibrating (§6.4); a constant
bias means `camera.mount`.

### 10.5 Commander and links

`commander.*` thresholds; `camera.gcs_stream` (video bandwidth); `DRONE_*` in the GCS `.env`.

## 11. Troubleshooting

**GCS CONNECT (the dialog states one of these):**

| Message | Fix |
|---|---|
| Permission denied on /dev/tty… | `sudo usermod -aG dialout $USER`, log out/in; dev container: recreate (§8.1) |
| … is busy (open in …) | close QGroundControl / the other program; `sudo systemctl disable --now ModemManager` |
| … does not exist | plug in, **Scan Ports**; CH340 on Ubuntu 22.04: `sudo apt remove brltty` |
| bytes but no valid MAVLink | baud mismatch: dialog baud = radio = `SER_TEL1_BAUD` |
| No data at all | air unit powered and bound? TX/RX swapped? `MAV_0_CONFIG` = TELEM1? |
| No MAVLink on UDP | Jetson `fcu.gcs_url` → this laptop's IP:14550; same network; `sudo ufw allow 14550/udp` |
| UDP port 14550 already in use | QGroundControl is running: close it, or use another port in both places |
| TAKEOFF … no answer from the onboard commander over MAVLink | no Wi-Fi ROS link and forwarding off: §7.3 |
| ROS LINK UNAVAILABLE | GCS not running in a ROS environment (§8.1) |
| ROS link waits forever | Jetson `network.jetson_ip` wrong / not on an interface; firewall; `DRONE_ROS_MASTER_URI` |

**Onboard (`hw_bringup.sh` stops with the reason):**

| Problem | Check |
|---|---|
| MAVROS never connects | `fcu.url` device and baud; TELEM2 TX/RX; `MAV_1_CONFIG 102`; `UXRCE_DDS_CFG 0` |
| Mid-360 not answering ping | its power (9–27 V), cable, Jetson `host_ip` static address (§3.2) |
| no `/nidar/livox/*` | `lidar_ip`/`host_ip` in `hardware.yaml` → regenerate → restart; `logs/hw/<run>/drivers.log` |
| no `/mavros/local_position/pose` | params not loaded (`EKF2_EV_CTRL`), FC not rebooted after loading, FAST-LIO not running |
| TAKEOFF refused: *vehicle not on the launch pad origin* | the stack was started away from the pad; restart it with the drone on the pad |
| Arming refused | QGC messages: safety switch, battery, RC, pre-flight checks |

## 12. File map

| Path | Role |
|---|---|
| `catkin_ws/src/nidar_config/config/hardware.yaml` | **all** vehicle-specific settings and placements |
| `catkin_ws/src/nidar_config/scripts/apply_hardware_config.py` | generator + validator (§5.4) |
| `catkin_ws/src/nidar_hardware/` | onboard package: `livox_bridge.py`, `rangefinder_node.py`, `camera_publisher.py`, `hw_static_tf.py`, `mission_commander.py`, `launch/hw_drivers.launch`, `launch/hw_mission.launch`, generated PX4/Livox config, `systemd/nidar-onboard.service`, tests |
| `catkin_ws/src/nidar_bringup/scripts/hw_bringup.sh`, `hw_stop.sh` | onboard orchestrator (never arms) / stop (refuses while armed) |
| `catkin_ws/src/nidar_slam/config/fast_lio/nidar_hw.yaml` | generated FAST-LIO config for the Mid-360 |
| `docker/Dockerfile.jetson`, `scripts/jetson_onboard.sh` | onboard image and helper (§3.3) |
| `docker/Dockerfile.gcs`, `scripts/gcs_docker.sh` | GCS-only image and helper (§8.1 A) |
| `catkin_ws/src/nidar_gcs/backend/app/services/mavlink_service.py`, `hardware_service.py`, `serial_ports.py` | GCS hardware link: MAVLink (serial/UDP/TCP), ROS link, commands, port discovery |
| `catkin_ws/src/nidar_gcs/frontend/src/hooks/useHardwareBackend.ts` | Hardware page ← backend (telemetry, map, events) |

Unchanged for the simulation: every launch-file argument added for the hardware defaults to the
simulation's previous value, and `./scripts/test_takeoff.sh` runs exactly as before.
