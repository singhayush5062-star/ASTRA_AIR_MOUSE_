# Raspberry Pi 4 Deployment Guide (Docker)

How to put the NIDAR AirMouse onboard stack on a **Raspberry Pi 4 (8 GB)** and fly it from the GCS
laptop. Branch: **`pi4_deployment`**.

| Part | What we use |
|---|---|
| Onboard computer | Raspberry Pi 4, 8 GB, 64-bit OS, Docker |
| Flight controller | **Aeromind 6X**, **PX4 v1.16.1** |
| LiDAR | Livox **Mid-360** (Ethernet) |
| Rangefinder | Benewake **TFmini Plus** (on the FC) |
| Camera | **SIYI A8 mini** (Ethernet, RTSP) |
| Link to the GCS | **Skydroid T12**: RC + MAVLink, and an Ethernet network bridged to the ground |

The Pi runs **one Docker image**, `nidar-onboard-pi4`. It already contains ROS Noetic, MAVROS, the
Livox driver and the **compiled** NIDAR workspace (FAST-LIO, FUEL, guard, mission, detector).
**The Pi never compiles anything:** the image is built once on the dev PC and copied to the Pi.

This guide replaces the Jetson-specific parts of [`DEPLOYMENT.md`](DEPLOYMENT.md). Everything
that is not Pi-specific (sensor placement, `hardware.yaml` reference, tuning, the bring-up
checklist in [`README.md`](README.md)) is still there.

---

## 0. Quick start

Do the steps **in this order**. The PC can run step A (2–4 h) while you do steps B–C on the Pi.

```bash
# ── A. DEV PC (x86, Docker): build the image ──────────────────── once per C++ change ──
git clone -b pi4_deployment https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
scripts/pi4_onboard.sh image                 # 2-4 h the first time (ARM emulation)

# ── B. PI 4: prepare the host (64-bit OS flashed, ssh working, §3.1) ──────────── once ──
sudo apt update && sudo apt install -y git python3-yaml curl
curl -fsSL https://get.docker.com | sh       # Docker for arm64
git clone -b pi4_deployment https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
scripts/pi4_onboard.sh setup                 # UART, docker/dialout groups, eth0 addresses (over Wi-Fi!)
sudo reboot                                  # needed: UART overlay + group membership

# ── C. get the image onto the Pi (after B: needs Docker + the docker group there) ──
scripts/pi4_onboard.sh deploy <user>@<pi-ip> # from the DEV PC, e.g. pi@192.168.x.y (its Wi-Fi address)
#   or, on the Pi, from the team registry (§5): docker login ghcr.io -u <github-user>; scripts/pi4_onboard.sh pull

# ── D. PI 4: start and verify ────────────────────────────────────────────────── once ──
cd ~/ASTRA_AIR_MOUSE_
docker images | grep nidar-onboard-pi4       # the image arrived
scripts/pi4_onboard.sh check                 # fix every line marked "!!"
scripts/pi4_onboard.sh start                 # container nidar_onboard (restarts with Docker)

# ── E. every flight session (drone on the launch pad, nose toward the arena door) ──
scripts/pi4_onboard.sh bringup               # bench first: LIDAR=0 CAMERA=0 scripts/pi4_onboard.sh bringup
# GCS laptop: scripts/gcs_docker.sh start -> http://localhost:8000 -> HARDWARE -> CONNECT (UDP 14550)
```

If a step prints an error, look it up in §10 (Troubleshooting).

---

## 1. What runs where

```text
                 DRONE                                                  GROUND
 ┌──────────────────────────────────────────────────────────┐      ┌───────────────────────────┐
 │  Mid-360 ───┐                                            │      │  T12 ground unit           │
 │  A8 mini ───┤  Ethernet switch  ── T12 air unit (Eth) ───┼─RF──►│   └ Ethernet ─ GCS laptop  │
 │  Pi 4 eth0 ─┘                         │                  │      │      192.168.144.100       │
 │   │ 192.168.1.50  (LiDAR subnet)      │ TELEM1 + SBUS    │      │      nidar_gcs container   │
 │   │ 192.168.144.50 (camera/T12/GCS)   ▼                  │      │      http://localhost:8000 │
 │   │                         Aeromind 6X (PX4 v1.16.1)    │      └───────────────────────────┘
 │   └─ UART pins 8/10/6 ─── TELEM2 ──┘   ▲ GPS2: TFmini    │
 │      Docker: nidar_onboard (MAVROS, Livox driver,        │
 │      FAST-LIO, FUEL, guard, mission, detector, commander)│
 └──────────────────────────────────────────────────────────┘
```

* **MAVLink to the GCS**, two paths (use either, or both):
  * MAVROS on the Pi forwards it as UDP to `192.168.144.100:14550` over the T12 Ethernet bridge.
  * PX4 TELEM1 sends it over the T12 data link.
* **ROS link** (live 2D map, survivors, camera with detections, health, TAKEOFF/RTL): the GCS
  connects to the ROS master on the Pi (`192.168.144.50:11311`) over the same T12 network.
* **The Pi never arms by itself.** `bringup` reports READY; the operator presses TAKEOFF on the GCS.

## 2. Wiring

| From | To | Notes |
|---|---|---|
| FC **TELEM2** | Pi header **pin 8** (GPIO14, TX), **pin 10** (GPIO15, RX), **pin 6** (GND) | FC TX → Pi RX (pin 10), FC RX → Pi TX (pin 8). 3.3 V logic. **Never connect the FC's 5 V pin.** 921600 baud. On the Pixhawk-standard 6-pin TELEM connector, pin 1 = 5 V, 2 = TX, 3 = RX, 6 = GND. The Aeromind 6X is not in PX4's official board list, so **check its pinout sheet**. |
| FC **TELEM1** + **RC IN** | T12 air unit telemetry UART + SBUS | MAVLink at `fcu.telem1_baud` (57600 by default; must equal the T12's data-link baud). The T12 transmitter is the safety pilot. |
| **TFmini Plus** | FC **GPS2** | 5 V supply, TX/RX crossed. PX4 driver started by `SENS_TFMINI_CFG` (in the params file). |
| **Mid-360**, **A8 mini**, **T12 air unit**, **Pi eth0** | small **Ethernet switch** (5-port, 100 Mbit or gigabit) | One subnet per device family, all on one cable to the Pi (§4). |
| Mid-360 power | battery → regulator, **9–27 V** | ~6.5 W. Never from the Pi. |
| A8 mini power | per SIYI spec | |
| **Pi 4 power** | **5.1 V BEC, ≥ 3 A (5 A recommended)** → USB-C or GPIO pins 2/4 + 6 | Under-voltage throttles the CPU and costs FAST-LIO rate. Check `vcgencmd get_throttled` = `0x0` (§9). |
| Pi cooling | heatsink + **fan** | Mandatory. The Pi 4 throttles at 80–85 °C under this load. |
| Storage | **USB 3 SSD** (boot from it) preferred over the SD card | SD cards are slow and get corrupted by power cuts while rosbags are being written. |

## 3. Pi 4 host setup (once)

### 3.1 Operating system

Flash with **Raspberry Pi Imager** (enable SSH, set the user and Wi-Fi in its settings):

* **Ubuntu Server 22.04 LTS (64-bit)**, or
* **Raspberry Pi OS Lite (64-bit, Bookworm)**.

It **must be 64-bit**: `uname -m` must print `aarch64`. A 32-bit OS cannot run the image. The first
user created by the Imager has uid 1000, which is the uid the image is built for. With another uid,
build with `PI_UID=<uid> PI_GID=<gid> scripts/pi4_onboard.sh image`.

> **Wi-Fi on the Pi is only for setup (ssh, git, internet).** It must **not** be on `192.168.1.x`
> (the LiDAR subnet) or `192.168.144.x` (the drone network). Many home routers use `192.168.1.x`.
> If yours does, see §4.

### 3.2 Docker and the repository

```bash
sudo apt update && sudo apt install -y git python3-yaml curl   # tools the helper script needs
curl -fsSL https://get.docker.com | sh          # Docker for arm64 (Ubuntu or Raspberry Pi OS)
git clone -b pi4_deployment https://github.com/singhayush5062-star/ASTRA_AIR_MOUSE_.git
cd ASTRA_AIR_MOUSE_
```

The repository on the Pi holds the **configuration** (`hardware.yaml`), the launch files and the
Python nodes. It is mounted into the container, and the run logs are written into it (`logs/hw/`).

### 3.3 UART, groups, network: `scripts/pi4_onboard.sh setup`

```bash
scripts/pi4_onboard.sh setup        # prints what it changes, asks, uses sudo; then:
sudo reboot
```

It does, idempotently and with backups (`*.nidar.bak`):

| Step | Why | Manual equivalent |
|---|---|---|
| `config.txt`: `enable_uart=1`, `dtoverlay=disable-bt` | Bluetooth otherwise owns the good UART (PL011). With the overlay, pins 8/10 are `/dev/ttyAMA0`, stable at 921600. | edit `/boot/firmware/config.txt` (`/boot/config.txt` on older images) |
| `cmdline.txt`: remove `console=serial0,115200` | a login console on the FC port garbles MAVLink | edit `cmdline.txt` |
| disable `hciuart`, mask `serial-getty@ttyAMA0` | they open the UART | `systemctl disable/mask` |
| add the user to `docker`, `dialout` | run docker without sudo; serial access | `usermod -aG` |
| eth0 static `192.168.1.50/24` + `192.168.144.50/24`, no gateway | LiDAR subnet + camera/T12/GCS subnet on the switch | Pi OS: `nmcli` profile `nidar-eth0`; Ubuntu: `/etc/netplan/60-nidar.yaml` |

Run it over **Wi-Fi or with a keyboard**: it reconfigures eth0, which drops an ssh session that
runs over eth0. The addresses come from `hardware.yaml` (`lidar.host_ip`, `network.jetson_ip`). The
key name `jetson_ip` is historical; on this vehicle it is the Pi.

### 3.4 Clock (recommended)

The Pi has no real-time clock. Without internet it boots with a stale time. ROS works anyway (the
GCS uses the latest transform, and MAVROS syncs with PX4), but logs and rosbags from the Pi and the
laptop only line up if the clocks agree. Easiest: `chrony` on the Pi with the laptop as its server
(`server 192.168.144.100 iburst` in `/etc/chrony/chrony.conf`), or let the Pi sync from the
internet over Wi-Fi before the session.

## 4. Network

| Device | Address | Set where |
|---|---|---|
| Pi eth0 (LiDAR side) | `192.168.1.50/24` | `hardware.yaml lidar.host_ip` → `setup` |
| Mid-360 | `192.168.1.1XX` (XX = last 2 digits of the serial no.) | `hardware.yaml lidar.lidar_ip` (factory default; Livox Viewer 2 shows it) |
| Pi eth0 (drone network) | `192.168.144.50/24` | `hardware.yaml network.jetson_ip` → `setup` |
| A8 mini | `192.168.144.25` (SIYI default) | `hardware.yaml camera.source` (`rtsp://192.168.144.25:8554/main.264`) |
| GCS laptop | `192.168.144.100` | the laptop's interface toward the T12 ground unit; `hardware.yaml network.gcs_ip` |
| T12 air / ground units | the T12's own defaults | must not collide with `.25`, `.50`, `.100` |

All Ethernet devices share one switch and the Pi's single `eth0`. The two subnets coexist on that
interface. Rules:

* Each subnet must exist on **one** interface of the Pi. `scripts/pi4_onboard.sh check` warns when
  the Wi-Fi is also on `192.168.1.x` or `192.168.144.x`.
* If your setup Wi-Fi is `192.168.1.x`, either (a) join the Pi to another Wi-Fi network or a phone
  hotspot (usually `192.168.43.x` / `172.20.10.x`), or (b) move the Mid-360 to another subnet with
  **Livox Viewer 2** (e.g. `192.168.2.112`). Then put the new `lidar.lidar_ip` and `lidar.host_ip`
  in `hardware.yaml`, run `python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py`,
  and run `setup` again.
* The laptop must reach the Pi on **all** TCP ports (ROS uses random ports): no firewall on the
  laptop toward `192.168.144.0/24` (`sudo ufw allow from 192.168.144.0/24`).

## 5. Build the image on the dev PC and ship it

```bash
scripts/pi4_onboard.sh image                      # -> image nidar-onboard-pi4 (linux/arm64)
scripts/pi4_onboard.sh deploy pi@192.168.144.50   # ssh: docker save | gzip | docker load on the Pi
# or, without a network path to the Pi:
scripts/pi4_onboard.sh export                     # -> nidar-onboard-pi4_<commit>.tar.gz (~2-3 GB)
#    copy it (USB stick / scp), then on the Pi:  scripts/pi4_onboard.sh load nidar-onboard-pi4_<commit>.tar.gz
```

* `image` first registers **QEMU arm64 emulation** on the PC (a throwaway privileged container,
  `tonistiigi/binfmt`, valid until the PC reboots). It then builds two images: the base
  `nidar-onboard-pi4-base` (`docker/Dockerfile.jetson` for arm64: ROS, MAVROS, Livox SDKs, detector
  runtime) and `nidar-onboard-pi4` (`docker/Dockerfile.pi4`: the workspace compiled in
  `/opt/nidar_ws`). The first run takes **2–4 h** because everything is compiled under emulation.
  Later runs only redo the workspace layer.
* Needs ~15 GB of free disk on the PC. `JOBS=2` if the PC runs out of memory while compiling.
  Builds use `--network=host`, because apt inside Docker's bridge network produced
  "Hash Sum mismatch" errors on our network.
* No existing container or image is touched (new names only). The x86 images (`nidar-gcs`, the dev
  container) are unaffected.
* The image records the commit it was built from. `scripts/pi4_onboard.sh status` (or `check`) on
  the Pi shows `image built from <sha>, repo at <sha>`.

### Team registry (ghcr.io): pull instead of building

The team does not need to build the image: it is published as a **private** package linked to this
repository, `ghcr.io/singhayush5062-star/nidar-onboard-pi4` (tags: the commit it was built from,
and `latest`).

Access: the repository owner shares the package with the team (GitHub → Packages →
`nidar-onboard-pi4` → Package settings → add the repo collaborators / give it the repository's
access). Each person needs a GitHub token (Settings → Developer settings → Tokens (classic)) with
**`read:packages`** (and **`write:packages`** to publish).

```bash
# publish (the person who built it, on the dev PC)
docker login ghcr.io -u <github-user>        # password = the token (never commit or share it)
scripts/pi4_onboard.sh push                  # -> :<commit> and :latest

# use (on the Pi, instead of deploy/load)
docker login ghcr.io -u <github-user>        # once per Pi
scripts/pi4_onboard.sh pull                  # or: pull <commit>
scripts/pi4_onboard.sh rm; scripts/pi4_onboard.sh start
```

The pull needs internet on the Pi (~1.3 GB). Without internet at the field, pull at home or use
`deploy` / `export` + `load`.

### When to rebuild what

| You changed | On the Pi |
|---|---|
| `hardware.yaml`, `mission_config.yaml`, launch files, Python nodes | `git pull` (or edit), then `stop` + `bringup`. **No new image.** After `hardware.yaml` changes, also run `apply_hardware_config.py` (§6.1). |
| C++ (FAST-LIO, FUEL, ...), `package.xml`/`CMakeLists.txt`, new messages | PC: `image` + `deploy`. Pi: `git pull`, `rm`, `start`. |
| Docker files | the same as for C++ |

Inside the container, the repository's `catkin_ws/devel` is a link to `/opt/nidar_ws/devel` (the
compiled workspace in the image). The container's entrypoint creates it on every start. Do not run
`catkin build` on the Pi.

## 6. Devices

### 6.1 Flight controller: Aeromind 6X, PX4 v1.16.1

1. **Board check.** QGroundControl → Analyze Tools → MAVLink Console → `ver all`. The image should
   be the FMUv6X target (`px4_fmu-v6x`); its serial ports are TELEM1, TELEM2 and GPS2, as used here.
   If the board reports another target or labels its ports differently, change `fcu.*` and
   `rangefinder.px4_port` in `hardware.yaml` and regenerate.
2. Airframe, motor order and direction (motor test with **props off**), calibration (accel, gyro,
   level, RC, ESCs, **power module: voltage and current**). No compass indoors. RC switches:
   Position, Land, **Kill**.
3. **Load the NIDAR parameters** and **reboot the FC**: QGC → Parameters → Tools → Load from file →
   `catkin_ws/src/nidar_hardware/config/px4/nidar_hw.params`. Or, from the Pi with the stack up:
   `scripts/pi4_onboard.sh shell`, then
   `rosrun mavros mavparam load catkin_ws/src/nidar_hardware/config/px4/nidar_hw.params` and
   `rosrun mavros mavcmd long 246 1 0 0 0 0 0 0` (reboot).
4. After a reboot, check in QGC: `SER_TEL2_BAUD` = 921600, `MAV_1_CONFIG` = TELEM 2,
   `SER_TEL1_BAUD` = the T12 data-link baud.

All 45 parameters in `nidar_hw.params` were checked against the **PX4 v1.16.1 source**: every name
exists and every value means the same as in v1.14 (estimator from FAST-LIO, no GPS or compass,
indoor failsafes, TELEM1/TELEM2/GPS2 = port ids 101/102/202). The simulator in this repo is still
v1.14.3, so do one **props-off** TAKEOFF → LAND on the bench to confirm the mode sequence
(AUTO.TAKEOFF → armed → OFFBOARD) on v1.16.1.

If you edit `hardware.yaml`:

```bash
python3 catkin_ws/src/nidar_config/scripts/apply_hardware_config.py   # regenerate + frame chain
# if it says nidar_hw.params changed: reload the params into PX4 and reboot the FC
```

### 6.2 Livox Mid-360

* Mount: upright, cable to the back, square to the frame, ideally straight above the FC. Measure
  `lidar.mount` (DEPLOYMENT.md §5.2).
* `scripts/pi4_onboard.sh check` must show `Mid-360 192.168.1.1XX answers`.
* With the stack up: `rostopic hz /nidar/livox/lidar /nidar/livox/imu` → **10 Hz / ~200 Hz**.

### 6.3 SIYI A8 mini

* Default address `192.168.144.25`, stream `rtsp://192.168.144.25:8554/main.264` (already in
  `hardware.yaml camera.source`).
* **Set the main stream to 1280×720, H.264** with the SIYI FPV app or SIYI Assistant. The Pi 4
  decodes video on the CPU. 4K or H.265 streams cost a whole core and starve FAST-LIO.
* **Gimbal.** Survivor positions are computed from the fixed camera angle in `camera.mount`
  (`pitch` positive = tilted down). Put the gimbal at a fixed pitch (e.g. 15° down), enter that
  angle in `camera.mount.pitch`, and keep it there for the whole flight. Moving the gimbal during
  the mission moves the survivor positions.
* Check the stream from the Pi:
  ```bash
  scripts/pi4_onboard.sh shell
  python3 -c "import cv2; c=cv2.VideoCapture('rtsp://192.168.144.25:8554/main.264'); print(c.read()[0], c.get(3), c.get(4))"
  # -> True 1280.0 720.0
  ```
* The GCS laptop can also open the RTSP stream directly (Hardware page → camera panel →
  CONFIGURE STREAM). That costs the Pi nothing. The onboard stream (`camera.gcs_stream`) is the one
  with the detector's boxes.

### 6.4 TFmini Plus

On FC GPS2. With the stack up: `rostopic echo /tfmini/range`. Check against a tape at
0.20 / 0.50 / 1.00 m.

## 7. Run

```bash
scripts/pi4_onboard.sh start              # container nidar_onboard (restarts with Docker)
scripts/pi4_onboard.sh check              # host checks; fix every "!!" line
LIDAR=0 CAMERA=0 scripts/pi4_onboard.sh bringup   # bench step 1: FC + TFmini only
scripts/pi4_onboard.sh bringup            # full stack -> "Onboard stack UP. Commander: READY..."
scripts/pi4_onboard.sh stop               # (refuses while armed)
scripts/pi4_onboard.sh status             # container, versions, CPU / RAM / temperature
scripts/pi4_onboard.sh autostart          # optional: bringup at every boot (systemd)
```

`bringup` is `hw_bringup.sh` from DEPLOYMENT.md, unchanged. It checks the config, the FC port, the
LiDAR ping and the camera. It then starts MAVROS → LiDAR → FAST-LIO → PX4 EKF2 → guard → FUEL →
mission + detector + commander, waiting for each, and stops with the reason if something is
missing. Logs: `logs/hw/<run>/` (one file per process + a rosbag).

**Always start it with the drone on the launch pad, nose toward the arena door**: that pose places
the map, the grid and the survivors in the arena.

## 8. GCS laptop

1. Laptop on the T12 ground network with **`192.168.144.100`**.
2. `scripts/gcs_docker.sh start` (its own container `nidar_gcs`, see DEPLOYMENT.md §8.1 A) →
   `http://localhost:8000` → **HARDWARE**.
3. **CONNECT DRONE → UDP**, port **14550**. The GCS then attaches to the Pi's ROS master by itself
   (event *ROS LINK ONLINE*). Over the T12 serial port instead: SERIAL, its port, 57600, and set
   `DRONE_ROS_MASTER_URI=http://192.168.144.50:11311` in `catkin_ws/src/nidar_gcs/backend/.env`.
4. **Link test** (read-only, sends nothing to the drone; 2 minutes):
   ```bash
   T=/home/developer/NIDAR/catkin_ws/src/nidar_gcs/scripts/gcs_link_test.py
   docker exec nidar_gcs python3 $T udp
   # or: docker exec nidar_gcs python3 $T serial --dev /dev/ttyUSB0 --baud 57600
   ```
   **PASS** = heartbeat never older than 2 s, no dropouts, ≥ 5 MAVLink msg/s, GCS telemetry
   ≥ 8 Hz with no stall over 500 ms. It also prints the ROS link state and the radio RSSI.

## 9. Performance on the Pi 4: gates before the first flight

The stack was developed on a desktop and sized for a Jetson Orin Nano (roughly 3–4× the Pi 4's CPU).
Defaults on this branch are set for the Pi: detector `detect_hz: 1.0`, camera `publish_fps: 5`,
720p H.264 from the A8 mini. FAST-LIO keeps the validated `point_filter_num: 4`, `filter_size: 0.5`
(`hardware.yaml lidar.fastlio`). FAST-LIO runs single-threaded on ARM (its CMake enables
multi-threading only on x86).

Run the full stack on the bench for **10 minutes**, carrying the drone around the room, and check:

| Gate | Command | Must be |
|---|---|---|
| FAST-LIO rate | `rostopic hz /Fast_LIO/odometry` | **≥ 9 Hz** steady |
| PX4 position | `rostopic hz /mavros/local_position/pose` | ≥ 25 Hz, no gaps |
| Heat / power | `vcgencmd measure_temp`, `vcgencmd get_throttled` | < 80 °C, **`0x0`** |
| CPU | `scripts/pi4_onboard.sh status`, `htop` on the host | some headroom (< ~85 % total) |
| Detector | `rostopic hz /survivor_detector/detections`; `latency_ms` in `rostopic echo -n1 /survivor_detector/detections` | ≈ `detect_hz`; latency well under `1000/detect_hz` ms |
| 1 m carry | FAST-LIO vs tape | 1.00 ± 0.05 m |

If a gate fails, change these in order (all in `hardware.yaml`, then `stop` + `bringup`):

1. `perception.detect_hz` 1.0 → 0.5, or `CAMERA=0` to see how much the detector costs.
2. `lidar.fastlio.point_filter_num` 4 → 6, then `filter_size` 0.5 → 0.6.
3. A smaller detector model (`perception.model`, export with `nidar_hardware/scripts/export_detector.sh`).
4. Check the A8 mini really sends 720p H.264 (§6.3).

If FAST-LIO still cannot hold 9 Hz with the full stack, the Pi 4 is not enough for this mission.
Move the onboard computer to a Pi 5 or a Jetson Orin Nano (same Docker approach,
`scripts/jetson_onboard.sh`), or run detection on the GCS.

## 10. Troubleshooting

| Symptom | Fix |
|---|---|
| `check`: `/dev/ttyAMA0 missing` | `setup`, then reboot; `ls -l /dev/ttyAMA0 /dev/serial0` |
| MAVROS never connects | TX/RX swapped? Baud: `fcu.url` = `SER_TEL2_BAUD` = 921600 (try 460800 on both if the wires are long). `MAV_1_CONFIG` = TELEM 2, FC rebooted after loading the params. Serial console still on (`cat /proc/cmdline`). |
| `Mid-360 ... does not answer ping` | power (9–27 V), switch, `ip -4 addr show eth0` shows `192.168.1.50`; LiDAR IP = `lidar.lidar_ip` |
| `subnet ... is on 2 interfaces` | the Wi-Fi is on the LiDAR or drone subnet (§4) |
| camera never publishes | `ping 192.168.144.25`; the RTSP test in §6.3; stream codec H.264 |
| GCS: *No MAVLink on UDP* | laptop IP = `192.168.144.100` (= `fcu.gcs_url`); `sudo ufw allow 14550/udp`; `ping 192.168.144.50` from the laptop |
| GCS: ROS link waits forever | laptop firewall (all TCP ports from `192.168.144.0/24`); `hardware.yaml network.jetson_ip` is on eth0 (`check`) |
| `catkin_ws not built` in bringup | container started from an old image, or `catkin_ws/devel` deleted (`git clean -x`): `scripts/pi4_onboard.sh rm && scripts/pi4_onboard.sh start` |
| `cannot write .../catkin_ws/devel` | the repository on the Pi is not owned by uid 1000. Rebuild with `PI_UID`/`PI_GID` (§3.1) or `chown -R`. |
| `python3-yaml missing` | `sudo apt install -y python3-yaml` |
| `deploy`: `permission denied ... docker.sock` | `setup` not run yet on the Pi, or no reboot/re-login after it (docker group) |
| `exec format error` | the image is x86, or the OS is 32-bit (`uname -m` must be `aarch64`) |
| detector: `libgomp-....so: cannot allocate memory in static TLS block` | an ARM limit with torch/ncnn. `detector.launch` handles it (`$NIDAR_DETECTOR_PREFIX`, set in the image). When you run the detector or ultralytics by hand: `$NIDAR_DETECTOR_PREFIX python3 ...` |
| detector: `KeyError: 16` in `cv_bridge` | pip OpenCV 5 in the image; the image pins `opencv-python<5`. Rebuild the image |
| FAST-LIO < 9 Hz, temperature > 80 °C, `throttled` ≠ `0x0` | §9; fan, 5 V supply |
| build on the PC fails with "Hash Sum mismatch" / network errors | re-run `image` (already-built layers are reused) |
| build on the PC killed (out of memory) | `JOBS=2 scripts/pi4_onboard.sh image` |

## 11. File map

| Path | Role |
|---|---|
| `scripts/pi4_onboard.sh` | everything in this guide: `image`, `deploy`, `export`, `push`, `pull`, `setup`, `check`, `load`, `start`, `bringup`, `stop`, `status`, `shell`, `autostart`, `rm` |
| `docker/Dockerfile.jetson` | the arm64 base image (ROS, MAVROS, Livox, detector runtime), shared with the Jetson |
| `docker/Dockerfile.pi4` (+ `.dockerignore`) | the Pi image: base + compiled workspace + entrypoint |
| `catkin_ws/src/nidar_config/config/hardware.yaml` | all vehicle settings (this branch: Pi 4, Aeromind 6X, A8 mini) |
| `catkin_ws/src/nidar_hardware/config/px4/nidar_hw.params` | generated PX4 parameters |
| `catkin_ws/src/nidar_bringup/scripts/hw_bringup.sh` | the onboard bring-up (never arms) |
| `catkin_ws/src/nidar_gcs/scripts/gcs_link_test.py` | read-only GCS ↔ drone link test |
| `hardware/DEPLOYMENT.md`, `hardware/README.md` | sensor placement, tuning, bring-up checklist |
