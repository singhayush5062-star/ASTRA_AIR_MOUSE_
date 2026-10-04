# nidar_hardware

Real drone only (the simulation never loads this package): the Jetson-side drivers and relays that
give the NIDAR stack the topics Gazebo provides in simulation, the vehicle's static TF, and the
onboard mission commander behind the GCS's TAKEOFF / RTL / LAND.

| File | Role |
|---|---|
| `launch/hw_drivers.launch` | MAVROS, livox_ros_driver2 + `livox_bridge.py`, `rangefinder_node.py`, `camera_publisher.py`, `hw_static_tf.py` |
| `launch/hw_mission.launch` | `nidar_bringup/mission_only.launch` with the real camera's settings + `mission_commander.py` |
| `scripts/livox_bridge.py` | Mid-360 scans/IMU → FAST-LIO's message type, on ROS time |
| `scripts/rangefinder_node.py` | TFmini Plus (via PX4 or on a Jetson UART) → `/tfmini/range` |
| `scripts/camera_publisher.py` | USB / RTSP / CSI camera → `/camera/*` + a JPEG stream for the GCS |
| `scripts/hw_static_tf.py` | `body`/`base_link`/sensor frames from `hardware.yaml` mounts |
| `scripts/mission_commander.py` | TAKEOFF (checks + AUTO.TAKEOFF → arm → OFFBOARD), RTL, LAND; onboard status; STATUSTEXT reports |
| `scripts/export_detector.sh` | detector `.pt` → NCNN for the Jetson CPU |
| `config/px4/nidar_hw.params`, `config/livox/MID360_config.json` | generated from `nidar_config/config/hardware.yaml` |
| `systemd/nidar-onboard.service` | optional autostart on boot |

Everything is configured in `catkin_ws/src/nidar_config/config/hardware.yaml`; start it with
`nidar_bringup/scripts/hw_bringup.sh`. Full guide: `hardware/DEPLOYMENT.md`.

Tests (no ROS needed): `python3 -m unittest discover -s catkin_ws/src/nidar_hardware/test -v`
