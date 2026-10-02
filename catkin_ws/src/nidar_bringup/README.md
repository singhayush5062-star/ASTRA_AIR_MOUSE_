# nidar_bringup

Top-level composition for the NIDAR AirMouse stack. This package owns no node
implementations. It owns the order in which the other packages come up.

## Run the full simulation mission

From the repo root:

```bash
./scripts/test_takeoff.sh true     # Gazebo GUI on;  RVIZ=1 for RViz
./scripts/test_takeoff.sh false    # headless
```

`scripts/test_takeoff.sh` (repo root) is a thin wrapper around `scripts/test_takeoff.sh` in this
package, which brings the stack up in this order:

| Step | Package / file | What starts |
|------|----------------|-------------|
| 1 | `nidar_qa/scripts/verify_fix_parity.sh` | pre-flight parity and XML check (`SKIP_PARITY=1` to skip) |
| 2 | `nidar_sim/launch/nidar_sim.launch` | Gazebo + competition world, PX4 SITL, MAVROS |
| 3 | `nidar_slam/launch/nidar_mapping.launch` | FAST-LIO |
| 4 | `nidar_platform/scripts/relay_odometry.py` | FAST-LIO -> PX4 EKF2 external vision |
| 5 | `nidar_safety/scripts/flight_envelope_guard.py` | setpoint validation (limits from `nidar_config`) |
| 6 | `nidar_planner/launch/nidar_fuel_upstream.launch` | FUEL exploration + traj_server |
| 7 | `tools/flightlog/record.py`, `watchdog.py` | flight recorder and crash watchdog |
| 8 | `nidar_bringup/launch/mission_only.launch` | mission FSM, coverage, detector, 2D map |
| 9 | (inline) | arm, OFFBOARD, climb; then `nidar_mission/scripts/mission_telemetry_logger.py` |

## Package map

| Package | Owns |
|---------|------|
| `nidar_msgs` | `Survivor`, `SurvivorArray` |
| `nidar_config` | `mission_config.yaml`, `arena_grid.yaml`, `flight_envelope_guard.yaml`, `apply_mission_config.py` |
| `nidar_sim` | competition world, Gazebo models, `nidar_sim.launch` (sim only) |
| `nidar_platform` | `relay_odometry.py`, `build_px4.sh` (the PX4 tree stays at `simulation/PX4-Autopilot-v1.14.3`) |
| `nidar_slam` | FAST-LIO launch, config, URDF, RViz profile |
| `nidar_planner` | FUEL launch |
| `nidar_safety` | `flight_envelope_guard.py` |
| `nidar_mission` | `mission_manager.py` (ROS node `entry_detection_module`), coverage reporter, telemetry logger |
| `nidar_perception` | camera TF chain, `survivor_detector.py`, model weights |
| `nidar_map2d` | `/map_2d` slicer, grid overlay, `view_map2d.sh` |
| `nidar_qa` | `verify_fix_parity.sh`, `analyze_exploration.py` |
| `nidar_bringup` | this package |

Every package's maintainer is listed in its `package.xml`.

After editing `nidar_config/config/mission_config.yaml`, re-render the derived files:

```bash
python3 catkin_ws/src/nidar_config/scripts/apply_mission_config.py          # apply
python3 catkin_ws/src/nidar_config/scripts/apply_mission_config.py --check  # drift only
```
