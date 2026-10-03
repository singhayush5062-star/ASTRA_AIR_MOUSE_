# nidar_gcs — Ground Control Station

The team's AirMouse GCS web UI (imported from `P07awan/Airmouse@68ba3c2`, see
`UPSTREAM_UI_COMMIT`) wired to the real NIDAR simulation. Open the UI, press **START**, and the
full stack (Gazebo + PX4 SITL + FAST-LIO2 + FUEL + guard + mission layer) is started; every panel
of the Simulation dashboard shows the live run.

## Setup (once per machine)

```bash
catkin build nidar_gcs nidar_perception && source catkin_ws/devel/setup.bash
catkin_ws/src/nidar_gcs/scripts/setup_gcs.sh
```

`setup_gcs.sh` installs, per user under `~/.local/share/nidar_gcs/` (override with
`NIDAR_GCS_HOME`):

* `pydeps/` — FastAPI, uvicorn, pydantic 2, … with `pip --target`. They are put on the backend's
  `sys.path` only, never on `PYTHONPATH`, so no ROS process can import them.
* `node/` — a Node.js LTS runtime (checksum-verified), used only to build the frontend.

and builds the UI into `frontend/dist/`.

## Run

```bash
catkin_ws/src/nidar_gcs/scripts/start_gcs.sh
```

Open **http://localhost:8000** → **SIMULATION** → **START**.

| Control | What it does |
|---|---|
| START | runs `scripts/test_takeoff.sh` (headless) — exactly what a terminal user runs. Its output goes to `logs/gcs/sim_<time>.log` and its milestones to the event timeline. On a paused sim, START resumes. |
| PAUSE | pauses Gazebo physics (`/gazebo/pause_physics`); PX4 SITL runs in lockstep, so the whole vehicle freezes. The mission clock (sim time) stops too. |
| RESET | stops every simulation process with `nidar_bringup/scripts/stop_sim.sh` (the same kill list `test_takeoff.sh` uses for its clean slate). |
| SCENARIO | `SCENARIO_01` = the active arena in `mission_config.yaml` (`arena.active`). Other entries are not installed yet. |
| SPEED | display only (unchanged from the upstream UI); the simulation always runs in real time. |

A simulation started from a terminal (`./scripts/test_takeoff.sh`) is picked up automatically.
Stopping the GCS (Ctrl-C) never stops a running simulation, and a restarted GCS re-attaches to it.

Backend settings are in `backend/.env`: `SIM_GAZEBO_GUI=True` also opens the Gazebo window on
START; `GCS_MOCK=True` serves the old synthetic data (UI work without ROS).

## How it works

```
 Gazebo / PX4 SITL / MAVROS / FAST-LIO2 / FUEL / nidar_* nodes
        │  ROS topics
        ▼
 scripts/gcs_ros_bridge.py      rospy node; JSON lines on stdout, commands on stdin;
        │                       exits when the ROS master goes away, restarted by the backend
        ▼
 backend/ (FastAPI, :8000)      app/services/ros_link.py     supervises the bridge
        │                       app/services/sim_service.py  START/PAUSE/RESET, health, events,
        │                                                    survivors, mission clock
        │                       app/services/mission_view.py stack → UI mission phases (pure)
        │  REST + WebSocket (unchanged contract) + the built UI
        ▼
 frontend/ (React)              hooks/useSimulationBackend.ts fills the existing store
```

| UI element | Source |
|---|---|
| position / attitude / velocity | `/mavros/local_position/{pose,velocity_local}` (FAST-LIO frame `camera_init`) transformed to the arena `world` frame through tf (`world → map → camera_init`) |
| battery, PX4 mode, armed | `/mavros/battery`, `/mavros/state` |
| mission state + phase tracker | `/edm/mission_state` (mission manager) mapped by `mission_view.py`; COMPLETE = landed and disarmed after LAND |
| elapsed | sim time since arming (frozen at completion) |
| autonomy target / distance | end point of FUEL's current B-spline (`/planning/bspline`), the entry door, or the launch pad |
| coverage | `/sdf_map/coverage` |
| system health | message age of `/mavros/imu/data`, `/velodyne_points`, `/Fast_LIO/odometry`, `/camera/image_raw`, `/survivor_detector/detections`, `/planning/pos_cmd`, `/flight_envelope_guard/status` |
| live 2D map (main panel) | `/map_2d` from `nidar_map2d/lidar_map_2d.py`: 0.05 m occupancy built from FAST-LIO's registered LiDAR scans (walls = returns 0.3–2.2 m above the floor, free = cells the laser rays crossed), north up. Drawn by the team's `OccupancyGridMap` component with the A1–G7 competition grid, frontier cells, the flown path, the drone, the entry/exit door, and each survivor as a hotspot with its grid box highlighted. Run-length encoded on the wire (tens of kB per update). |
| survivors list (sidebar) | `/survivors`: id, grid box, confidence, world position |
| camera | `/camera/image_raw`, rotated upright (the sim sensor is mounted rolled 180°), boxes from `/survivor_detector/detections`, MJPEG on `/api/camera/stream`; frames are encoded only while a viewer is connected |
| survivors | `/survivors` (latched), grid labels in the `grid_visualizer.py` convention (A1..G7) |
| event timeline | orchestrator milestones, stack state changes, survivor confirmations, warnings from mission-level nodes and errors from any node (rate-limited) |

EMERGENCY ABORT (Simulation page control bar, press twice within 3 s like the Hardware page's
button) calls `POST /api/mission/abort`, which commands PX4 `AUTO.LAND`: the vehicle lands where
it is. Nothing in the stack switches back to OFFBOARD afterwards.

The UI ships its fonts (IBM Plex Sans, JetBrains Mono via `@fontsource`) instead of loading them
from Google Fonts, so it renders identically with no internet connection (Mission Brief §7).

## Changes to the upstream UI

The upstream look is kept. Changes, all reviewable with
`git diff 87c046af -- catkin_ws/src/nidar_gcs/frontend`:

* `pages/Simulation/index.tsx` — data from `useSimulationBackend` instead of the in-browser mock;
  START/PAUSE/RESET call the backend. At the team's request (2026-10-02): the main panel shows the
  team's own 2D map component (from the Hardware page) instead of the illustrative 3D maze, the
  sidebar has a survivors section (the Hardware page's survivor entry styling), and the control
  bar has the Hardware page's two-press EMERGENCY ABORT.
* `components/map/OccupancyGridMap.tsx` — survivors drawn as hotspots with the grid box
  highlighted, the A1–G7 grid, entry/exit marker position and empty-state text as props, north up
  (it drew south up), cells cached per map update (a 0.05 m map is ~118k cells), and a React
  hook-order bug fixed (it would crash the moment a map arrived). At the team's request
  (2026-10-03) the cells use RViz's "map" colours (white floor, grey unknown, black walls)
  instead of the original near-black greys, so the GCS map reads like the RViz view of the same
  `/map_2d`; the legend swatches have an outline so black and white stay visible.
* `providers/SimulationProvider.ts` — the backend owns the simulation state; abort reaches it.
* `store/index.ts` — map, flown path, arena/grid and the setters the hook uses.
* `services/websocket.ts` — `ManagedWebSocket` exported.
* `styles/globals.css` — fonts bundled instead of fetched (same fonts and weights).

`components/simulation/Sim3DView.tsx` is unchanged and no longer used by the Simulation page.
* backend: real simulation service instead of `MockDataProvider` (kept for `GCS_MOCK=True`);
  Python 3.8 compatible; serves the built UI. The Hardware routes are untouched.

## Tests

```bash
python3 -m unittest discover -s catkin_ws/src/nidar_gcs/backend/tests -v
```
