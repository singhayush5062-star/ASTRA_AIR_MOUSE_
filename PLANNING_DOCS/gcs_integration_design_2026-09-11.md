# GCS Integration Design — NIDAR AirMouse

**Author:** Ayush Singh (with implementation notes from the 2026-09-11 session)
**Status:** design; nothing here is implemented yet

> **Superseded in part (2026-10-02).** The team built its own operator UI (React + FastAPI,
> `P07awan/Airmouse`), so the Foxglove Studio display, layout file and button extension in this
> document are not used. That UI is integrated with the simulation as
> `catkin_ws/src/nidar_gcs` (see its README): a ROS bridge node feeds the backend instead of
> rosbridge, and the UI is served on http://localhost:8000. The topic inventory and the
> no-external-network reasoning below still apply.
**Sits above:** Phase 4 (survivor detection, done in the same session)
**Sits below:** Phase 5–7 of `PLANNING_DOCS/nidar_phase_plan_to_mission_complete.md`
**Competition sections it satisfies:** Mission Brief §1 (overall objective — GCS
must display live feed), §4 (autonomy — no separate piloting device beyond the
GCS), §5 (Mission Planner/GCS requirements — the full display list), §7
(communication constraint — no external network), §10 (safety — abort/RTH).

---

## 1. What the competition actually asks for

The Mission Brief §5 says the GCS **MUST** display, all during flight:

1. Live mission status of the drone
2. Live camera feed from the drone
3. A 2D map, continuously updated during flight
4. Identified corridors / rooms / sections wherever feasible
5. The grid coordinate / grid box of each detected survivor
6. Tagged locations of detected survivors
7. Drone position (or estimated position) inside the mapped area
8. Mission progress and completion status

Constraint §7: **no external network**. No cloud, no LTE/5G, no public Wi-Fi.
Everything is local-only.

Constraint §8: exactly one operator supervising during the run. The only
allowed interactions are start, safety abort, and RTH trigger.

Constraint §10: failsafe interlocks (battery, link loss, geofence, abort,
recall) must be visible and actuatable through the GCS.

### 1.1 What we already have on the drone side (as of 2026-09-11)

| Concern | State | Topic |
|---|---|---|
| Live pose / flight status | ✅ | `/mavros/state`, `/mavros/local_position/pose`, `/Fast_LIO/odometry` |
| Live camera | ✅ (15 Hz, 640×480) | `/camera/image_raw`, `/camera/camera_info` |
| 2D map | ❌ **not yet** — FUEL only exposes a 3D voxel via `/sdf_map/occupancy_all` | needs a slicer |
| Corridors / rooms | ❌ nothing labelled | needs post-hoc segmentation |
| Survivor tags | ✅ (this session) | `/survivors` (latched SurvivorArray), `/survivor_markers` |
| Drone position on map | ✅ | `/Fast_LIO/odometry` + `world→map` static TF |
| Progress % + area breakdown | ✅ | `/sdf_map/coverage` + `coverage_reporter.py` terminal output |
| Failsafes | 🟡 partial — geofence works via `flight_envelope_guard.py`; no battery/link failsafe wired to the operator seat |

So the drone-side data is 60% of the way there. What's missing is:

* A 2D `nav_msgs/OccupancyGrid` publisher (a "slicer" node).
* A GCS application that consumes the ROS topics and renders them in one
  window an operator can drive from.
* An RTH / abort button in that same window.

This document specifies both.

---

## 2. Architecture

```
    ┌──────────────────────────────────────────────────────────┐
    │                        DRONE                             │
    │                                                          │
    │  gzserver / real camera         mavros / FAST-LIO / FUEL │
    │        │                                    │            │
    │        ▼                                    ▼            │
    │  /camera/image_raw          /Fast_LIO/odometry           │
    │  /camera/camera_info        /mavros/state                │
    │                             /sdf_map/occupancy_all       │
    │                             /sdf_map/coverage            │
    │                             /survivors                   │
    │        │                                    │            │
    │        └──────────────┬─────────────────────┘            │
    │                       ▼                                  │
    │        (this design)  MAP_2D_PUBLISHER                   │
    │                       (nidar_mission/map_2d_slicer.py)   │
    │                       │                                  │
    │                       ▼                                  │
    │                    /map_2d  (nav_msgs/OccupancyGrid)     │
    │                                                          │
    │        ┌──────────────────────────────────────┐          │
    │        │      GCS BRIDGE (rosbridge_server)   │          │
    │        │      WebSocket on 9090               │          │
    │        └──────────────┬───────────────────────┘          │
    │                       │                                  │
    │        ┌──────────────┴──────────────┐                   │
    │        │   CAMERA BRIDGE             │                   │
    │        │   web_video_server on 8080  │                   │
    │        │   MJPEG over HTTP           │                   │
    │        └──────────────┬──────────────┘                   │
    └───────────────────────┼──────────────────────────────────┘
                            │  Local 5 GHz Wi-Fi or Ethernet.
                            │  No internet, no LTE, no cloud.
                            ▼
    ┌──────────────────────────────────────────────────────────┐
    │                    LAPTOP (GCS)                          │
    │                                                          │
    │  Foxglove Studio (desktop, offline-capable)              │
    │    ├── map panel (map_2d + survivors + drone pose)       │
    │    ├── image panel (MJPEG stream)                        │
    │    ├── plot panel (battery, mode, coverage %)            │
    │    ├── button panel (arm/disarm, abort, RTH)             │
    │    └── raw-topic panel (Survivor.msg full dump)          │
    └──────────────────────────────────────────────────────────┘
```

### 2.1 Why these choices

* **Foxglove Studio** (desktop) — free, offline-capable, native rosbridge
  client, drop-in panels for maps, images, plots, buttons. It replaces the
  need for us to write any Qt/Electron GUI code. Its `.foxe` extension format
  lets us script small panels (RTH button that publishes to `/mission/abort`,
  etc.) without maintaining a separate frontend build. Fits §7 (fully offline
  once installed).
* **rosbridge_server** — the standard bridge between ROS 1 and any web /
  JavaScript client. Ships in `ros-noetic-rosbridge-server`. WebSocket on
  9090, no external dependencies.
* **web_video_server** — MJPEG stream of any ROS image topic, no rosbridge
  round-trip needed. Foxglove Studio's image panel can consume the stream
  directly, which keeps latency around 100 ms instead of the 500–800 ms
  round-trip through rosbridge for raw image messages. Ships in
  `ros-noetic-web-video-server`.
* **`rosbag` file for post-flight replay** — a second `rosbag record` writing
  to a local disk during the flight. The flightlog under `tools/flightlog/`
  already captures the mission-critical topics; extending it to include
  `/survivors`, `/map_2d` and one throttled image topic gives judges a
  reviewable artifact.

### 2.2 Why NOT the alternatives

* **QGroundControl / Mission Planner** — proprietary GCS aimed at operator-
  driven MAVLink flight, does not natively render arbitrary ROS topics
  (`/survivors`, `/map_2d`). Grafting our data into it would take longer than
  writing our own panels.
* **RViz over VNC** — RViz was measured at 290% CPU during the takeoff run;
  transporting it over VNC adds another 50 ms of latency and needs a
  full desktop session on the GCS side.
* **Custom Flask+React frontend** — I estimate 2–3 weeks to get to feature
  parity with what Foxglove gives us for free. Cost-benefit is against it.
* **MQTT** — extra broker, no advantage over rosbridge for a single-drone
  single-operator setup on a local link.

### 2.3 Physical link

The MAVLink RC link (~433 MHz, low bandwidth) is unusable for video. The
solution is a 5 GHz Wi-Fi bridge between the drone's onboard Jetson and the
operator's laptop, both configured as static IPs on the same subnet with
NO gateway, NO DNS, NO DHCP server — hard-guaranteeing §7.

Recommended: a Ubiquiti / TP-Link AP in ad-hoc mode, or an ESP-based bridge.
Signal envelope of the 15 m arena is well within a single AP's range.

---

## 3. Node-by-node contract

### 3.1 map_2d_slicer (new node, nidar_mission)

**Job:** slice FUEL's 3D voxel map into a 2D occupancy grid, publish at 2 Hz.

**Subscribes:**
* `/sdf_map/occupancy_all` (`sensor_msgs/PointCloud2`) — free+occupied voxels
  from MapROS. Already published by FUEL, no changes required upstream.
* `/sdf_map/coverage` — for the pct field, so the header carries mission
  progress; we've been using this since Phase 3.
* TF: `map` (world-aligned) is the target frame.

**Publishes:**
* `/map_2d` (`nav_msgs/OccupancyGrid`), 2 Hz throttled.
    * `resolution: 0.10` m (matches FUEL's `map_resolution`)
    * `width, height` from the arena bounds + a margin: 160 × 160 cells covers
      the 15×15 m arena with a 0.5 m halo.
    * origin at arena bottom-left, aligned to `map` frame.
* `/map_grid_lines` (`visualization_msgs/MarkerArray`) — the 2 m competition
  grid overlaid on the map, drawn once at startup and latched.

**Algorithm:** for each incoming PointCloud2, iterate points, bucket each
into an (i, j) 2D cell, mark that cell as 100 (occupied) if any occupied
sample lands in it, else 0 (free). Cells with no samples stay -1 (unknown).
`z ∈ [0.3, 1.9]` slice band matches Phase 5 §5.1 recommendation.

**CPU budget:** measured `/sdf_map/occupancy_all` at ~2 Hz with ~15k points
per message on the last run. A single-pass Python bucketing is well under
20 ms. Non-competitive with FAST-LIO, no core pinning needed.

**Config:** `mission_config.yaml` gains a `map_2d:` block with `slice_z_min`,
`slice_z_max`, `publish_hz`, `resolution`. All values are echoed at start-up
by the node, so a wrong slice band is a visible one-liner rather than a
silent drift.

### 3.2 gcs_bridge (existing package, one launch file)

**Job:** expose ROS topics to Foxglove Studio over WebSocket.

**Composed of:**
* `rosbridge_websocket` on port **9090**
* `web_video_server` on port **8080**
* One `topic_tools/throttle` node per outgoing image topic — the raw
  `/camera/image_raw` at 15 Hz is 44 MB/s (raw BGR8, 640×480×3×15). We
  publish `/camera/image_raw/throttled` at 5 Hz, ~15 MB/s, which the MJPEG
  encoder then compresses.

**New launch file:** `launch/gcs_bridge.launch`. Included from
`test_takeoff.sh` behind a `GCS=1` env gate so it does not run in the
CI-shaped smoke runs the flight log records against.

**Security:** rosbridge is unauthenticated by default. On the arena network
that is acceptable (§7 allows no external network), but the Wi-Fi
configuration MUST NOT bridge to any other network. Enforced at the AP.

### 3.3 mission_status_publisher (small aggregator)

**Job:** roll up state the GCS needs from many topics into one compact msg
so the panel does not need N subscriptions.

**Publishes** `/mission/status` (new custom msg) at 2 Hz containing:

```
Header header
string phase              # IDLE, ARM, TAKEOFF, TRANSIT_IN, EXPLORE, ...
                          # matches mission_config.yaml:mission.phases
string flight_mode        # from /mavros/state.mode
bool armed
float32 battery_voltage   # from /mavros/battery
float32 coverage_pct      # index 7 of /sdf_map/coverage
float32 free_area_m2      # index 3
float32 elapsed_s
float32 remaining_s       # clock_limit_s - elapsed
uint32 n_survivors
```

One publisher, one subscriber (Foxglove), and everything the operator sees
at a glance flows through it. Also latched so the panel gets it immediately
on connection.

### 3.4 mission_control_service (RTH / abort)

**Job:** a single ROS service the GCS calls to trigger an operator
intervention. Two RPCs:

* `/mission/abort` (`std_srvs/Trigger`) — hard land in place, disarm.
* `/mission/rth` (`std_srvs/Trigger`) — force the return-to-home leg the EDM
  already implements (see §6 of `mission_optimization_and_arena_entry_plan_
  formatted.md`).

Both are one-shot; the GCS button fires them via rosbridge's `/rosapi/topics`
call-through. Neither takes any parameters, so operator error is bounded
to "did I mean to press that."

---

## 4. Foxglove Studio layout (the operator-facing side)

One `.foxglove-layout` file, checked in under `PLANNING_DOCS/gcs/`, so a
teammate can File → Import Layout and match what we practise with. Panels:

```
┌──────────────────────────────┬──────────────────────────────────────────┐
│                              │                                          │
│    MAP PANEL                 │      IMAGE PANEL                         │
│    - /map_2d (occupancy)     │      - MJPEG from web_video_server       │
│    - /survivor_markers       │      - /camera/image_raw/throttled       │
│    - drone pose from tf      │      - 5 Hz, ~200 kB/frame               │
│    - grid overlay            │                                          │
│    - trail                   │                                          │
│                              │                                          │
├──────────────────────────────┴──────────────────────────────────────────┤
│                                                                          │
│    STATE PANEL (raw messages from /mission/status)                       │
│    ┌────────────────────────────────────────────────────────────────┐   │
│    │  phase: EXPLORE   mode: OFFBOARD   armed: true    batt: 15.4 V │   │
│    │  cov: 48.3%   free: 76.2 m²   t: 214 s   left: 386 s   surv: 3 │   │
│    └────────────────────────────────────────────────────────────────┘   │
│                                                                          │
├──────────────────────────────┬──────────────────────────────────────────┤
│                              │                                          │
│    SURVIVOR TABLE            │      BUTTONS                             │
│    - Full SurvivorArray dump │      [ ABORT   ]  [ RTH  ]  [ ARM ]      │
│    - Sorted by first_seen    │                                          │
│    - grid_x, grid_y visible  │      (calls /mission/abort etc.)         │
│                              │                                          │
└──────────────────────────────┴──────────────────────────────────────────┘
```

Layout follows a **left-to-right narrative**: where is the drone, what is
it seeing, what state is the mission in, who has it found. Panels are
resizable; the RTH button is deliberately kept away from the mouse's
default resting spot to avoid accidental clicks (the competition treats
manual RTH as an intervention that costs points if unnecessary).

---

## 5. Telemetry log for instant debugging

The mission already writes a rich flight log under `logs/runs/`; the design
for **live** debugging telemetry is different — the operator has to be able
to answer "what is going wrong right now" without opening a bag.

### 5.1 Console side (already implemented in this session)

`coverage_reporter.py` prints one line per 20% coverage growth carrying
`pct`, `free_area`, `unknown_area`, plus the still-dark grid cells. That
already gives 5 milestone lines through a healthy 8 min run.

`survivor_detector.py` prints one `[SURVIVOR]` line per confirmed detection,
including ground-truth-vs-tag comparison when `/gazebo/model_states` is
available. It also logs a FAST-LIO smoke-assertion `ROSERROR` at 5 Hz min
floor.

Both lines are already `grep`-friendly. Extending them further:

* Add a **per-second heartbeat** to `mission_telemetry_logger.py` that
  writes CSV columns for coverage_pct, n_survivors, phase, alongside the
  pose it already logs. Consumers: `scripts/analyze_exploration.py` (post-
  hoc grading), and the GCS live panel.

### 5.2 GCS side

Foxglove Studio's built-in **Raw Messages panel** on `/rosout` gives the
operator a scrolling log during the mission. We filter for `nidar_mission`
and `flight_envelope_guard` and pipe it into a fixed panel. That is the
"see-what-just-went-wrong" surface.

For deeper debugging, `rosbag record` a curated topic list to disk on the
drone; the bag can be replayed into the same Foxglove layout post-flight,
so a diagnosis is the same interaction as a live one.

**Curated recording set** (drone-side, ~10 MB/min):

```
/mavros/state
/mavros/local_position/pose
/Fast_LIO/odometry
/planning/pos_cmd
/sdf_map/coverage
/exploration_completed
/survivors
/mission/status
/mission/stop_exploration
/rosout_agg
```

Rate-limited image topic goes to a **separate** bag (~120 MB/min), so a
short run does not fill the SSD; kept only long enough to review post-run
if a detection was disputed.

---

## 6. What has to be built, in dependency order

Estimate is a wall-clock upper bound for one engineer with sim access.

1. **map_2d_slicer.py** — 4 h. Depends on nothing new; consumes existing
   topics. Verify: `rostopic hz /map_2d` at ≥ 1.8 Hz, IoU against arena
   mesh > 0.85 per Phase 5 §5.1.
2. **mission_status_publisher.py** — 3 h. Aggregator over existing topics
   plus /survivors (already published this session).
3. **`launch/gcs_bridge.launch`** — 2 h including the throttle nodes and
   the port-9090 / port-8080 wiring.
4. **`config/gcs.foxglove-layout`** — 3 h. Time-consuming because layout
   choices are experimental until an operator has practised with them.
5. **/mission/abort + /mission/rth services** — 3 h. Handlers in the EDM,
   which already owns the RTH state machine.
6. **Foxglove button-panel extension `.foxe`** — 4 h. Small, but the
   dev-tooling round-trip is the cost.
7. **`docker/gcs.Dockerfile`** — 2 h. Separate image, since the GCS runs
   on the operator laptop and does not need PX4 / Gazebo. Foxglove Studio
   is a native binary and does not go in the container.
8. **Practice runs on the local link** — several hours over multiple
   sessions until the operator's mental model of the panels is solid.

**Total: ~21 engineering hours, one week of practice on top.**

Nothing here blocks Phase 5 (2D map) — item 1 IS Phase 5's slicer, and
building it once serves both the on-drone map generation and the GCS
display. That is why we start there.

---

## 7. Test plan

### 7.1 Bench

Run `nidar_mission.launch` + `gcs_bridge.launch` on the drone-side, open
the layout on the laptop, tick each of the eight `§5` items off:

| # | Requirement | Sim assertion |
|---|---|---|
| 1 | Live mission status | `phase` cycles IDLE→ARM→TAKEOFF→EXPLORE within 30 s of takeoff |
| 2 | Live camera feed | MJPEG panel updates at 5 Hz measured with `wget --spider` on the stream URL |
| 3 | 2D map | `/map_2d` publishes ≥1.8 Hz, occupied cells IoU vs arena mesh > 0.85 |
| 4 | Corridors / rooms | (post-Phase 5 nice-to-have; not blocking) |
| 5 | Grid coord of survivor | Every confirmed survivor's grid_x/grid_y matches its `<actor>` position ± 1 cell (already asserted by `survivor_detector.py`'s ground-truth log) |
| 6 | Tagged locations on map | `/survivor_markers` renders on the map panel at the same position as `/survivors[i].position` |
| 7 | Drone position | tf shown pose in map matches `/Fast_LIO/odometry` position within 0.05 m |
| 8 | Mission progress | `coverage_pct` reaches 90+% before landing |

### 7.2 Failure injection

Kill `rosbridge_websocket` mid-flight; the drone must keep flying, keep
mapping, keep detecting. Reconnect the GCS; latched topics should
re-populate the panels with the full survivor set. **This is the
non-negotiable safety property**: the drone is autonomous, and the GCS is
a passive observer.

### 7.3 Network

`iptables -L` on the arena network's AP: allowlist only the drone's IP
and the operator's IP. Anything outbound to the internet is a DROP.
Verified before every practice session.

---

## 8. Open questions to close before implementation

1. **Which laptop is the GCS?** Foxglove Studio needs 4 GB RAM and modest
   GPU. If it's the same laptop that runs the sim during development, we
   need a lightweight mode that only listens (does not drive Gazebo). One
   flag on `test_takeoff.sh` (`SIM_ONLY=1`) covers it.
2. **AP model.** Any 5 GHz consumer AP works; we should pick one that runs
   OpenWrt so the outbound DROP rule is checked into git alongside the
   rest of the config.
3. **Battery telemetry mapping.** `/mavros/battery` is in Volts, but the
   operator cares about "minutes remaining" more than volts. Empirical
   curve from three bench discharges gets us that mapping; needs a real
   airframe, cannot be sim-derived.
4. **RTH from where the drone currently is, or from the arena centre?**
   The EDM already implements breadcrumb reverse-transit; RTH from
   arbitrary points in the arena needs a graph search over the map. Punt
   to Phase 6 unless there is field time to prototype it.

---

## 9. Non-goals for this document

* MAVLink command-and-control from the GCS. The competition allows only
  start, abort, and RTH — everything else is manual intervention. We do
  not implement waypoint upload, mode changes, or attitude commands.
* Multi-drone coordination. Single vehicle for this year.
* Replaying an in-flight bag file over an rosbridge stream to "rehearse."
  Nice-to-have; explicit non-goal until the primary loop is solid.
* Post-flight report generation. `tools/flightlog/pack.py` already writes
  a review bundle; the GCS's job is live operations, not offline analysis.
