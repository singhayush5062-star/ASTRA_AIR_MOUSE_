# NIDAR AirMouse — Package Split for a Production-Ready Repo

**Date:** 2026-09-11
**Status:** design doc, no code changes yet
**Audience:** anyone about to add a subsystem to this repo and wondering "which package does it belong in"

---

## 1. Why split at all

Today's `catkin_ws/src/nidar_mission/` is a single package that owns nine loosely-coupled subsystems:

| Concern                     | Files today                                                       |
|-----------------------------|-------------------------------------------------------------------|
| Central config              | `config/mission_config.yaml`, `scripts/apply_mission_config.py`   |
| Vehicle / SDF generation    | rendered into `x500_vlp16.sdf` by the generator                   |
| Mapping frame (FAST-LIO)    | rendered into `config/fast_lio/nidar_sim.yaml`                    |
| Planner wiring (FUEL)       | rendered into `algorithm.xml`, `nidar_fuel_upstream.launch`       |
| Safety envelope             | `config/flight_envelope_guard.yaml`, `scripts/flight_envelope_guard.py` |
| Mission FSM + entry + return | `scripts/entry_detection_module.py`                              |
| Perception                  | `scripts/survivor_detector.py`, `msg/Survivor*.msg`, `models/`    |
| Coverage / telemetry        | `scripts/coverage_reporter.py`, `scripts/mission_telemetry_logger.py`, `scripts/strict_monitor.py` |
| Launch orchestration        | `launch/nidar_mission.launch`, top-level `launch/*.launch`, `scripts/test_takeoff.sh` |

Every subsystem depends on `mission_config.yaml`. Every subsystem also imports whatever else is in `nidar_mission/scripts/`. The result today:

- A person changing the detector rebuilds the guard (same `CMakeLists.txt`).
- Nothing forces `apply_mission_config.py` to be run on the machine that renders SDF and the machine that runs the FSM.
- Test builds pull the whole tree because the smallest linkable unit is "everything".
- New team members can't tell which script is authoritative until they trace it.

The goal of the split is not aesthetic. It is to make **who owns what** answerable in one `ls`, and to make an on-vehicle build possible without the sim-only bits (perception model, arena world, generator).

---

## 2. Proposed layout

```
catkin_ws/src/
├── nidar_msgs/                    # message-only, zero code
│   ├── msg/
│   │   ├── Survivor.msg           (moved from nidar_mission)
│   │   ├── SurvivorArray.msg
│   │   ├── MissionStatus.msg      (new, see §4.7)
│   │   └── CoverageReport.msg     (replaces Float64MultiArray)
│   ├── srv/
│   │   ├── AbortMission.srv       (new — GCS panic button)
│   │   └── ReturnHome.srv         (new — GCS force RTH)
│   ├── CMakeLists.txt
│   └── package.xml
│
├── nidar_config/                  # single source of truth + generator
│   ├── config/
│   │   ├── mission_config.yaml    (moved from nidar_mission/config)
│   │   ├── arena_grid.yaml        (moved from repo root config/)
│   │   ├── flight_envelope_guard.yaml
│   │   └── smoke_thresholds.yaml  (new, see nidar_qa)
│   ├── scripts/
│   │   ├── apply_mission_config.py  (renders the four derived files)
│   │   └── preflight_check.py       (new, §0.2 of phase plan)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  (nothing from this repo)
│
├── nidar_sim/                     # SIM-ONLY: worlds, models, arena
│   ├── worlds/
│   │   └── nidar_competition.world  (moved from repo root)
│   ├── models/
│   │   ├── arina_nidar/
│   │   ├── launch_pad/
│   │   ├── tfmini_lidar/
│   │   └── velodyne_vlp16/
│   ├── launch/
│   │   └── nidar_sim.launch       (spawns Gazebo, world, PX4 SITL)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_config
│
├── nidar_platform/                # vehicle SDF + PX4 airframe overlay
│   ├── models/x500_vlp16/         (rendered by nidar_config generator; still
│   │                               committed so a clean checkout has a
│   │                               working file without running the generator)
│   ├── px4_overlay/               (the "overlay/" from phase plan §0.4)
│   │   ├── ROMFS/.../1025_gazebo-classic_x500_vlp16
│   │   └── Tools/.../iris/iris.sdf.jinja  (motor constants)
│   ├── launch/
│   │   ├── mavros_posix_sitl.launch  (moved from PX4 vendored tree)
│   │   └── vehicle.launch            (composes mavros + relay + guard)
│   ├── scripts/
│   │   ├── relay_odometry.py         (FAST-LIO -> PX4 EKF2)
│   │   └── build_px4.sh              (moved from repo root)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_config
│
├── nidar_slam/                    # FAST-LIO glue
│   ├── config/fast_lio/           (rendered by generator; committed)
│   ├── launch/nidar_mapping.launch  (moved from launch/fast_lio/)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  fast_lio (upstream package), nidar_config
│
├── nidar_planner/                 # FUEL wiring
│   ├── launch/nidar_fuel_upstream.launch  (moved from launch/)
│   ├── config/                    (any planner-specific overrides)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  fuel (upstream), nidar_config
│
├── nidar_safety/                  # flight envelope guard (safety-critical)
│   ├── scripts/
│   │   └── flight_envelope_guard.py
│   ├── config/                    (defaults; canonical values live in nidar_config)
│   ├── launch/guard.launch
│   ├── test/                      (unit tests: axis conversion, envelope math)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_msgs, nidar_config
│   RUNTIME NOTE: this package alone MUST stay < 100 SLOC of runtime logic and
│   have a unit-test suite; every other package's failure is worse if the guard
│   is not the last line of defence.
│
├── nidar_mission/                 # the FSM only — no side-quests
│   ├── scripts/
│   │   ├── mission_manager.py     (renamed from entry_detection_module.py)
│   │   └── mission_telemetry_logger.py
│   ├── launch/nidar_mission.launch  (composes the mission layer only, not
│   │                                 the full stack)
│   ├── test/                      (state-machine tests with mocked topics)
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_msgs, nidar_config, nidar_safety
│
├── nidar_perception/              # camera + detector + tracker
│   ├── scripts/survivor_detector.py
│   ├── models/detection/          (moved from repo root models/)
│   │   └── PERSON_DETECTION_MODEL_V3/
│   ├── launch/detector.launch
│   ├── test/
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_msgs, nidar_config
│
├── nidar_map2d/                   # NEW (Phase 5): live 2D map + tagging
│   ├── scripts/
│   │   ├── map_2d_slicer.py         (nav_msgs/OccupancyGrid publisher)
│   │   ├── grid_overlay.py          (MarkerArray for grid lines + labels)
│   │   └── survivor_tag_publisher.py  (MarkerArray for tagged survivors)
│   ├── launch/map2d.launch
│   ├── test/
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_msgs, nidar_config
│
├── nidar_gcs/                     # NEW (Phase 6): operator station
│   ├── scripts/
│   │   ├── gcs_bridge.py            (rosbridge_server config)
│   │   └── mission_status_publisher.py
│   ├── foxglove/nidar_layout.json
│   ├── launch/gcs.launch
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  nidar_msgs, nidar_config, rosbridge_server, web_video_server
│
├── nidar_qa/                      # smoke test + preflight + acceptance
│   ├── scripts/
│   │   ├── ci_smoke.sh              (phase plan §0.3)
│   │   ├── verify_fix_parity.sh     (moved from repo root scripts/)
│   │   ├── run_acceptance.py        (phase plan §11)
│   │   └── analyze_run.py           (moved from repo root scripts/)
│   ├── config/smoke_thresholds.yaml
│   ├── test/
│   ├── package.xml
│   └── CMakeLists.txt
│   Depends on:  everything (this is the top-level integration test package)
│
└── nidar_bringup/                 # top-level composition + docs
    ├── launch/
    │   ├── nidar_full_sim.launch    (moved logic from test_takeoff.sh)
    │   ├── nidar_full_hardware.launch
    │   └── mission_only.launch
    ├── scripts/
    │   └── start_mission.sh         (single entry point for competition)
    ├── docs/README.md
    ├── package.xml
    └── CMakeLists.txt
    Depends on:  every runtime package
```

**Dependency graph is a DAG** (this matters — no cycles):

```
                       nidar_msgs
                            ^
                            |
                       nidar_config
                     /    |    \      \
       nidar_sim    /     |     \      \
       nidar_platform  nidar_slam  nidar_planner  nidar_safety
                                            \        |
                                             \       |
                                              nidar_mission
                                                     ^
                                                     |
                            nidar_perception    nidar_map2d
                                     \             /
                                      \           /
                                       nidar_gcs
                                            ^
                                            |
                                       nidar_bringup
                                            ^
                                            |
                                       nidar_qa
```

---

## 3. What each package DOES and DOES NOT own

### Rules that override "obvious" placement

1. **A file has one owner.** If `flight_envelope_guard.yaml` currently exists in both `nidar_mission/config/` and `config/`, exactly one of those is deleted as part of this migration.
2. **Config is a package, not a directory.** `nidar_config` owns *the values*; `nidar_platform`, `nidar_slam`, etc. each own the *rendered artefact* that their runtime loads. The generator runs at build time; the rendered file is committed so a clean clone works, and `preflight_check.py` asserts it is current.
3. **Sim-only never links to runtime.** `nidar_sim` is not on the ROS runtime deps of `nidar_bringup`'s hardware launch. This is enforced by the two `nidar_full_*.launch` files, not by wish.
4. **The safety guard is its own package.** It has one job. Its build is not affected by the detector's Ultralytics dep. If MoveIt bloats CI tomorrow, the guard still builds.
5. **Messages are message-only.** `nidar_msgs` has zero `.py` and zero C++. It exists so the on-vehicle Jetson does not need to pull the model files just to subscribe to `/survivors`.

### Per-package do / don't

| Package         | OWNS                                                     | DOES NOT OWN                                                |
|-----------------|----------------------------------------------------------|-------------------------------------------------------------|
| nidar_msgs      | .msg / .srv definitions                                  | any code, any config                                        |
| nidar_config    | mission_config.yaml, generator, preflight_check          | *runtime* config the generator produces (that lives in the pkg that reads it) |
| nidar_sim       | Gazebo worlds + arena/pad/lidar models + sim launch      | vehicle SDF (that's nidar_platform)                         |
| nidar_platform  | vehicle SDF, PX4 overlay, MAVROS, odometry relay         | flight-envelope guard, mission FSM                          |
| nidar_slam      | FAST-LIO glue and its config                             | any consumer of /Fast_LIO/*                                 |
| nidar_planner   | FUEL wiring and its config                               | any consumer of /planning/pos_cmd                           |
| nidar_safety    | flight envelope guard, guard tests                       | mission FSM, RTH, LAND (those are decisions, not safety)    |
| nidar_mission   | mission FSM (TAKEOFF..DESCEND..LAND), telemetry logger    | detector, camera, guard, planner                             |
| nidar_perception| camera bringup, detector, tracker, model weights         | survivor cell math (that's nidar_map2d)                     |
| nidar_map2d     | 2D map slicer, grid overlay, survivor tags on grid       | 3D SLAM, detector inference                                 |
| nidar_gcs       | rosbridge, video, Foxglove layout, mission_status pub    | anything that owns motion                                   |
| nidar_qa        | smoke test, parity check, acceptance runner              | any runtime node                                            |
| nidar_bringup   | full-stack .launch and start_mission.sh                  | any node's implementation                                   |

---

## 4. Concrete migration steps (roughly one PR per numbered item)

Each of these is a **mechanical move + rename**, verified by re-running the parity check and the sim, before the next one begins. Do NOT combine them — a single PR that touches every package is not reviewable.

1. **Create `nidar_msgs`** with existing Survivor/SurvivorArray. Update `nidar_mission/package.xml` to depend on it. No runtime change. Verifies the message split.
2. **Create `nidar_config`**. Move `mission_config.yaml`, `arena_grid.yaml`, `flight_envelope_guard.yaml`. Update every consumer's `rosparam` path. Move the generator. Regenerate. Parity check must still pass.
3. **Create `nidar_safety`**. Move `flight_envelope_guard.py`. Its unit tests come with it. No runtime change if the launch order is preserved.
4. **Create `nidar_platform`**. Move `relay_odometry.py`, `build_px4.sh`. Move MAVROS + PX4 overlay contents from repo root. The vehicle SDF is already generated into the vendored PX4 tree; move it OR link to it under `nidar_platform/models/x500_vlp16/`.
5. **Split `nidar_planner` and `nidar_slam`** off of the top-level `launch/` directory.
6. **Create `nidar_sim`**. Move `nidar_competition.world` + arena/pad/lidar models. Move `nidar_sim.launch`.
7. **Rename `entry_detection_module.py` → `nidar_mission/scripts/mission_manager.py`**. This is renaming a file, not rewriting it — the FSM logic is fine. The name mattering because six planning docs already call it `mission_manager.py`.
8. **Create `nidar_perception`**. Move `survivor_detector.py`, `models/detection/`. Update image + camera_info topic subscribers unchanged.
9. **Create `nidar_map2d`** (Phase 5, this session, in parallel).
10. **Create `nidar_gcs`** (Phase 6).
11. **Create `nidar_qa`**. Move `scripts/verify_fix_parity.sh`, `scripts/analyze_run.py`. Add the smoke test + preflight from the phase plan.
12. **Create `nidar_bringup`**. Move `test_takeoff.sh` logic into `nidar_full_sim.launch`. `test_takeoff.sh` becomes a thin wrapper that calls the launch and sets env.

**Effort estimate**: 1 PR/day at low risk, ~2 weeks calendar total. Do not compress by combining PRs.

---

## 5. Guardrails so the split doesn't rot

A layout is only worth having if it stays true. Three enforcement mechanisms:

### 5.1 CI: package graph must remain acyclic

Add a job to CI that runs `catkin_tools list --deps` and asserts:
- `nidar_config` depends on nothing internal.
- `nidar_safety` depends on `nidar_msgs, nidar_config` and nothing else internal.
- No package depends on `nidar_sim` except `nidar_bringup` (via the sim-only launch).
- `nidar_msgs` has no Python or C++ sources.

Fail the build if any assertion breaks. This is the same idea as the current XML well-formedness check — encoded, not documented.

### 5.2 Preflight: assert what's on disk matches what will run

`nidar_config/scripts/preflight_check.py` (phase plan §0.2) additionally checks:
- No file at repo root shadows a package file (regression guard for the pre-split state).
- Every rendered artefact's on-disk hash matches what the generator would emit from the current `mission_config.yaml`. Failure means someone hand-edited a rendered file.

### 5.3 Ownership file per package

Each `package.xml` gains a `<maintainer email="...">Name</maintainer>` block, and each package's `README.md` names the human accountable for it. When a bug crosses two packages, the two maintainers meet — but the tree tells you who to invite in the first place.

---

## 6. Cost / benefit sanity check

**Cost**: ~2 weeks of PRs, some flakiness while migrating (mitigated by keeping the sim green as the gate — you don't merge a migration PR that reddens the smoke test).

**Benefit** (measured, not aspirational):
- **On-vehicle build shrinks**. Today `catkin build nidar_mission` also compiles the guard, the detector, the messages, the telemetry logger, and every launch file. After the split the on-vehicle bringup builds `nidar_msgs, nidar_config, nidar_safety, nidar_slam, nidar_planner, nidar_mission, nidar_perception, nidar_platform, nidar_map2d, nidar_gcs, nidar_bringup` and skips `nidar_sim` — no Gazebo world, no arena mesh, no models, no test data.
- **Sim → hardware becomes a launch swap**, not a repo edit. `nidar_full_sim.launch` vs `nidar_full_hardware.launch` differ only in whether they include `nidar_sim` / real MAVROS URL. Nothing else.
- **New team member onboarding**. A "where does X live" question ends at the first `ls src/`. Today it ends after ten minutes of grep.
- **CI parallelism**. Independent packages test in parallel. `nidar_safety`'s test suite doesn't wait on Ultralytics to build.
- **A guard bug is a `nidar_safety` PR**, not a mission-layer PR. Reviewer knows what to focus on.

---

## 7. What this doc does NOT decide

- **Whether to use Python or C++ in `nidar_perception`**. Today it's Python + Ultralytics. On Jetson at competition time it may be C++ + TensorRT. Both variants can live in this package; the choice is orthogonal to the split.
- **Whether the FSM stays a giant Python file or becomes SMACH**. The rename in step 7 is a pure mv. Any FSM refactor is a separate PR under `nidar_mission/`.
- **Whether to move to ROS 2**. Every package name and dependency here is ROS 1. A future ROS 2 port would preserve this layout and change only build system files. The split makes such a port easier, not harder — porting one package at a time is possible only if the packages exist.
- **Vendored PX4's fate**. Phase plan §0.4 decides that; this doc only names the package the overlay ends up in.

---

## 8. Anti-patterns to avoid during the split

- **Do not create `nidar_common` or `nidar_utils`**. Every "utils" package in every ROS repo becomes the new kitchen sink. If a utility is shared between two packages, put it in whichever package uses it more; if a third package needs it later, promote it to `nidar_config` or `nidar_msgs` at that point, not preemptively.
- **Do not add a wrapper launch file per package that just includes the "real" launch**. If `nidar_perception/launch/detector.launch` is what runs the detector, don't add `nidar_bringup/launch/detector_wrapper.launch` that includes it. Consumers `<include>` the real one directly.
- **Do not split `mission_config.yaml` per package**. The whole point of that file is that arena, vehicle, planner and mission see the same numbers. Splitting it undoes the 2026-09-04 dedup work.
- **Do not add a new package "just for this one script"**. A new package is a new build target, a new maintainer, a new place for state to hide. Only split when there is either a clear owner or a clear runtime asymmetry (e.g. sim-only, hardware-only, safety-critical).
- **Do not change interface contracts during the split**. A pure package split moves files and updates `package.xml` — it does NOT rename topics, rearrange launch arg names, or reformat YAML. Save behavioural changes for later PRs so review can distinguish "moved" from "modified".

---

## 9. Where this fits with the phase plan

The phase plan (`nidar_phase_plan_to_mission_complete.md`) is a *phase* sequence — what capability lands next. This doc is an *architecture* — where each capability lives. Both are true at the same time:

| Phase plan item                       | Package it will land in       |
|---------------------------------------|-------------------------------|
| P0.2 `preflight_check.py`             | nidar_config                  |
| P0.3 `ci_smoke.sh`                    | nidar_qa                      |
| P2.1 guard state validation           | nidar_safety (done pre-split) |
| P2.4 failsafes                        | nidar_mission + nidar_safety  |
| P4 survivor detection                 | nidar_perception              |
| **P5 2D map + tagging (this session)**| **nidar_map2d (created here)**|
| P6 GCS                                | nidar_gcs                     |
| P7 mission FSM                        | nidar_mission                 |
| P8 acceptance suite                   | nidar_qa                      |

Implementing Phase 5 today directly under `nidar_mission/scripts/` is fine — the file(s) move to `nidar_map2d` in the split. The location of a script under this monolith is a rehearsal for where the package boundary will go.

---

## 10. Recommendation

Do the split **after** Phase 5–7 land, not before. Two reasons:

1. **You do not yet know the shape of `nidar_map2d`, `nidar_gcs`, `nidar_mission`'s FSM API**. Splitting on today's shape locks in guesses. Wait until each subsystem has been used end-to-end at least once.
2. **The smoke test doesn't exist yet.** Splitting without one means every migration PR is "did we break something?" instead of "the smoke test is green, ship it". Land P0.3 first.

Target order: **P0.3 (smoke)** → **P4, P5, P6, P7** (features in the monolith) → **package split** (steps 1–12) → **P8 acceptance** (runs on the split repo, so the split is what ships).
