# CPU Bottleneck Fixes — Implementation Plan (2026-08-17)

Turns the 4 findings from today's CPU-bottleneck review into concrete, scoped changes against the current repo
state, plus a dedicated plan for verifying drone position/behavior once the GUI is off (the review's own
open question). Nothing in this doc has been applied yet — this is the plan, pending sign-off on item 2 as the
review itself flagged.

Machine: AMD Ryzen 5 7640HS, **6 physical cores / 12 threads** (confirmed via `lscpu -p`) — this matters for
item 3 below, since naive core-number pinning can accidentally pin two heavy processes to the same physical
core's hyperthread pair, which doesn't actually separate them.

## 1. Default test runs to headless (no GUI/RViz)

**Confirmed accurate.** `scripts/test_takeoff.sh:2` — `GUI_ARG=${1:-true}` — feeds both `gui:=$GUI_ARG`
(`test_takeoff.sh:27`, Gazebo's renderer) and `rviz:=$GUI_ARG` (`test_takeoff.sh:48`). Every crash run documented
today used the default `true`.

**Change**: none needed to the script itself — the headless mode already exists and works
(`./scripts/test_takeoff.sh false 0.0 -6.5 0.1 1.5708`). The actual change is **process discipline**: use `false`
as the default for future diagnostic/verification runs, reserving `true` for the rare case where a human is
actively watching the 3D view live. Recommend flipping the shipped default:
```diff
- GUI_ARG=${1:-true}
+ GUI_ARG=${1:-false}
```
Risk: none — purely a default-argument flip, `true` still available by passing it explicitly. This is the
highest-leverage, zero-risk change in this plan and should go first.

## 2. FAST-LIO Release build (needs explicit sign-off — touches FAST-LIO source tree)

**Confirmed accurate.** `catkin_ws/src/FAST_LIO/CMakeLists.txt:4` — `SET(CMAKE_BUILD_TYPE "Debug")`. Line 8 adds
`-O3` manually, but `CMAKE_BUILD_TYPE=Debug` never defines `-DNDEBUG`, so Eigen's runtime assertion/bounds
checking stays active in FAST-LIO's IKFoM filter (the point-to-plane update running every scan) regardless of
the `-O3` flag. Verified: no `NDEBUG` anywhere in the file.

**Change** (either form, pick one):
```diff
- SET(CMAKE_BUILD_TYPE "Debug")
+ SET(CMAKE_BUILD_TYPE "Release")
```
or leave the source untouched and override at build time (no repo diff at all):
```
catkin build fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```
The build-time-override form is the safer starting point — it validates the theory with zero risk to the
checked-in FAST-LIO tree, and can be promoted to the CMakeLists diff afterward once confirmed. **This does not
touch FAST-LIO's algorithm/logic at all** — it's a compiler-flag-only change (Release vs Debug changes
optimization/assertions, not numerical behavior), so it shouldn't be treated the same as an algorithmic FAST-LIO
change, but it does modify a file inside a package the team has otherwise been treating as hands-off, hence
flagging for explicit go-ahead before applying either form.

**Verification after applying**: `catkin build fast_lio` succeeds, then run one short test and diff the SLAM
trajectory/map output against a known-good Debug-build run (or just visually inspect via the rosbag replay
procedure in section 5) to confirm Release build produces the same map — expected, since this is a compile-flag
change, but worth a sanity check before relying on it going forward.

## 3. CPU pinning (topology-aware — refines the raw finding)

The original finding's core ranges (`0-2`, `3-5`) don't account for hyperthreading: on this CPU, adjacent even/odd
pairs share a physical core (`lscpu -p` confirms: `0,1`→core0, `2,3`→core1, `4,5`→core2, `6,7`→core3, `8,9`→core4,
`10,11`→core5). Pinning FAST-LIO to `0-2` and Gazebo/PX4 to `3-5` would still leave `2,3` (physical core1)
straddled by both allocations. Corrected, physical-core-aligned allocation for this specific stack (not the
Jetson/YOLO/FIESTA case `next_phase_work.md` Step 7.2 was written for):

| Physical core | Threads | Process |
|---|---|---|
| core0 | 0,1 | FAST-LIO (`fastlio_mapping`) — most latency-critical, own core |
| core1 | 2,3 | `gzserver` — physics + sensor sim |
| core2 | 4,5 | `px4` SITL — real-time flight control loop, needs low jitter |
| core3 | 6,7 | MAVROS + `flight_envelope_guard.py` + `relay_odometry.py` — safety-critical bridge chain |
| core4-5 | 8-11 | unpinned pool: `exploration_manager` (FUEL), `rosout`, telemetry/rosbag logging, OS overhead |

**Change**: rather than wrapping the `roslaunch` invocations (which would pin every sub-process launched under
them to the same set, defeating the separation — `mavros_posix_sitl.launch` starts gzserver+px4+mavros as one
tree), pin already-running processes by PID after each is confirmed up. This needs no changes to any PX4/Gazebo/
FAST-LIO/FUEL launch files — additions live entirely in `test_takeoff.sh`:

```bash
pin_process() {
    local pattern="$1" cores="$2" pid
    pid=$(pgrep -f "$pattern" | head -1)
    if [ -n "$pid" ]; then
        taskset -pc "$cores" "$pid" >/dev/null 2>&1 && echo "Pinned $pattern (pid $pid) -> cores $cores"
    fi
}
```
Call sites (after each process is confirmed alive by the script's own existing checks):
- after `verify_topic "/Fast_LIO/odometry" ...` succeeds: `pin_process "fastlio_mapping" "0,1"`
- after the MAVROS-connected check succeeds: `pin_process "gzserver" "2,3"` and `pin_process "bin/px4" "4,5"`
  and `pin_process "mavros_node" "6,7"`
- after `flight_envelope_guard.py` and `relay_odometry.py` are started: `pin_process "flight_envelope_guard.py" "6,7"`
  and `pin_process "relay_odometry.py" "6,7"`

Optional (only if pinning alone isn't enough): raise FAST-LIO's scheduling priority so it preempts non-real-time
work — `chrt -f -p 10 $(pgrep -f fastlio_mapping)`. Start without this; add only if FAST-LIO still shows stalls
under load with pinning alone.

**Risk**: low — `taskset -p` on a running process fails safely (non-fatal, script continues) if the process
name pattern doesn't match or the PID already exited; wrap each call as shown (`&&`, not required to succeed).

## 4. Test-run process discipline (no code change — a documented convention + one script guard)

The crash investigation docs from today already name concurrent diagnostic commands (repeated `rosservice call`,
manual `tail`/`grep` polling, ad hoc Python odometry checks) as a plausible contributor to the exact CPU
contention being investigated. Fix is procedural, not code:

- Treat a timed test run as a **single-purpose window**: start it, then do not run parallel `rostopic echo`,
  manual polling loops, or analysis commands on the same machine until the run ends.
- Do all live observation through the one persistent, already-lightweight channel that's supposed to be running
  anyway: `mission_telemetry_logger.py` (already a single subscriber-based node, not a polling loop — see
  section 5).
- Do all forensic/diagnostic digging (CSV parsing, log grepping, mesh analysis, etc.) **after** the run ends and
  the sim processes are torn down, not concurrently.
- One small script guard worth adding: have `test_takeoff.sh` print current `uptime` load average right before
  launching Gazebo, so every run's log records the load-average baseline it started from — makes it easy to spot
  in hindsight whether a given run's stall correlates with pre-existing load versus something the run itself
  caused. Cheap, non-invasive:
  ```bash
  echo "Load average at test start: $(uptime | awk -F'load average:' '{print $2}')"
  ```

## Implemented (2026-08-17, revised)

Per direction: GUI stays **on** (item 1 not applied — `GUI_ARG` default left as `true`), no new observability/
debugging scaffolding added (dropped the headless-verification section below as moot — GUI provides live visual
verification, so the one existing method, `mission_telemetry_logger.py`, stays as-is). Items 2 and 3 applied:

**Item 2 — FAST-LIO Release build.** Applied directly: `catkin_ws/src/FAST_LIO/CMakeLists.txt:4`,
`CMAKE_BUILD_TYPE` changed from `"Debug"` to `"Release"`. Verified via `catkin build fast_lio` — succeeds
cleanly, only pre-existing PCL deprecation warnings, unrelated to this change.

**Item 3 — CPU pinning.** Implemented in `test_takeoff.sh`, wired at each process's actual startup point (not a
blanket wrapper around the `roslaunch` calls, which would have pinned every sub-process launched under them to
the same set): `pin_process()` pins `gzserver`→cores 2,3, `bin/px4`→4,5, `mavros_node`→6,7,
`fastlio_mapping`→0,1, `relay_odometry.py`→6,7, `flight_envelope_guard.py`→6,7 (cores 8-11 left unpinned for
FUEL/exploration_manager, GUI rendering, and OS overhead).

Two real correctness bugs were found and fixed while validating this live, not just planned:

1. **Wrapper/child PID mismatch.** `gzserver` is started by roslaunch through a `/bin/sh` wrapper that then
   forks the actual `gzserver` binary as a *separate* child PID. The first version of `pin_process` matched only
   the first `pgrep` hit (the lightweight wrapper) — the real, heavy `gzserver` process was never pinned at all.
   Fixed by pinning every matching PID, not just the first.
2. **Per-thread affinity, not per-process.** `taskset -pc <cores> <pid>` only sets affinity for the single
   thread/LWP identified by that PID — a multithreaded process (`gzserver` runs ~80 threads for physics,
   rendering, and ROS transport) keeps all its *other* threads unrestricted unless each is pinned individually.
   Fixed by enumerating `/proc/<pid>/task/*` and pinning every existing thread. Verified authoritatively via
   `/proc/<tid>/status`'s `Cpus_allowed_list` field (not `ps`'s `psr` column, which only reflects the last CPU a
   thread happened to run on and can show stale values for idle threads well after their affinity mask has
   actually changed — confirmed all 81 gzserver threads showed `Cpus_allowed_list: 2-3` after pinning, even
   though `ps -eLo psr` still showed historical residue on other cores for several seconds).

**Remaining gap, addressed with a re-pin loop.** A one-shot pin only catches threads that exist at the moment it
runs — `gzserver` continues spawning new worker threads afterward, which start unpinned until caught. The
architecturally correct fix is a cgroup (cpuset) rather than per-thread `taskset`, since moving a process into a
cpuset-constrained cgroup makes the constraint inherited automatically by all its current *and future* threads/
children. Checked: `/sys/fs/cgroup` is mounted **read-only in this environment, even with root** (`sudo mkdir`
under it fails with "Read-only file system"), so cgroups aren't available here. Practical fallback implemented
instead: `scripts/cpu_repin_loop.sh`, a lightweight loop (3s interval) that re-applies the same per-thread pin to
all six targets for the duration of the run, started in the background by `test_takeoff.sh` right after the guard
comes up, and cleaned up by the next run's existing top-of-script `pkill` cleanup block (same pattern already
used for every other process the script starts).

**Verified live**: full test run with GUI on, pinning confirmed holding via `/proc/<tid>/status` on the actual
running processes (not just inspecting the script). Stack came up cleanly end-to-end (MAVROS connected, FAST-LIO
topics active, position locked, armed, took off, FUEL launched and started exploring) with pinning active
throughout.

## Dropped from this pass (per direction)

- Item 1 (headless default) — explicitly not wanted; GUI stays on.
- Item 4 (test-run process discipline / `uptime` baseline print) — not added; keeping to the single existing
  telemetry method rather than adding more logging surface.
- The "headless verification plan" section below is now moot with GUI staying on — kept in this doc for
  reference only, in case headless mode is revisited later, but nothing in it was implemented this pass.

---

# Headless verification plan: how to know where the drone is without the GUI

Disabling Gazebo's 3D view and RViz removes the only channel that's been used so far to *see* the drone directly.
Three of the four replacement channels already exist in the repo from today's and earlier sessions' work — this
section is about using them systematically, not building new tooling, plus one new piece (wiring
`record_mission.sh` into the test flow) to close the "I want to actually look at it" gap.

## Channel 1 — live, human-readable position printout (already running every test)

`scripts/mission_telemetry_logger.py` already subscribes directly to `/mavros/local_position/pose`,
`/mavros/extended_state`, `/Fast_LIO/odometry`, and `/planning/pos_cmd`, and prints one correlated line every 5
seconds to the console *and* to `logs/mission_telemetry_*.csv` — completely independent of any renderer:
```
[Telemetry t+65s] Landed=IN_AIR EKF(px4,camera_init)=(0.62,6.27,1.05) yaw=1.05 FastLIO(...)=(...) diverge=0.01m Vel=(...) Mode=OFFBOARD Armed=True | FUEL target(...)=(...) age=0.3s [CHANGED] | Guard: ACCEPT ...
```
This is already wired into `test_takeoff.sh` (the last foreground command in the script) and needs no changes —
it's the primary "is the mission healthy right now" channel with the GUI off.

## Channel 2 — on-demand exact ground truth (used throughout today's crash investigations)

`rosservice call /gazebo/get_model_state "model_name: 'iris_vlp16'"` returns the physics-authoritative pose
directly from Gazebo's simulation state, independent of any estimator (FAST-LIO/EKF2) and independent of
whether the GUI is rendering. This is exactly how today's two real crashes were confirmed (drone resting at
(-1.74, 3.18, 0.055) and (-6.28, -6.10, 0.055) respectively) — a one-off, precise "where is it right now"
check, usable at any point during or after a headless run, with effectively zero CPU cost (Gazebo tracks this
state regardless of whether it's being drawn).

## Channel 3 — full diagnostic trail (unthrottled, post-hoc)

`flight_envelope_guard.py` already writes every single command decision to
`logs/flight_envelope_guard_<timestamp>.csv` at full 20Hz resolution (FUEL's raw command, the world-frame
conversion, the guard's accept/reject decision, and the real measured pose) — this is what let today's
investigations reconstruct exact altitude/position timelines down to the sample after the fact. No GUI
dependency at all; already running on every test.

## Channel 4 — recorded rosbag for after-the-fact visual replay (exists, not yet wired in)

`scripts/record_mission.sh` already exists and records exactly the topics needed for a full visual
reconstruction: `/Fast_LIO/odometry`, `/cloud_registered`, `/mavros/imu/data`, `/mavros/local_position/pose`,
`/mavros/state`, `/mavros/setpoint_raw/local`, `/mavros/setpoint_position/local`, `/planning/pos_cmd`, `/tf`,
`/tf_static`, `/exploration_node/frontier_num`. It is **not currently invoked by `test_takeoff.sh`** — this is
the one actual gap to close.

**Change**: add one line to `test_takeoff.sh`, started in the background right before the FUEL trigger is
published (same point `mission_telemetry_logger.py` is launched from), and killed by the same cleanup path as
everything else:
```bash
/home/developer/NIDAR/scripts/record_mission.sh > /tmp/rosbag_record.log 2>&1 &
ROSBAG_PID=$!
```
**Usage after a headless run**: bring up `roscore` + `rviz` (or just `rqt_bag`) *outside* the timed test window
(satisfies item 4 above — no GUI competing for CPU during the actual run) and play the bag back at normal or
reduced rate:
```bash
roscore &
rosbag play /home/developer/NIDAR/rosbags/nidar_mission_<timestamp>.bag --clock
rviz -d <existing rviz config from launch/fast_lio/>
```
This gives the full 3D visual reconstruction — point cloud, trajectory, TF tree, setpoints — exactly as if the
GUI had been on live, but reviewed after the fact instead of competing with FAST-LIO/PX4 for cores during the
actual test.

## Summary: what to check, and when

| When | Channel | What it answers |
|---|---|---|
| Continuously during the run | telemetry logger console/CSV (channel 1) | "Is the mission progressing / healthy right now?" |
| At a specific suspicious moment | `get_model_state` (channel 2) | "Where is the drone *actually* at this exact instant?" |
| After the run, forensic timeline | guard CSV (channel 3) | "What did every single command look like, sample by sample?" |
| After the run, visual confirmation | rosbag replay (channel 4) | "What did the map/trajectory/point cloud actually look like?" |

No new tooling is required beyond wiring `record_mission.sh` into `test_takeoff.sh` — everything else needed to
verify drone position headlessly already exists and was exactly how today's two crashes were diagnosed without
ever needing the live 3D view.
