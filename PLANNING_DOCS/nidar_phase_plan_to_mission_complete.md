# NIDAR AirMouse — Phase Plan from "Mapping Works" to "Mission Complete, Flawless in Sim"

**Repo audited:** `singhayush5062-star/ASTRA_AIR_MOUSE_` @ `852482e5` (shallow clone, default branch)
**Reference:** `Mission Brief — NIDAR AirMouse` (Track 1, Drone Innovation)
**Date:** 2026-09-04
**Status of this document:** proposal as originally written. **Updated 2026-09-04 (same day):** most
of Phase 0/1/2's concrete items have since been applied and live-verified — see the Status column in
Appendix A for the item-by-item record, and `PLANNING_DOCS/guard_state_validation_and_wall_stuck_fix_2026-09-04.md`
for the guard/physics fixes this audit's A4 finding led to. §3.1 (promote the parametric
`nidar_airmouse_arena.sdf`) will **not** be implemented — superseded by testing against multiple
user-supplied arena STL variants instead; see the note at §6/3.1 below.

---

## 0. Read this first — the framing

The diagnosis you brought in is correct, and I verified every claim in it against the actual tree
(receipts in §1). But the diagnosis understates the problem in one direction and overstates it in
another, and both matter for how the phases get ordered.

**Understated:** "two divergent copies of the vehicle SDF" is not two files. It is a *pattern* that
appears at least **eight** times in this repo — vehicle SDF, arena geometry, odometry relay, FAST-LIO
launch, FAST-LIO config, vehicle URDF, RViz profile, and the planning docs themselves. Five files at
the repo root are diverged shadows of the five files that actually load. `PLANNING_DOCS/` documents
four scripts that do not exist. This is not tidiness. It means that **for any given behaviour, no one
on the team can currently answer "which file produced that?" without running a trace.** That is the
single root cause behind all four session failures.

**Overstated:** the smoke test is the highest-value *next* step, but it is not the highest-value step
*first*. A smoke test that asserts against a stack with eight ambiguous sources of truth will produce
green runs that mean nothing, because you cannot tell which copy of the config the green run
exercised. **Deduplication must precede assertion by about a day.** Otherwise you build a very fast
way to be confidently wrong.

**The other thing worth saying plainly:** you are roughly 40% of the way to the mission brief, not
80%. Exploration + SLAM + guard is the *substrate*. The scored deliverables — survivor detection,
grid-box tagging, live 2D map, live camera feed, GCS display, autonomous exit — do not exist in the
repo in any form. There is no camera on the vehicle. There is no `nav_msgs/OccupancyGrid` publisher.
There is no GCS. Phases 5–9 below are net-new engineering, not integration. Plan calendar accordingly.

---

## 1. Verified findings (receipts)

Each of these was confirmed by reading the file, not by reading a doc about the file.

### 1.1 The divergence metric is arithmetically wrong — and I can tell you by exactly how much

`scripts/mission_telemetry_logger.py:151`

```python
divergence = ((px - lx) ** 2 + (py - ly) ** 2 + (pz - lz) ** 2) ** 0.5
```

`px, py, pz` come from `/mavros/local_position/pose` — **ENU** (x=East, y=North).
`lx, ly, lz` come from `/Fast_LIO/odometry` — **camera_init** frame.

The stack's own conversion between those two frames is written down twelve files away, in
`scripts/flight_envelope_guard.py:244-246`:

```python
c_xc = self.current_pose.pose.position.y     # xc =  y
c_yc = -self.current_pose.pose.position.x    # yc = -x
```

So the two frames differ by a 90° yaw. Subtracting them without rotating gives, for a vehicle at
radius `r` from the origin:

```
error = sqrt((a-b)² + (a+b)²) = √2 · r
```

**The logger does not measure divergence. It measures √2 × distance-from-origin.** Z is unaffected
(both frames share Z), so the metric is exactly zero at the pad and grows linearly the moment the
drone moves. Your reported `24.97 m` back-solves to `r ≈ 17.66 m` — which is *outside a 15×15 m
arena*, i.e. the vehicle was genuinely somewhere it should not have been, but the number attached to
it described nothing.

Compounding it, the print at `mission_telemetry_logger.py:172` labels the PX4 pose
`EKF(px4,camera_init)` — **the label asserts a frame conversion that the code never performs.**
That label is why this survived review.

The `> 0.5` threshold at line 169 therefore fires at 35 cm of lateral travel. Every flight past the
first half-metre has been printing `[EKF/FASTLIO DIVERGED!]`.

### 1.2 The guard validates commands and *declines to act* on state

`scripts/flight_envelope_guard.py:242-255`. Block 4 reads the actual vehicle pose, converts it
correctly, checks it against the envelope — and on failure calls `rospy.logwarn_throttle(...)`, then
falls through to:

```python
return True, "ACCEPT", "Safe setpoint inside arena envelope", (xw, yw, zw)
```

The information is already in the function. The plumbing is already correct. It is thrown away in the
last four lines. This is a ~15-line fix, not a redesign (§4.3).

### 1.3 Thrust-to-weight 1.22 — confirmed, with the arithmetic

Live model: `simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_vlp16/`

| Component | File:line | Value |
|---|---|---|
| Iris base link | `iris/iris.sdf:8` | 1.500 kg |
| Velodyne link | `iris_vlp16.sdf:13` | 0.830 kg |
| Rotors ×4 | `iris/iris.sdf:92,157,222,287` | 0.005 kg ea |
| Aux link | `iris/iris.sdf:59` | 0.015 kg |
| **Total** | | **≈ 2.365 kg → 23.2 N** |
| `motorConstant` | `iris/iris.sdf:361,378,395,412` | 5.84e-06 |
| `maxRotVelocity` | `iris/iris.sdf:360,377,394,411` | 1100 rad/s |
| **Max thrust** | `4 × 5.84e-6 × 1100²` | **28.26 N** |

**T/W = 28.26 / 23.2 = 1.218.**

For context: the stock PX4 Iris is tuned around T/W ≈ 1.9. Bolting an 0.83 kg VLP-16 onto a 1.5 kg
airframe without touching the motor model consumed 36% of the vehicle's mass budget and left it
barely able to hover, let alone accelerate. `MPC_THR_HOVER` is set to `0.65` at
`ROMFS/.../airframes/1023_gazebo-classic_iris_vlp16:42`; true hover throttle at this T/W is ≈ 0.82.
**PX4 has been flying with a hover-thrust estimate 17 points low since day one.** That alone explains
sluggish climb, altitude undershoot, and the guard timing out waiting for 1.2 m.

There is a second-order question here that matters more than the fix: **what is the real vehicle's
mass and thrust?** The brief allows ≤10 kg with prop guards. If your physical airframe is, say, 2.8 kg
with a lighter LiDAR, then the correct action is to *match the sim to the real vehicle*, not to inflate
`maxRotVelocity` until the sim flies nicely. Inflating it makes the sim pass and the hardware fail.
See §4.2 — this is a decision the team owes itself before it is a config change.

### 1.4 Eight instances of "no single source of truth"

Every root-level file below is a **diverged** copy of the file that actually loads:

| Root-level (dead) | Live (loaded by) | Diverged lines |
|---|---|---|
| `relay_odometry.py` | `scripts/relay_odometry.py` (`test_takeoff.sh`) | 172 |
| `nidar_mapping.launch` | `launch/fast_lio/nidar_mapping.launch` | 11 |
| `nidar_lidar.rviz` | `config/nidar_lidar.rviz` (`nidar_mapping.launch:40`) | 137 |
| `iris_vlp16.urdf` | `config/iris_vlp16.urdf` (`nidar_mapping.launch:18`) | 12 |
| `nidar_sim.yaml` | `config/fast_lio/nidar_sim.yaml` (`nidar_mapping.launch:6`) | 29 |

Plus:

- **Vehicle SDF** — `simulation/custom_models/iris_vlp16/iris_vlp16.sdf` (SDF 1.5, self-contained,
  own `imu_sensor` + `mavlink_interface`) vs the live
  `simulation/PX4-Autopilot-v1.14.3/Tools/.../models/iris_vlp16/iris_vlp16.sdf` (SDF 1.6, `<include>`s
  base iris). `custom_models/` is **not** on the Gazebo model path that `mavros_posix_sitl.launch`
  resolves. It is the one that *looks* canonical. This is your takeoff bug.
- **Arena geometry** — live is `simulation/custom_models/nidar_arena/model.sdf` → `meshes/mapdraw.stl`.
  Dead: root `nidar_airmouse_arena.sdf` (49 procedural wall/collision models — and honestly the
  *better* artifact, see §4.4), root `nidar_world_arena.dae`, `ARENA/result.dae`,
  `ARENA/rooms for drone denied simulation.glb`.
- **`PLANNING_DOCS/` itself** — `implementation_plan.md` and `Phase_4_completion.md` reference
  `scripts/mission_manager.py`, `scripts/fuel_to_mavros_bridge.py`,
  `config/fuel/exploration_planner.yaml`, `launch/nidar_fuel.launch`, `ARENA/tomar.dae`. **None of the
  five exist.** `Phase_4_completion.md` states Phase 4 is complete and that `mission_manager.py` is
  "configured with A* shortest path search and auto-land sequence." There is no such file and no
  RTH/land state machine anywhere in the tree.

That last one is the most dangerous item in this document. Your written record of what is done does
not match what is done, and the record is the thing new team members and future-you will trust.

### 1.5 No rangefinder in the spawned vehicle

`grep -c "tfmini\|lidar\|sonar\|rangefinder"` against the live `iris_vlp16.sdf` returns **0**.
`EKF2_HGT_REF` is `3` (vision) with `EKF2_BARO_CTRL 1` at airframe lines 48–50. Indoor baro drifts
metres over a 30-minute mission with door movement and prop wash. There is currently no independent
height source, which is precisely the condition under which "guard says ACCEPT while the vehicle sits
on the ground believing it's at 10 m" is *possible* rather than merely embarrassing.

### 1.6 Coverage against the mission brief

| Brief requirement | § | State |
|---|---|---|
| GPS-denied indoor nav | 1 | ✅ works |
| Autonomous corridor/room traversal | 4 | ✅ works (FUEL) |
| Enter from designated entry point | 4 | 🟡 `entry_detection_module.py` exists, not wired to a mission FSM |
| Detect up to 6 survivors onboard | 6 | ❌ **nothing** — no camera on the vehicle, no detector |
| Identify survivor **grid box** | 6 | ❌ nothing — no grid frame defined |
| Live 2D map, generated **during flight** | 5 | ❌ nothing — no `OccupancyGrid` publisher |
| Tag survivors **on that map** | 5,6 | ❌ nothing |
| Live camera feed at GCS | 5 | ❌ nothing |
| GCS: status / map / position / progress | 5 | ❌ **no GCS exists** |
| Autonomous exit via designated exit point | 4 | ❌ no RTH/exit state machine |
| ≤30 min flight, timer-aware | 4 | ❌ no mission clock |
| Failsafes: batt / link / geofence / abort / recall | 10 | 🟡 geofence partial (guard), rest absent |
| ≤5 min setup, one operator, no external network | 4,7,8 | ❌ not designed for |
| Prop guards, ≤10 kg, 1 m corridor / 8 ft clearance | 2,9 | 🟡 sim geometry ok; airframe undecided |

**Six hard ❌ against explicitly scored requirements.** That is the honest picture.

---

## 2. Phase map

```
P0  Truth        ── dedupe + preflight gate + smoke test        [BLOCKS EVERYTHING]
P1  Physics      ── airframe mass/thrust decision, hover thrust, rangefinder
P2  Integrity    ── guard state validation, honest diagnostics, failsafes
P3  Arena        ── sim fidelity: net, survivors, grid, entry/exit markers
P4  Perception   ── camera + survivor detection + 3D localisation
P5  Map          ── 2D occupancy slice + grid frame + survivor tagging
P6  GCS          ── operator station, live feed, mission status
P7  Mission FSM  ── ARM→ENTER→EXPLORE→SEARCH→EXIT→LAND + 30 min clock
P8  Acceptance   ── N-run regression, unknown-layout robustness, exit code
P9  Hardware     ── the sim→real transition, gated on P8 green
```

P0→P2 is *repair*. P3→P7 is *build*. P8 is the gate you asked for: "fully test the mission
requirement without any flaw in simulation, then proceed." P9 is what "proceed" means.

P0 and P1 can run in parallel with P3 (arena work touches no runtime code). P4 and P6 can parallelise
once P3 lands. Nothing in P4–P7 should start before P0 is green, because you will not be able to tell
whether new failures are yours or the substrate's.

---

## 3. Phase 0 — Truth infrastructure `[BLOCKING]`

**Goal:** the stack tells you the truth, automatically, with an exit code.
**Exit criterion:** `./scripts/ci_smoke.sh` returns 0 on a good tree and non-zero on each of the four
session-4 bugs, reintroduced deliberately one at a time.

### 0.1 Deduplicate — half a day, do it first

For each pair in §1.4: confirm the live path by tracing the load (`test_takeoff.sh` →
`nidar_mapping.launch:6,18,40`), then `git rm` the dead copy. Do **not** merge the diverged content
first — the dead copies are stale by 11–172 lines and merging imports old bugs. Delete, then diff the
deleted content in git history if a specific behaviour is missed.

Decision required on the vehicle SDF: `simulation/custom_models/iris_vlp16/` is dead but *looks*
canonical. Two options, and the team should pick explicitly:

- **(a) Delete `custom_models/iris_vlp16/`.** Live model stays inside vendored PX4. Simple, but keeps
  a hand-edited file inside a vendored tree — which is exactly what makes the PX4 decision (§0.4) hard.
- **(b) Make `custom_models/` canonical and symlink/copy it into the PX4 model path at build time**,
  via `scripts/build_px4.sh`. More work, but the vehicle definition becomes yours, version-controlled
  outside the vendor tree, and the copy step is a build artifact rather than a manual edit.

Recommendation: **(b)**. It is ~20 lines in `build_px4.sh` and it converts a permanent ambiguity into
a build step that can be asserted (§0.2).

> ### ✅ DECIDED 2026-09-04: option (b) — promote `custom_models/`
>
> **Status: DECIDED, NOT YET IMPLEMENTED.** `simulation/custom_models/iris_vlp16/iris_vlp16.sdf`
> becomes the canonical, hand-edited vehicle definition; `scripts/build_px4.sh` gains a step that
> copies it into the PX4 model path at build time, and `preflight_check.py` (§0.2) asserts the copy
> is current.
>
> Until that build step exists, **the live spawned model is still
> `simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_vlp16/iris_vlp16.sdf`**
> — that is the file to edit today. `custom_models/iris_vlp16/iris_vlp16.sdf` currently carries a
> warning header saying exactly this, and also still has a divergence that must be resolved as part
> of the promotion: it declares its own `mavlink_interface` plugin *and* includes `model://iris`
> (which carries one), so it would bind two MAVLink interfaces to the same ports if spawned as-is.
> Promoting it means reconciling that first, not just flipping a path.

Same call for the arena: root `nidar_airmouse_arena.sdf` has 49 named wall/collision primitives
(`corr1_wall_north`, `p1_col`, `launch_pad`, …). `mapdraw.stl` is a single opaque mesh. **The SDF is
strictly the better artifact** — you can move a wall, you can query geometry for ground-truth grid
validation in §5, and STL collision meshes in Gazebo are slow and prone to normals bugs. Recommend
promoting `nidar_airmouse_arena.sdf` to live and retiring the mesh path. This also unblocks P3.

### 0.2 Preflight config gate — `scripts/preflight_check.py`

Runs before any sim start, exits non-zero on any failure. Not a linter — an assertion that *what is on
disk is what will run*.

| Check | Implementation |
|---|---|
| **ROMFS == build** | Hash `ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16`; compare against a hash stamped into `build/px4_sitl_default/` at build time by `build_px4.sh`. Mismatch ⇒ "you edited the airframe and did not rebuild." **This is failure class #2, closed permanently.** |
| **Spawned SDF is the intended SDF** | Resolve `GAZEBO_MODEL_PATH` the same way the launch does; print and hash the file that will actually load. Fail if it is not the canonical one. |
| **Rangefinder present** | Assert a `<sensor type="ray">` down-facing link exists in the resolved SDF (once §1.2 lands). |
| **T/W ≥ 1.5** | Parse masses + `motorConstant` + `maxRotVelocity` from the resolved SDF chain, compute, assert. Print the number every run. |
| **Hover thrust sane** | Assert `MPC_THR_HOVER` is within ±0.05 of `1/(T/W)`. |
| **No orphan processes** | `pgrep` for `gzserver|px4|mavros_node|fastlio|exploration_node|traj_server`; fail loudly rather than killing silently. **This is failure class #3.** |
| **No dead configs** | Assert every path referenced in live launch files exists; assert no file at repo root shadows a file under `config/` or `launch/` or `scripts/`. Regression guard for §0.1. |

### 0.3 The smoke test — `scripts/ci_smoke.sh`

Yes — build it. It is the correct call. Spec:

```
scripts/ci_smoke.sh [--profile hover|explore|mission] [--seed N]
```

**Structure**

1. `preflight_check.py` — abort on non-zero.
2. Reap orphans; fail if any survive (do not `killall -9` and continue — that is how run 3 got
   contaminated).
3. Launch headless (`GUI_ARG=false`; §Key-learnings: GUI contention was present in every documented
   crash run).
4. Run the profile to completion or timeout.
5. Tear down; assert teardown succeeded.
6. Evaluate assertions from the recorded bag; emit JUnit XML + a one-line PASS/FAIL; exit 0/1.

**Assertions (profile `hover`, ~90 s)**

| Assertion | Threshold | Catches |
|---|---|---|
| Altitude band | `1.35 ≤ z ≤ 1.65` for ≥80% of samples after `t+20 s` | thrust failure, T/W 1.22 |
| Climb time to 1.2 m | ≤ 12 s | low T/W, bad `MPC_THR_HOVER` |
| Motor saturation | `actuator_outputs` at max for <5% of samples | T/W 1.22 **directly** |
| Guard `REJECT` count | 0 | frame bugs, envelope bugs |
| **Guard `ACCEPT` while grounded** | 0 (needs §2.1) | the ACCEPT-at-10 m bug |
| Vision↔EKF divergence, **frame-corrected** | < 0.30 m | real divergence only |
| `/Fast_LIO/odometry` rate | ≥ 9 Hz, no gap > 0.5 s | CPU starvation |
| Armed-after-landed | never > 5 s | premature-land / stuck-armed |
| Sim RTF | ≥ 0.85 | CPU starvation, before it corrupts results |

**Profile `explore` (~5 min)** adds: frontier count monotonically non-increasing over any 60 s window;
explored volume ≥ X m³ by `t+240 s`; zero collisions (Gazebo contact topic); Z stays in
`[0.8, 1.8]` for 100% of samples.

**Profile `mission` (P8)** adds the full-mission assertions in §11.

**Design rules that make it trustworthy**

- Every assertion reads the **bag**, not stdout. Stdout is for humans; assertions on stdout rot.
- One run = one directory `runs/<utc>_<git-sha>_<profile>/` containing bag, ULog, `preflight.json`,
  `assertions.json`, all node logs. **Never** `/tmp`. Failure class #3 dies here too.
- Every assertion has a **negative test**: a fixture that deliberately breaks it. An assertion that
  has never failed is an assertion you have not tested. Add `scripts/ci_smoke_selftest.sh` that
  reintroduces the four session-4 bugs and asserts smoke goes red on each.
- Assertion thresholds live in `config/smoke_thresholds.yaml`, not in the script.

**Effort:** 2–3 days for `hover` + `explore` including the self-test. This is the best-value work in
the entire document.

### 0.4 The vendored PX4 decision

`simulation/PX4-Autopilot-v1.14.3/` is a vendored tree with no `.git` (build script bootstraps a fake
one). You have hand-edited its ROMFS and its model directory. Three options:

- **(a) Keep vendored, document the edits.** Cheapest. Guarantees you re-hit failure class #2 forever
  on every teammate's machine.
- **(b) Git submodule pinned to v1.14.3 + an `overlay/` directory** that `build_px4.sh` copies over
  before building, with the copy asserted by `preflight_check.py`. Your edits become a visible,
  reviewable diff.
- **(c) Upstream-clean submodule; airframe registered via `PX4_SITL_EXTRA` / external model path.**
  Cleanest, most work, and v1.14.3's extension points for this are partial.

Recommendation: **(b)**. It composes with §0.1(b), it makes "what did we change in PX4?" answerable in
one `ls overlay/`, and the preflight hash check makes drift impossible. Budget 1 day.

> ### ✅ DECIDED 2026-09-04: option (b) — submodule + `overlay/`
>
> **Status: DECIDED, NOT YET IMPLEMENTED.** The vendored tree stays exactly as-is until the
> migration is actually done — do not assume `overlay/` exists.
>
> Known edits that must move into `overlay/` when this lands (i.e. everything hand-changed inside
> the vendored tree so far):
> - `ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16` — `EKF2_HGT_REF=2`,
>   `EKF2_RNG_CTRL`, `EKF2_MIN_RNG`, `MPC_Z_VEL_MAX_UP`, and `EKF2_EV_CTRL` (11, deliberately kept —
>   see its inline comment; 9 was tried and measurably worsened altitude accuracy)
> - `Tools/.../models/iris_vlp16/iris_vlp16.sdf` — the TFmini nested model + `tfmini_lidar_joint`
> - `Tools/.../models/iris/iris.sdf` **and** `iris.sdf.jinja` — `motorConstant` 5.84e-06 → 9.6e-06
>   (both files; the `.sdf` is generated from the `.jinja`, so editing only one lets a rebuild
>   silently revert it — a trap worth carrying into the overlay design)
>
> Note this decision also removes the reason §0.2's "ROMFS == build" hash check exists in its
> current form: with an overlay + assert-the-copy step, the failure mode it guards against
> (editing ROMFS source and forgetting to rebuild) is structurally prevented rather than detected.

---

## 4. Phase 1 — Physics correctness

**Exit criterion:** `preflight_check.py` T/W assertion passes with numbers derived from the *real
planned airframe*, and `hover` smoke is green with motor saturation < 2%.

### 1.1 Decide the real vehicle first `[TEAM DECISION — do not skip]`

> ### 🟡 DEFERRED 2026-09-04 — sim keeps a placeholder, risk accepted knowingly
>
> Real hardware numbers are not available yet, so `PLANNING_DOCS/airframe_spec.md` is **not**
> written and the sim keeps a placeholder: `motorConstant` 9.6e-06 giving T/W ≈ 2.00 on the
> currently-modelled 2.365 kg VLP-16 vehicle. That value was chosen to give sane control margin in
> sim, **not** derived from a real airframe — which is precisely the thing this section warns
> against. Accepted deliberately, with these consequences on the record:
>
> - This section's own exit criterion ("T/W assertion passes with numbers derived from the *real
>   planned airframe*") is **not met**, and P1 is therefore not truly green regardless of how well
>   the sim flies.
> - If the real vehicle uses RPLidar (~0.19 kg) rather than VLP-16 (0.83 kg), the sim is modelling
>   an aircraft ~0.64 kg heavier than the real one — and per §12 that also changes the SLAM software
>   path (FAST-LIO vs Hector), so this is not only a mass-model discrepancy.
> - Revisit before P8 acceptance, and certainly before P9. Sim results gathered under the
>   placeholder do not transfer to hardware claims.
>
> **What unblocks this:** frame/motor/ESC/battery/FC/companion/LiDAR/camera/prop-guard masses, and
> the motor manufacturer's bench thrust at the intended prop and voltage.

Before touching `motorConstant`, answer: what is the physical drone? Brief constraints: ≤10 kg,
prop guards mandatory, must fly a 1 m corridor with 8 ft ceiling, 30 min endurance.

The sim currently models a 2.365 kg vehicle carrying a 0.83 kg VLP-16. Your hardware track
(per prior sessions) is RPLidar, which is ~0.19 kg. **The sim vehicle and the real vehicle are not the
same aircraft, and tuning the sim to fly nicely will actively mislead the hardware bring-up.**

Deliverable: a one-page `PLANNING_DOCS/airframe_spec.md` with measured/quoted mass for frame,
motors, ESCs, battery, FC, companion computer, LiDAR, camera, prop guards; and thrust from the motor
manufacturer's bench data at your prop/voltage. Then set the sim to match. Target T/W ≥ 1.8 on the
real vehicle — 1.5 is the *smoke-test floor*, not a design target; indoor flight in 1 m corridors
needs authority for aggressive attitude corrections in ground effect.

### 1.2 Apply to sim

Once `airframe_spec.md` exists: set link masses and `motorConstant`/`maxRotVelocity` in the canonical
SDF to match. (For reference: holding `motorConstant` at `5.84e-06` and the current 2.365 kg,
`maxRotVelocity = 1250` gives T/W ≈ 1.57. But do this from the spec, not from this number.)

Then set `MPC_THR_HOVER ≈ 1/(T/W)` in the airframe file — and let `MPC_THR_HOVER` be *learned*
(`MPC_USE_HTE = 1`) so the sim reports what it converges to, which is a free cross-check on your mass
model.

### 1.3 Add a down-facing rangefinder

Add a TFmini-class `<sensor type="ray">` to the canonical vehicle SDF, wire the PX4 SITL distance
sensor plugin, set `EKF2_RNG_CTRL` and `EKF2_HGT_REF` appropriately (rangefinder as primary height
under ceiling, vision as fallback). This is what makes "believes it's at 10 m while grounded"
detectable by the vehicle itself rather than only by the smoke test.

Note: a rangefinder over a maze floor with debris/thresholds needs `EKF2_RNG_A_*` innovation gating
tuned, or you trade one height bug for another. Budget a day of tuning, and add a smoke assertion:
range-vs-EKF-height agreement < 0.15 m while below 2 m.

---

## 5. Phase 2 — Diagnostic and guard integrity

**Exit criterion:** every diagnostic number is either correct or absent. No metric prints a value it
cannot justify.

### 2.1 Guard: validate state, not just commands

`scripts/flight_envelope_guard.py:242-255`. Replace the `logwarn`-and-continue with a real state
machine:

- **State outside envelope** ⇒ do not forward the FUEL command. Enter `SAFE_HOLD`, command the last
  known-good in-envelope setpoint, publish `REJECT: STATE_OUT_OF_ENVELOPE`.
- **State disagrees with commanded by > `pos_err_max`** (suggest 1.5 m sustained > 2 s) ⇒ the vehicle
  is not tracking. Enter `SAFE_HOLD`, escalate after 5 s.
- **Grounded (`landed_state == ON_GROUND`) while commanded altitude > 0.5 m** ⇒ hard fault. Publish
  `FAULT: GROUNDED_BUT_COMMANDED_AIRBORNE`, do not ACCEPT. **This is the specific bug you found.**
- Publish guard state on a latched topic so the smoke test and the GCS can both assert on it.

Also fix the latent inverted-Z-window bug in the hardcoded fallback bounds (`eff_zw_min >
eff_zw_max` when the YAML load fails). Make a failed param load **fatal**, not silently defaulted —
a safety guard that silently falls back to unvalidated constants is worse than no guard.

### 2.2 Fix the divergence metric

`scripts/mission_telemetry_logger.py:151`: apply the `(xc = y, yc = -x)` rotation before subtracting.
Better: **delete the hand-rolled conversion entirely** and use `tf2` to look up
`camera_init → map` and transform properly, so the conversion exists in exactly one place instead of
being re-derived in `flight_envelope_guard.py:244`, `:422`, and here.

Fix the label at `:172`. Add a unit test that feeds a synthetic pose pair with known ground-truth
divergence and asserts the metric returns it — including a 90°-rotated pair that must return ~0.

### 2.3 One clock

Establish a single mission time base — `/clock` sim time, with `t=0` at the arm transition, stamped
identically into telemetry CSV, guard diagnostics, and the rosbag. Cross-reference to PX4 ULog boot
time recorded once at startup in `run_meta.json`. Every log line carries both. You should never again
have to ask "does this timestamp match the flight log."

### 2.4 Failsafes (brief §10)

Implement and smoke-assert each, since all five are explicitly required:

| Failsafe | Implementation | Assertion |
|---|---|---|
| Low battery | `/mavros/battery` threshold → FSM `RETURN` | inject fake low-batt, assert RETURN within 2 s |
| C2 link loss | GCS heartbeat timeout → FSM `RETURN` (**not** land-in-place; you are indoors) | kill GCS node, assert RETURN |
| Geofence breach | guard `SAFE_HOLD` → FSM `RETURN` | command out-of-bounds, assert no forward |
| Mission abort | operator topic/service → immediate `LAND` | assert < 500 ms latency |
| Emergency recall | operator → FSM `RETURN` | assert path replan to entry |

Note the brief permits the operator exactly two inputs: start, and abort. Abort must therefore be a
single physical action with no confirmation dialog, and must work when ROS is unhealthy — consider a
direct MAVLink path that bypasses your own stack.

---

## 6. Phase 3 — Arena fidelity

**Exit criterion:** the sim arena is a faithful instance of the brief's *class* of arena, and a second,
structurally different layout exists and runs.

The brief says the layout **will not be disclosed**. So the arena is not a fixed asset to perfect — it
is a *generator* of test cases. Building one beautiful maze and tuning until it works is the single
most likely way to fail on competition day.

### 3.1 Promote the parametric arena — **NOT BEING IMPLEMENTED (2026-09-04, team decision)**

> **Superseded.** The team has various arena STL files already prepared and will test against
> those directly instead of retiring `mapdraw.stl` for a promoted/parameterised
> `nidar_airmouse_arena.sdf`. This still satisfies the underlying goal this section was chasing —
> multiple structurally different layouts, not one hand-tuned maze — just via hand-authored STL
> variants rather than a procedural SDF generator. `nidar_airmouse_arena.sdf` was restored from a
> mistaken cleanup deletion on 2026-09-04 (see `repo_cleanup_and_mind_map_2026-09-03.md`) and left
> in the tree, but is not being promoted to live.
>
> Practical note for whoever swaps in the next STL: the live arena's wall collision friction
> (`simulation/custom_models/nidar_arena/model.sdf`) was found and fixed this session — it was set
> to `mu=100/mu2=50` (should be ~1.0), which was one of two root causes of the drone getting
> physically stuck against walls (see
> `PLANNING_DOCS/guard_state_validation_and_wall_stuck_fix_2026-09-04.md` Sec 2a). That
> `<surface><friction>` block wraps whichever mesh URI is referenced — check it stays at 1.0/1.0 (or
> is otherwise deliberately set, not left unset-and-assumed) on any new STL swapped in, since a raw
> STL import with no explicit friction override is the more common way to end up back at an
> engine/tool default that may not be 1.0.

Retire `mapdraw.stl`; promote `nidar_airmouse_arena.sdf` (§0.1). Then parameterise it: a small Python
generator emitting valid SDF from a grid spec, honouring the brief's constraints — 15×15 m max, ≥1 m
corridors, 2×2 m rooms, uniform grid, 8 ft (2.44 m) clearance, single shared entry/exit.

Deliverable: **three** structurally different layouts (`arena_a/b/c.sdf`) plus a `--seed` random
generator. Smoke `mission` profile runs against a seed the developer did not choose.

### 3.2 Add what the brief specifies and the sim lacks

- **Net ceiling at 2.44 m.** Currently absent. This matters: it is a LiDAR return surface that will
  appear in the FUEL voxel map and generate frontiers *above* the flight envelope. Model it as a thin
  collision plane with realistic sparse returns. Do not skip this — it is a likely source of
  exploration deadlock that you have never seen because it is not modelled.
- **Six survivor models.** Human-scale, prone/seated/partially-occluded, placed in rooms. Needs to be
  a Gazebo model with a known ground-truth pose that the acceptance test reads back (§11).
- **Grid markings + a defined grid origin.** The brief scores "grid coordinate or grid box." Define
  the grid frame now, in `config/arena_grid.yaml` — origin, cell size, labelling convention (A1..N14).
  Everything in P5 depends on this existing.
- **Entry/exit marker** the vehicle can detect (`entry_detection_module.py` presumably expects
  something — verify what cue it looks for and model *that*).
- **Debris / non-planar obstacles** at varying heights. Prior sessions concluded obstacle complexity is
  "fundamentally 2D" — the brief's "damaged building" framing does not guarantee that, and a
  low obstacle under the flight altitude is invisible to a 2D assumption.

### 3.3 Sensor realism

Add noise to the LiDAR (Gaussian + dropout), IMU bias/random-walk, and baro drift. A stack that only
works on noiseless sim sensors is a stack that has not been tested. Add a smoke profile
`explore --noise high` that must still pass.

---

## 7. Phase 4 — Perception (survivor detection)

**Exit criterion:** ≥5 of 6 survivors detected, each localised within one grid cell of ground truth,
zero false positives, running within the CPU budget alongside FAST-LIO and FUEL.

### 4.1 Camera on the vehicle

Add an RGB camera to the canonical SDF. **Orientation is a design decision with real consequences:**
forward-facing sees survivors in rooms as the drone passes doorways but has narrow coverage; downward
sees only what it overflies. A forward camera with ~90° HFOV, tilted down ~20°, is the usual answer for
corridor search. Publish `/camera/color/image_raw` + `camera_info`.

Budget the CPU cost now, not later. Prior sessions already found FAST-LIO starving at load average 12+.
Adding a detector is adding a competing consumer to a system that has already demonstrated it will
freeze SLAM under contention. **Add a smoke assertion on FAST-LIO rate that runs with the detector
active**, and treat any regression as a blocking failure rather than something to tune later.

### 4.2 Detector

YOLOv8n or v11n, person class, ONNX Runtime CPU in sim / TensorRT on Jetson. Run at 5 Hz, not camera
rate — you do not need 30 Hz to find a stationary body, and the CPU headroom is worth more.

Train/validate on renders of your Gazebo survivor models *plus* real prone-person imagery. A detector
that only fires on your specific Gazebo mesh will score zero on real dummies.

### 4.3 3D localisation

Back-project the bbox centroid, raycast against `/cloud_registered`, transform to `map`, snap to grid
cell via `config/arena_grid.yaml`. Track detections across frames — one survivor seen in 40 frames is
one survivor, not 40. Use a simple nearest-neighbour tracker with a 0.75 m association radius and a
confirmation threshold (≥3 consistent detections) before publishing a tag.

Publish `/survivors` as a custom message: `id, grid_cell, position, confidence, first_seen, n_obs`.
Latched, so a late-joining GCS gets the full set.

**Accuracy note:** prior sessions flagged a 4 cm LiDAR extrinsic mismatch that mattered. A 2° camera
extrinsic error at 5 m range is 17 cm of projection error — under a 1 m grid cell that is survivable,
but stack it with LiDAR extrinsic error and yaw error and you can land in the wrong cell. Calibrate,
and add a sim-only assertion comparing tagged cell against Gazebo ground truth.

---

## 8. Phase 5 — Live 2D map and grid tagging

**Exit criterion:** a `nav_msgs/OccupancyGrid` publishes at ≥2 Hz during flight, is legible as a maze
floorplan, and carries survivor markers in correct cells — all *before landing*, per brief §5.

### 5.1 2D slicer node

Subscribe to FUEL/FIESTA's voxel grid (or `/cloud_registered` accumulated). Slice `0.3 m ≤ z ≤ 1.9 m`,
project occupied → 100, known-free → 0, unknown → -1. Publish `/map_2d` at 2–5 Hz throttled.

Slice band choice matters: too low and you catch floor debris as walls; too high and you miss
waist-height obstacles and catch the net. Make it configurable and validate against ground-truth arena
geometry — you can do this automatically now that the arena is parametric SDF (§3.1): compare published
occupancy against the known wall positions and assert IoU > 0.85.

### 5.2 Grid overlay + tagging

Publish the grid lines and labels as `visualization_msgs/MarkerArray` in the same frame, driven by
`config/arena_grid.yaml`. Publish survivor tags as a second MarkerArray with text labels showing the
cell ID.

**Brief compliance detail worth flagging:** §5 requires "identified corridors, rooms or sections,
wherever technically feasible." A raw occupancy grid does not do this. A cheap win: run a
morphological room-segmentation pass over the 2D grid (distance transform + watershed) and publish
coloured room regions. Judges reading "usable by rescue teams" will see the difference. Low effort,
visible score.

---

## 9. Phase 6 — Ground Control Station

**Exit criterion:** a single operator, on one laptop, with no internet, sees everything brief §5
enumerates, and can trigger exactly two actions: start and abort.

> **Status 2026-10-02 — simulation side built.** The team's own web UI replaces Foxglove (§6.1):
> `catkin_ws/src/nidar_gcs` (README there). In simulation it starts / pauses / resets the run and
> shows: live telemetry and mission phase; the mission clock (sim time since arming); the live 2D
> map (`/map_2d`, now built from the LiDAR by `nidar_map2d/lidar_map_2d.py`: walls + explored
> floor, A1–G7 grid, drone and flown path, survivors as hotspots with their grid box highlighted);
> a survivors list with grid box and position; the camera with detector boxes; subsystem health; an
> event timeline; and a two-press EMERGENCY ABORT (PX4 AUTO.LAND). Fonts are bundled, so it works
> offline (brief §7). Verified with flights driven from the browser; map scored against the arena
> mesh with `nidar_qa/scripts/map_accuracy.py`. Not done yet: room/corridor labelling (§5.2), the
> 30-minute clock / coverage progress, link-loss and recall failsafes (§2.4), the Hardware page's
> real link (§6.2), the bandwidth assertion (§6.2) and the setup-time test (§6.3).

### 6.1 Stack

Recommend **Foxglove Studio** (self-hosted, offline-capable) over custom web UI, with a saved layout
committed to the repo as `gcs/nidar_layout.json`. Rationale: you have limited calendar time and the
custom-dashboard path historically eats it. If Foxglove's offline licensing is a concern, fall back to
`rosbridge_suite` + a static HTML page — but decide early, because the transport choice constrains §6.2.

Panels: live camera (compressed), `/map_2d` with marker overlays, drone pose, mission FSM state,
mission clock (elapsed / 30 min remaining), survivor count + cell list, guard state, battery, link
health.

### 6.2 Transport under the no-network rule

Brief §7: no GSM/LTE/WiFi-public/internet/cloud. A private 5 GHz link is permitted (it is the team's
own local link). Stream **only**: `image_transport/compressed` camera at ~5 Hz and reduced resolution,
`/map_2d` throttled, marker arrays, and a compact status message. Do **not** stream point clouds — it
will saturate the link and the resulting frame drops will look like system failure to a judge.

Measure the bandwidth budget and add it to the smoke test: assert total GCS-bound bandwidth < 4 Mbps.

### 6.3 Setup time

Brief §4: 5 minutes, two people, for everything. This is a real engineering requirement, not a
footnote. Deliverable: a single `./start_mission.sh` that brings up the entire onboard stack and a
single GCS launch, both tested cold-start with a stopwatch. Add `scripts/setup_timer_test.sh` that
measures cold-boot-to-mission-ready and asserts < 180 s (leaving margin for physical placement).

---

## 10. Phase 7 — Mission state machine

**Exit criterion:** one command from the operator produces a complete mission with no further input.

### 7.1 `scripts/mission_commander.py`

States: `PREFLIGHT → ARMED → TAKEOFF → ENTER → EXPLORE → RETURN → EXIT → LAND → COMPLETE`, plus
`ABORT` and `SAFE_HOLD` reachable from any state.

Note that `PLANNING_DOCS/next_phase_work.md` and `Phase_4_completion.md` both describe this node as
existing. It does not (§1.4). Treat those documents as requirements, not as status.

Transition logic worth thinking about carefully:

- **`EXPLORE → RETURN`** is the highest-risk decision in the mission. Trigger on
  `survivors == 6 OR frontiers == 0 OR t_elapsed > t_budget`. The third one is what saves you: compute
  `t_budget = 30 min − t_return_estimate − margin`, where `t_return_estimate` is a live A* path cost
  from current position back to entry, recomputed every 10 s. A drone that maps beautifully and gets
  timed out inside the maze scores near zero. **Set the margin at 25% and resist tuning it down.**
- **`RETURN → EXIT`**: the entry and exit are the same point (brief §3). Path back to `(X₀, Y₀)`
  through explored space only — never through unexplored frontier, which is slower but cannot deadlock.
- **`EXIT`**: the vehicle must actually leave the maze, not just reach the entry cell. Define exit
  success geometrically against the arena spec.

### 7.2 Mission clock

A single authoritative countdown from arm, published to the GCS and consumed by the FSM. Assert in
smoke that it never disagrees with wall time by > 1 s.

---

## 11. Phase 8 — Full-mission acceptance `[THE GATE]`

This is what you meant by "fully test the mission requirement without any flaw in simulation."

**Exit criterion — all of the following, on 20 consecutive runs across ≥3 arena layouts including at
least one seed no team member has seen:**

| # | Assertion | Threshold |
|---|---|---|
| 1 | Mission completes autonomously, zero operator input after start | 20/20 |
| 2 | Survivors detected | ≥ 5/6, mean ≥ 5.5 |
| 3 | Survivor grid cell correct vs Gazebo ground truth | ≥ 95% of detections |
| 4 | False positive survivors | 0 |
| 5 | 2D map IoU vs ground-truth arena geometry | ≥ 0.85 |
| 6 | Map + tags published **before** landing | 20/20 |
| 7 | Wall/ceiling/obstacle collisions | 0 |
| 8 | Mission duration | ≤ 25 min (20% margin on 30) |
| 9 | Returns to entry and exits | 20/20 |
| 10 | Guard `FAULT` events | 0 |
| 11 | FAST-LIO rate never < 8 Hz, no gap > 0.5 s | 20/20 |
| 12 | Sim RTF ≥ 0.85 throughout | 20/20 |
| 13 | GCS bandwidth | < 4 Mbps peak |
| 14 | Each of the 5 failsafes fires correctly when injected | 5/5 |
| 15 | Cold setup to mission-ready | < 180 s |

Plus **fault-injection runs** (separate, need not be 20/20 but must fail *safely*): FAST-LIO freeze
mid-flight; LiDAR dropout for 3 s; GCS link severed; battery drop to 20%; a survivor placed in a
corridor rather than a room; a dead-end that traps the planner.

**Do not proceed to hardware until the table above is green.** And when it is green, spend one day
trying to break it on purpose — a suite that has never gone red since it was written is not evidence.

---

## 12. Phase 9 — Hardware transition

Only after §11. Sequence: bench (props off, full stack, verify all topics) → tethered hover →
single-corridor → 2-room mock maze → full physical arena. Port the smoke test to hardware with
relaxed thresholds; the assertions are the same, only the numbers change.

The RPLidar-vs-VLP16 decision from prior sessions (Hector SLAM for X/Y/yaw + separate Z source,
rather than wrapping 2D `LaserScan` into FAST-LIO) means **the hardware stack is not the stack you are
validating in sim.** That is a significant risk that no amount of sim testing retires. Mitigation:
build the SITL `iris_rplidar` variant *during* P3 and run the acceptance suite against **both**
sensor configurations. If the RPLidar path cannot pass §11 in sim, you learn it in October rather than
in the arena.

---

## 13. Sequencing and effort

| Phase | Effort | Depends on | Parallel with |
|---|---|---|---|
| P0 Truth | 4–5 d | — | P3.1 |
| P1 Physics | 2–3 d (+ airframe decision) | P0.1 | P3 |
| P2 Integrity | 3–4 d | P0 | P3 |
| P3 Arena | 4–5 d | P0.1 | P1, P2 |
| P4 Perception | 6–8 d | P3.2, P1.1 | P6 |
| P5 Map | 4–5 d | P3.2, P4.3 | P6 |
| P6 GCS | 4–5 d | P5.1 | P4 |
| P7 Mission FSM | 4–5 d | P2, P5 | — |
| P8 Acceptance | 5–7 d | all | — |
| P9 Hardware | ongoing | P8 green | — |

Roughly **6–8 weeks** of focused work to a green §11 gate, assuming the team is not also doing
hardware fabrication in the same window. If it is, double P9's overlap and start P3's RPLidar SITL
variant early.

---

## 14. Immediate next actions

**Updated 2026-09-04.** Original list, with outcomes:

1. ~~**Team decision, today:** SDF canonicalisation option (a) or (b) — §0.1; vendored PX4 option
   (a), (b), or (c) — §0.4.~~ ✅ **Both decided:** §0.1 **(b)** promote `custom_models/`, §0.4 **(b)**
   submodule + `overlay/`. Both **decided, not yet implemented** — see the boxes in those sections
   for exactly what still has to be built and what the live paths are in the meantime.
   §1.1 (real airframe) 🟡 **deferred** — placeholder T/W in sim, risk recorded in that section.
2. ~~**Delete the five root-level shadow files**~~ ✅ **Done** — plus 22 more dead files; see
   `PLANNING_DOCS/repo_cleanup_and_mind_map_2026-09-03.md` §6. Two of those deletions were
   subsequently reversed as mistakes (`nidar_airmouse_arena.sdf`, `scripts/strict_monitor.py`).
3. ~~**Correct `PLANNING_DOCS/Phase_4_completion.md`**~~ ✅ **Done** — status note added and archived;
   `implementation_plan.md` annotated the same way.
4. **Build `preflight_check.py` and `ci_smoke.sh` (§0.2, §0.3).** ⬅ **still the next system-track
   item.** Deferred 2026-09-04 at the team's request while P3 (arena STL variants) is worked in
   parallel. Nothing else in P0 remains once this lands.

### 14a. Where the system actually stands (2026-09-04)

| Phase | State |
|---|---|
| P0 Truth | 🟡 partial — dedup done; both structural decisions made but unimplemented; **preflight + smoke test not built** |
| P1 Physics | 🟡 partial — rangefinder ✅, thrust margin ✅ (placeholder numbers), real airframe spec deferred |
| P2 Integrity | 🟡 partial — guard state validation ✅ (§2.1, live-verified), divergence metric ✅ N/A, **one clock (§2.3) and 4 of 5 failsafes (§2.4) not started** |
| P3 Arena | 🔵 in progress by the team — STL variants instead of §3.1's parametric generator; §3.2 (net ceiling, survivors, grid frame, entry/exit markers) and §3.3 (sensor noise) not started |
| P4–P7 | ❌ not started — no camera, no detector, no `OccupancyGrid`, no GCS, no mission FSM |
| P8 Acceptance | ❌ gated on all of the above |

Two bugs found during this work that predate and were missed by this audit are logged as A21/A22 in
Appendix A, both fixed and live-verified: FUEL's exploration box exceeding the guard envelope, and
arena wall friction set ~100x too high (the literal cause of the drone sticking to walls).

---

## Appendix A — Exact edit targets

Provided as file:line for independent application. No patches written.

**Status column key:** ✅ DONE (applied + live-verified) · ✅ N/A (checked against live telemetry;
the original finding did not hold for this repo's actual configuration) · 🟡 OPEN (not applied —
either a team decision this doc itself flags as one, or genuinely still outstanding) · 🟡 PARTIAL
(mitigated but not the full recommended fix) · 🔁 SUPERSEDED (decided against in favour of a
different approach). All statuses as of 2026-09-04; see
`PLANNING_DOCS/guard_state_validation_and_wall_stuck_fix_2026-09-04.md` for A4/A5's full story
including a false-positive regression caught and fixed the same day, and
`PLANNING_DOCS/repo_cleanup_and_mind_map_2026-09-03.md` §6 for A11-A15/A17.

| # | File | Line(s) | Issue | Status |
|---|---|---|---|---|
| A1 | `scripts/mission_telemetry_logger.py` | 151 | ENU−camera_init subtraction without rotation; metric = √2·r | ✅ N/A — checked live: `/mavros/local_position` reads in the *same* frame as `/Fast_LIO/odometry` in this stack (relay_odometry.py forwards FAST-LIO's pose unmodified as "vision"), so there is no unrotated 90° gap here. Metric is correct as written. |
| A2 | `scripts/mission_telemetry_logger.py` | 172 | Label claims `camera_init` for an ENU quantity | ✅ N/A — label is accurate given A1 |
| A3 | `scripts/mission_telemetry_logger.py` | 169 | `> 0.5` threshold on a broken metric | ✅ N/A — metric isn't broken, threshold stands |
| A4 | `scripts/flight_envelope_guard.py` | 242–255 | State check logs, then returns `ACCEPT` | ✅ DONE — now rejects (`FAULT_STATE_OUT_OF_ENVELOPE`) on real X/Y excursion. First attempt also gated on Z and produced 33/33 false positives from normal hover noise against a 4cm band; narrowed to X/Y only and re-verified clean (480s live, 0 events). |
| A5 | `scripts/flight_envelope_guard.py` | 244–246, 422 | Frame conversion duplicated 3× — centralise in tf2 | ✅ DONE (centralised to one Python helper, not tf2 — sufficient to remove the duplication risk without a bigger refactor) |
| A6 | `scripts/flight_envelope_guard.py` | (fallback bounds) | Inverted Z window when YAML load fails; must be fatal | ✅ N/A — checked, fallback defaults are correctly ordered (not inverted) |
| A7 | `.../models/iris_vlp16/iris/iris.sdf` | 360–361, 377–378, 394–395, 411–412 | `motorConstant`/`maxRotVelocity` → T/W 1.22 | ✅ DONE — `motorConstant` 5.84e-06→9.6e-06, T/W 1.22→2.00, verified 338s+ and 480s flights |
| A8 | `.../airframes/1023_gazebo-classic_iris_vlp16` | 42 | `MPC_THR_HOVER 0.65` vs true ≈0.82 | ✅ DONE indirectly — A7's fix brought true hover to ≈0.68, close enough to the declared 0.65 that no separate change was needed |
| A9 | `.../models/iris_vlp16/iris_vlp16.sdf` | — | No rangefinder sensor | ✅ DONE — full TFmini pipeline (Gazebo model + dual plugin + FAST-LIO Z-pin with validity gating + `EKF2_HGT_REF=2`), predates this doc, done in an earlier session |
| A10 | `simulation/custom_models/iris_vlp16/iris_vlp16.sdf` | whole file | Dead but looks canonical | 🟡 OPEN — team decision (§0.1 (a) delete vs (b) promote+build-step). Warning header added so it can't silently mislead again; not deleted or promoted. |
| A11 | `relay_odometry.py` (root) | whole file | Dead shadow, 172 lines diverged | ✅ DONE — deleted |
| A12 | `nidar_mapping.launch` (root) | whole file | Dead shadow, 11 lines diverged | ✅ DONE — deleted |
| A13 | `nidar_lidar.rviz` (root) | whole file | Dead shadow, 137 lines diverged | ✅ DONE — deleted |
| A14 | `iris_vlp16.urdf` (root) | whole file | Dead shadow, 12 lines diverged | ✅ DONE — deleted |
| A15 | `nidar_sim.yaml` (root) | whole file | Dead shadow, 29 lines diverged | ✅ DONE — deleted |
| A16 | `nidar_airmouse_arena.sdf` (root) | whole file | Better artifact than live `mapdraw.stl`; promote | 🔁 SUPERSEDED — team will test multiple hand-authored arena STL variants instead (§3.1 above). File itself restored after a mistaken cleanup deletion and kept in the tree, but not promoted to live. |
| A17 | `PLANNING_DOCS/Phase_4_completion.md` | — | Certifies 5 non-existent files | ✅ DONE — status note added pointing at this audit; archived to `PLANNING_DOCS/archive/` |
| A18 | `PLANNING_DOCS/implementation_plan.md` | — | References 5 non-existent files | ✅ DONE — status note added pointing at this audit |
| A19 | `scripts/test_takeoff.sh` | 9–20 | `killall -9` masks orphans instead of failing on them | 🟡 PARTIAL — added the missing kills (rviz, robot_state_publisher, static_transform_publisher were leaking and contaminating re-runs) so orphans stop accumulating, but cleanup is still silent-kill-and-continue, not the fail-loud behaviour this item recommends |
| A20 | `scripts/strict_monitor.py` | — | `PREMATURE_LAND_ALERT` logs only; not wired to MAVROS land/disarm (carried over, still open) | 🟡 OPEN — file restored after a mistaken cleanup deletion (it documents a real open gap, not dead code); the gap itself is still open |
| A21 *(new, found 2026-09-04)* | `launch/nidar_fuel_upstream.launch` | box_min/max_x/y | FUEL's exploration box was the raw arena size; the guard's effective envelope is 0.2m smaller on every side (boundary_margin). FUEL kept planning into the resulting dead ring, guard always rejected, contributing to "stuck at walls." | ✅ DONE — box tightened to `[-0.3,13.3] x [-6.8,6.8]`, the exact camera_init pre-image of the guard's envelope |
| A22 *(new, found 2026-09-04)* | `simulation/custom_models/nidar_arena/model.sdf` | `<surface><friction>` | Wall collision `mu=100/mu2=50` — ~100x any real material. Made wall contact resist sliding almost completely instead of letting the vehicle deflect off, which was the primary literal "stuck to the wall" mechanism (independent of A4/A5/A21, and invisible to the guard since it's pure Gazebo physics). | ✅ DONE — set to 1.0/1.0, verified live: vehicle reached within 10cm of the boundary (world Y=6.70 vs 6.8 limit) and continued flying normally, 0 stuck signatures across 480s of monitored flight |
| A19 | `scripts/test_takeoff.sh` | 9–20 | `killall -9` masks orphans instead of failing on them |
| A20 | `scripts/strict_monitor.py` | — | `PREMATURE_LAND_ALERT` logs only; not wired to MAVROS land/disarm (carried over, still open) |
