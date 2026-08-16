# Implementation Plan — Z-Axis Boundary Crossing Causes Crash Instead of Soft Recovery

Status: **plan only, no code changed except this doc**. Root causes below are traced directly against
`scripts/flight_envelope_guard.py`, `config/flight_envelope_guard.yaml`, the PX4 airframe file, and
`catkin_ws/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp`.

**Revision note (post-review):** the strategy below has been restructured around a source-level fix — constrain
FUEL's own exploration map (`sdf_map/box_max_z`) so it structurally cannot generate a viewpoint above the ceiling,
rather than relying on the downstream guard to reactively clamp/recover from a bad command after the fact. See
§3, new **Phase 0**, which is now the primary fix. Phase 5 (verification) has been executed — see its section for
results. Phases 3–4 (reactive clamp-and-resume, planner feedback loop) are downgraded to optional hardening,
since Phase 0 eliminates the vast majority of the trigger conditions they were written to handle.

---

## 1. Symptom (as reported)

The Z limit *is* being enforced (commands outside `world_z_min`/`world_z_max` are detected and rejected), but
instead of smoothly stopping the drone at the boundary and letting mapping resume, the drone crosses the limit
anyway, "gets fixed" abruptly, and crashes/falls. This is qualitatively different from the X/Y behavior, which
apparently degrades gracefully.

## 2. Confirmed root causes

### 2.1 Z has no safety margin — X/Y does

In `flight_envelope_guard.py.__init__`:
```python
self.eff_xw_min = self.world_x_min + self.boundary_margin
self.eff_xw_max = self.world_x_max - self.boundary_margin
self.eff_yw_min = self.world_y_min + self.boundary_margin
self.eff_yw_max = self.world_y_max - self.boundary_margin
self.eff_zw_min = self.world_z_min      # <-- no margin subtracted
self.eff_zw_max = self.world_z_max      # <-- no margin subtracted
```
X and Y get a `boundary_margin` (currently 0.2 m, was previously tuned to 0.65 m using the kinematic
stopping-distance formula `s = v²/2a`) subtracted before the effective bound is computed. **Z gets none.** There
is zero buffer between "still flying normally" and "hard rejection" on the vertical axis.

### 2.2 Z velocity is never damped approaching the boundary — X/Y velocity is

In `fuel_cb`, once a command is accepted, X and Y velocity components are directionally clamped near their
effective bounds:
```python
if xw_clamped <= self.eff_xw_min + 0.3:
    vx_w = max(0.0, vx_w)
elif xw_clamped >= self.eff_xw_max - 0.3:
    vx_w = min(0.0, vx_w)
# ... same pattern for yw_clamped/vy_w ...
target.velocity.z = vz   # <-- passed through completely unmodified, no bound-proximity damping at all
```
So even while the drone is still technically inside bounds, it can be commanded at full vertical velocity right
up to the instant it crosses — there is no analogue of the X/Y "start braking within 0.3 m of the wall" logic for
the ceiling/floor. Combined with §2.1 (no margin), Z gets no early warning and no deceleration — the two
mechanisms that make X/Y forgiving are both absent for Z.

### 2.3 Out-of-bounds Z is a hard REJECT, not a clamp — creates a position discontinuity

`validate_command()` returns `False` outright for any Z outside `[eff_zw_min, eff_zw_max]` — the clamping logic
(`xw_clamped = min(max(xw, ...), ...)`) only runs inside the `if is_valid:` branch, i.e. **after** the command
already passed the bounds check. A command that fails only on Z is rejected wholesale, X/Y improvement in the
same message included.

On rejection, `last_valid_command` / `last_valid_pos_world` are **not updated** — the guard just keeps streaming
the last *accepted* position (from before the drone approached the boundary) via `timer_cb`, with velocity forced
to zero (`IGNORE_VX|VY|VZ`). If the drone had any un-damped vertical velocity when the reject started (see §2.2),
it will have physically overshot past that last-accepted position by the time PX4 receives the frozen setpoint.
PX4 then sees a large, sudden position error and commands a correspondingly large corrective thrust/attitude
change — this abrupt correction is almost certainly what's read as "crashes and falls": not the boundary
detection failing, but the recovery response being a discontinuous jump instead of a continuous decelerate-and-
clamp.

### 2.4 No feedback from the guard back into the FUEL planner's internal state

`fast_exploration_fsm.cpp::EXEC_TRAJ` replans purely on **elapsed time / trajectory coverage**
(`fp_->replan_thresh1_/2_/3_`, `isFrontierCovered()`), evaluated against the planner's own internally-modeled
B-spline (`info->position_traj_.evaluateDeBoorT(t_r)`), not against actual odometry. Only when
`fd_->static_state_` is true (freshly re-triggered from hover) does replanning start from `fd_->odom_pos_`
(actual measured position). This means: while the guard is holding/rejecting near a Z boundary, FUEL keeps
"believing" its planned trajectory executed as commanded, and the *next* replan is seeded from that same wrong
assumption — not from where the drone actually is. The divergence introduced by §2.3 is never corrected by the
planner; it can only be reset by a full stop back to `static_state_`. This is why, per your description, mapping
doesn't cleanly "resume" after a boundary event — the planner's world-model and the drone's real position have
silently split.

### 2.5 No PX4-side backstop at all

Grepped the active airframe file
(`simulation/PX4-Autopilot-v1.14.3/ROMFS/px4fmu_common/init.d-posix/airframes/1023_gazebo-classic_iris_vlp16`)
for `GF_*` (geofence), `COM_POS_FS`, `LNDMC_*`, `COM_OF_LOSS`, `MPC_Z_VEL_*`, `FD_*` — **none are set**. PX4 has
no independent altitude/geofence protection; the Python guard is the *only* thing standing between a bad setpoint
and a physical ceiling/floor collision in Gazebo. Given §2.1–2.4, that single line of defense currently has a
gap exactly on the axis you're seeing failures on.

### 2.6 Test suite doesn't reflect current production config or this failure mode

`scripts/test_flight_envelope_guard.py` still hardcodes `world_z_min = 1.45` / `world_z_max = 1.55` (the old
narrow band), while the live `config/flight_envelope_guard.yaml` now uses `world_z_min = 0.3` /
`world_z_max = 2.0`. There is no test at all for Z velocity damping (because it doesn't exist) or for the
reject→resume transition. This is why the regression shipped unnoticed.

---

## 3. Proposed fix — phased

### Phase 0 (PRIMARY FIX): Constrain FUEL's own exploration ceiling at the source

**This is the fix that most directly matches the intended operational profile: hover at 1.5 m, allow up to 2.0 m
world altitude for safety/mission headroom, and if FUEL's frontier search would otherwise propose a point above
that, it should never be generated in the first place — the planner naturally moves on to the next feasible
frontier instead.**

Why this is better than reactively handling the crossing downstream (original Phases 1–4): those phases fight the
problem *after* FUEL has already committed to an illegal point and the drone is already moving toward it — the
best they can do is decelerate/clamp/recover more gracefully. Phase 0 removes the illegal point from FUEL's
search space entirely. `FrontierFinder::expandFrontier()` and `FrontierFinder::sampleViewpoints()`
(`active_perception/frontier_finder.cpp`) both bound-check every candidate against `sdf_map`'s box
(`edt_env_->sdf_map_->isInBox(...)`), which is set directly from `sdf_map/box_min_z` / `sdf_map/box_max_z` in
`algorithm.xml`. Tighten that box and FUEL structurally cannot propose, cluster, or score a frontier/viewpoint
above the ceiling — "reject and move to a feasible point" happens for free, as the normal behavior of the
existing TSP/frontier-selection logic, with no new reject/retry code needed.

**Exact change (see §Phase 5 for the frame-math verification behind this number):**
`sdf_map/box_max_z` in `catkin_ws/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml` should go from
`2.2` → **`1.9`** (camera_init frame), which maps to exactly `2.0` in world frame — matching
`config/flight_envelope_guard.yaml`'s `world_z_max`. `box_min_z = 0.2` is already correct (maps to world `0.3`,
matching `world_z_min` exactly) and needs no change.

The low-level guard (`flight_envelope_guard.py`) remains in place unchanged as a **last-resort backstop** for
odometry drift, EKF noise, or transient state-estimation error — not as the primary mechanism for keeping the
drone under the ceiling. Phases 1–2 below (margin + velocity damping) are still worth doing as defense-in-depth
for that backstop role, but Phases 3–4 (reactive clamp-and-resume, planner feedback loop) are downgraded to
optional hardening — see §4.

**UPDATE — `box_max_z` alone was applied and tested; it did not stop the crash.** Root cause of that gap, found
and fixed:

`box_min_z`/`box_max_z` only constrain two things: `FrontierFinder`'s frontier/viewpoint search
(`isInBox()` checks in `frontier_finder.cpp`) and the initial kinodynamic A* guide path
(`isInBox()` in `kinodynamic_astar.cpp:134,173`). They are a **logical search-space bound**, not a physical
obstacle. The actual flown trajectory comes from `BsplineOptimizer`, which smooths that initial guide path using
cost terms (smoothness, distance-to-obstacle, feasibility, start/end, guide, waypoint, view, time —
`bspline_optimizer.cpp`) — **none of which reference `box_min_z`/`box_max_z`**. The only thing that would stop the
optimizer from placing control points above the box is `calcDistanceCost`, and that only reacts to cells the SDF
map considers *occupied* — the ceiling isn't occupied, it's just outside a search-space rectangle nothing else
respects.

FUEL actually ships a mechanism for exactly this — a "virtual ceiling" that stamps a real occupied layer into the
map at a chosen height so the EDT distance-cost term treats it like a wall:
```cpp
// plan_env/src/sdf_map.cpp
if (mp_->virtual_ceil_height_ > -0.5) {
  int ceil_id = floor((mp_->virtual_ceil_height_ - mp_->map_origin_(2)) * mp_->resolution_inv_);
  ...
  md_->occupancy_buffer_[toAddress(x, y, ceil_id)] = mp_->clamp_max_log_;
}
```
It was disabled: `sdf_map/virtual_ceil_height` was `-10` in `algorithm.xml` (fails the `> -0.5` gate). **Fixed** —
set to `1.9`, matching `box_max_z` exactly (same camera_init frame, no offset needed since both params are
consumed directly by `sdf_map` in its native frame). This is now a *physical* obstacle the B-spline optimizer's
distance cost will push away from, not just a logical search boundary.

No equivalent "virtual floor" exists in `sdf_map.cpp` — only the ceiling case is implemented. This isn't a gap in
practice: the real arena floor is physical, sensed geometry (LiDAR sees it), so it's already a genuine occupied
obstacle in the SDF map and the optimizer already avoids it the normal way. Only the ceiling was virtual/logical
and therefore invisible to the optimizer.

**No rebuild required** — `algorithm.xml` and `nidar_fuel_upstream.launch` are pure roslaunch XML/params, read
fresh at every `roslaunch`. Just restart the stack (`test_takeoff.sh` or equivalent) to pick up both the
`box_max_z=1.9` and `virtual_ceil_height=1.9` changes together.

### Phase 1: Give Z the same predictive margin X/Y already has
- Add a dedicated `boundary_margin_z` (don't reuse the X/Y `boundary_margin` directly — vertical stopping
  dynamics differ from horizontal: different max climb/descend rate, and the usable Z band is much shorter than
  the X/Y arena, 1.7 m vs 14 m, so the same absolute margin eats a much bigger fraction of the range).
- Size it the same way `boundary_margin` was derived for X/Y previously (`s = v²/(2a)` using the vehicle's actual
  vertical max velocity/accel, plus headroom), not copied by feel.
- Apply it symmetrically: `eff_zw_min = world_z_min + boundary_margin_z`, `eff_zw_max = world_z_max -
  boundary_margin_z`.

### Phase 2: Add Z velocity damping mirroring the existing X/Y pattern
- Mirror the `xw_clamped <= eff_xw_min + 0.3 → vx_w = max(0, vx_w)` pattern for `zw_clamped` / `vz`, using a
  vertical-specific proximity threshold (not necessarily the same 0.3 m used for X/Y — pick it consistently with
  the Phase 1 margin sizing).
- This must run inside the `if is_valid:` branch alongside the existing X/Y damping, same code path.

### Phase 3: Replace hard-reject-and-freeze with continuous clamp-and-hold for boundary crossings
- Currently: Z-out-of-bounds → whole command rejected → stale `last_valid_command` streamed → discontinuity.
- Change to: when only Z is out of bounds (X/Y otherwise valid), **clamp Z to the boundary and still update
  `last_valid_command`/`last_valid_pos_world`** with the clamped position, instead of discarding the command
  outright. This keeps the streamed setpoint continuous with the drone's actual trajectory instead of snapping
  back to a stale pre-approach position. Decide deliberately whether this should be a `is_valid=True` result with
  a new `code` (e.g. `CLAMPED_Z`) for diagnostics, or a distinct third outcome in `validate_command`'s return
  contract — needs a clear design decision before implementation, not an ad hoc patch.
- Keep the existing outright-reject behavior for cases where clamping isn't safe to attempt (e.g., large
  simultaneous X/Y violations, or extended loss of valid odometry) — this phase is specifically about the
  "otherwise-valid trajectory that only slightly overshoots Z" case, which is what your description matches.

### Phase 4: Close the planner feedback gap
- Establish some signal path so `fast_exploration_fsm` knows when the guard has been actively clamping/holding
  (e.g., a status topic already exists: `/flight_envelope_guard/status` — currently only logged, not consumed by
  the FSM). At minimum, define what "resume mapping" should mean operationally: does the FSM need to force a
  `static_state_ = true` reset (replan from real `fd_->odom_pos_`) after a sustained guard intervention, so the
  next plan starts from truth instead of the diverged internal model? This needs a design decision on where that
  trigger lives (FSM subscribing to guard status vs. guard publishing directly to the FSM's replan-trigger topic)
  before implementation.
- This phase is the one most likely to explain "tries to cross, gets fixed, but doesn't cleanly resume mapping"
  specifically, as opposed to the crash itself (which Phases 1–3 target).

### Phase 5 (verification-only, no code) — **EXECUTED, mismatch confirmed**

**Goal:** determine whether FUEL's exploration box and the guard's hard ceiling actually agree on where "the
ceiling" is, before touching any code.

**Method:** traced the coordinate frame FUEL's `sdf_map` actually operates in. `nidar_fuel_upstream.launch`
wires `odometry_topic` = `/Fast_LIO/odometry` directly into `exploration_node`'s `/odom_world` — FUEL's internal
state (including `sdf_map/box_min_z`/`box_max_z` bound checks) is therefore expressed in **camera_init frame**
(FAST-LIO's native output frame), not world frame. This is the same frame `flight_envelope_guard.py`'s
`camera_to_world()` converts *from* (`zw = zc + 0.1`).

**Result:**

| Quantity | Value | Frame | Equivalent world Z |
|---|---|---|---|
| `sdf_map/box_min_z` (algorithm.xml) | 0.2 | camera_init | **0.3** — matches `world_z_min` exactly ✅ |
| `sdf_map/box_max_z` (algorithm.xml) | 2.2 | camera_init | **2.3** — guard's `world_z_max` is 2.0 ❌ |
| `flight_envelope_guard.yaml: world_z_min` | 0.3 | world | — |
| `flight_envelope_guard.yaml: world_z_max` | 2.0 | world | — |

**Finding confirmed: there is a real, permanent 0.3 m gap.** The floor bound was already consistent
(`box_min_z` happens to convert exactly to `world_z_min`). Only the ceiling was ever mismatched — FUEL's
frontier/viewpoint search is structurally allowed to target up to 2.3 m world altitude, a full 0.3 m above the
guard's hard 2.0 m ceiling. This is not an occasional edge case: any frontier whose best viewpoint naturally
falls in that 0.3 m band will be generated, selected, and committed to by FUEL every time it's the best available
option — then unconditionally rejected by the guard. This is very likely the dominant trigger for the reported
crash pattern, independent of everything in §2.1–2.4.

**Physical ceiling mesh check:** `simulation/custom_models/nidar_arena/meshes/mapdraw.stl` /
`nidar_world_arena.dae` are binary/opaque mesh geometry — exact ceiling height couldn't be extracted via static
analysis in this pass. Recommend confirming empirically (e.g. spawn the arena and check the collision height in
Gazebo, or ask whoever authored the arena mesh) once Phase 0 is applied, to confirm 2.0 m world leaves adequate
physical clearance under the real ceiling — this doc's fix corrects the FUEL/guard *config* mismatch, but doesn't
independently verify the arena's physical ceiling height against either bound.

**Conclusion: proceed with Phase 0** (`box_max_z: 2.2 → 1.9`) as the primary fix.

### Phase 6: Rebuild the test suite around this behavior
- Update `scripts/test_flight_envelope_guard.py` to use the current production `world_z_min/max` (0.3/2.0) instead
  of the stale 1.45/1.55.
- Add tests for: Z velocity damping near bounds, the new clamp-vs-reject decision from Phase 3, and (once Phase 4
  is scoped) the resume-after-clamp transition.

---

## 4. Suggested execution order (revised)

1. **Phase 5 — DONE.** Verified the box-height mismatch is real (0.3 m gap) and gave the exact corrective number.
2. **Phase 0 — do next.** Single-line config change (`box_max_z: 2.2 → 1.9` in `algorithm.xml`), lowest possible
   risk, directly implements the "hover 1.5 m / ceiling 2.0 m / reject-and-move-on" behavior you described as the
   actual desired operational model. This alone is expected to eliminate most Z-boundary crash events, since it
   stops FUEL from ever targeting the illegal band rather than trying to recover after the fact.
3. Test Phase 0 (see §6 below) before deciding whether Phases 1–2 are still needed at all, or only as
   belt-and-suspenders.
4. Phase 1 + 2 (Z margin + velocity damping in the guard) — keep as defense-in-depth against odometry
   drift/EKF noise even after Phase 0, but now optional-not-urgent rather than the primary fix.
5. Phase 6 partial — fix the test file's stale `world_z_min/max` (1.45/1.55 → 0.3/2.0) regardless, since it's
   wrong today independent of anything else in this plan.
6. Phases 3 and 4 (reactive clamp-vs-reject redesign, planner feedback loop) — **downgraded to optional,
   revisit only if Phase 0 + 1 + 2 still leave residual boundary incidents in testing.** Given Phase 0 removes
   the dominant trigger, the added complexity of these two phases (new `validate_command` return contract,
   cross-language FSM signaling) may not be justified at all.

## 5. Open decisions before implementation starts

- **Phase 0:** does `box_max_z = 1.9` leave enough headroom above `box_min_z = 0.2` for FUEL to still find useful
  frontiers/viewpoints in a 1.7 m-tall exploration volume? Should be fine (unchanged from today's usable band,
  just the top edge moves down 0.3 m to close the gap) but worth a sanity check against `frontier/cluster_size_z
  = 10.0` and viewpoint sampling behavior once tested.
- Exact `boundary_margin_z` value for Phase 1 (if still pursued) — needs the vehicle's real max vertical
  velocity/accel (PX4 `MPC_Z_VEL_MAX_UP`/`MPC_Z_VEL_MAX_DN`/`MPC_ACC_UP_MAX`, not yet checked).
- Whether Phases 3–4 are worth doing at all post-Phase-0 — defer this decision until Phase 0 is tested.
