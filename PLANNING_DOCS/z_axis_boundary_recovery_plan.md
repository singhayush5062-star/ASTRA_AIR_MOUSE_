# Implementation Plan — Z-Axis Boundary Crossing Causes Crash Instead of Soft Recovery

Status: **plan only, no code changed**. Root causes below are traced directly against
`scripts/flight_envelope_guard.py`, `config/flight_envelope_guard.yaml`, the PX4 airframe file, and
`catkin_ws/src/fuel/fuel_planner/exploration_manager/src/fast_exploration_fsm.cpp`.

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

### Phase 5 (verification-only, no code): Confirm no physical ceiling/floor collision margin issue
- Cross-check `world_z_max = 2.0` (guard) against the actual Gazebo arena mesh ceiling height
  (`simulation/custom_models/nidar_arena/meshes/mapdraw.stl` / `nidar_world_arena.dae`) and the FUEL exploration
  box (`sdf_map/box_max_z = 2.2` in `algorithm.xml`, in camera_init frame → world Z via `+0.1` offset ⇒ effective
  world 2.3). There's already a ~0.3 m mismatch between the guard's `world_z_max=2.0` and the exploration map's
  box in world frame (~2.3) worth resolving as part of this work, since FUEL may keep proposing viewpoints above
  what the guard considers legal, generating a continuous stream of Z violations rather than an occasional edge
  case.

### Phase 6: Rebuild the test suite around this behavior
- Update `scripts/test_flight_envelope_guard.py` to use the current production `world_z_min/max` (0.3/2.0) instead
  of the stale 1.45/1.55.
- Add tests for: Z velocity damping near bounds, the new clamp-vs-reject decision from Phase 3, and (once Phase 4
  is scoped) the resume-after-clamp transition.

---

## 4. Suggested execution order

1. Phase 5 first (pure verification, no risk) — confirms whether the box-height mismatch is even a contributing
   factor before touching guard logic.
2. Phase 1 + 2 together (margin + velocity damping) — same shape as the X/Y fix already proven in production,
   lowest-risk change, directly addresses "no early braking."
3. Phase 6 partial — get the test file matching current config before further changes, so Phases 1–2 can be
   verified against real assertions immediately.
4. Phase 3 (clamp vs. reject redesign) — needs the design decision flagged above resolved first; this is the
   piece most likely to eliminate the discontinuous "sudden fix" jump.
5. Phase 4 (planner feedback) — largest scope, touches FSM C++ code and cross-language (Python guard ↔ C++ FUEL)
   signaling; do last once 1–3 are validated to actually stop the crash, since it's solving the "doesn't resume
   cleanly" complaint rather than the "crashes" complaint.
6. Phase 6 remainder — finish test coverage once behavior is finalized.

## 5. Open decisions before implementation starts

- Exact `boundary_margin_z` value — needs the vehicle's real max vertical velocity/accel (not yet confirmed from
  PX4 `MPC_Z_VEL_MAX_UP`/`MPC_Z_VEL_MAX_DN`/`MPC_ACC_UP_MAX` params, which weren't checked in this pass).
- Whether Phase 3's "clamp instead of reject" should be a permanent behavior change or gated by how close the
  violation is (e.g., clamp for a small overshoot, still hard-reject for a large one).
- Where the Phase 4 trigger should live — guard-side publish vs. FSM-side subscribe — and whether that coupling
  is acceptable given the guard is meant to be a downstream safety net, not something the planner depends on
  functionally.
