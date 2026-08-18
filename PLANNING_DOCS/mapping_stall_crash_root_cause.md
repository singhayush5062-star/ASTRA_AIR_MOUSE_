# Finalized Root Cause — "Drone Descends and Settles on Ground, Props Still Spinning, No Full Mapping"

Status: **root cause confirmed from log evidence**, fix implementation pending your go-ahead (see §5).
Diagnostic tooling (`scripts/mission_telemetry_logger.py`, per-message CSV in `flight_envelope_guard.py`) is what
made this traceable — this doc is the payoff of that instrumentation.

---

## 1. Judgment call on Hypothesis A/B — both refuted by data

You proposed Hypothesis A (FUEL's planner stuck retrying, frozen target) based on the t+20–60s telemetry snippet.
Asked to verify it against `logs/mission_telemetry_1786881447.csv`. **It does not hold up**, and neither does
Hypothesis B:

- **Hypothesis A (frozen FUEL target) — refuted.** In the CSV, `fuel_target_changed` is `True` on almost every row
  from t=40s through t=580s+. FUEL is actively producing new targets continuously. `/tmp/fuel.log` confirms this
  independently — `Replan: traj fully executed` fires repeatedly throughout the entire run (checked up to
  sim_time 750s+), meaning the planner keeps successfully completing and replanning trajectories. It never got
  stuck in a `PLAN_TRAJ`/`FAIL` loop.
- **Hypothesis B (EKF/vision divergence) — refuted.** `ekf_fastlio_divergence_m` stays at 0.01–0.03m throughout
  the log, including through the fall. PX4's EKF2 and FAST-LIO's raw odometry agree closely the entire time —
  this is not a state-estimation failure. Whatever happened, it happened to the *real, physical* vehicle, not to
  its position estimate.

## 2. What the data actually shows

Reading the CSV row by row:
- **t=35s** (`wall_time`≈82.09s sim time): `ekf_z=1.227`, `fastlio_z=1.265` — normal hover altitude, actively
  exploring (x,y moving across rows 1–9).
- **t=40s** (`wall_time`≈87.09s sim time): `ekf_z=0.309`, `fastlio_z=0.244` — **on the ground**. Both EKF and raw
  FAST-LIO agree independently, so this is a real physical event, not an estimator glitch.
- **From t=40s to the end of the log (t=580s+, 9 minutes)**: the vehicle never climbs back above ~0.3–0.5m,
  despite `guard_last_status` showing `ACCEPT` on nearly every row, with FUEL continuously commanding targets
  around Z=0.9–1.2m the entire time. The guard is doing its job correctly — it keeps forwarding legitimate climb
  commands — but the vehicle physically never responds.

## 3. Root cause, pinned to the exact second

`/tmp/bridge.log` (the guard's own log) shows the first "vehicle out of margin" warning at **sim_time 85.096s**,
inside the 82–87s window identified above. `/tmp/fuel.log` at **sim_time 83.336s** — right in the middle of that
window — has a single, telling line:

```
dt_yaw: 0.137455, start yaw: -2.70316 0.0270678 -0.0228503, end: -0.898593
[ERROR] [1786881502.212977090, 83.336000000]: Yaw change rapidly!
```

Traced to `catkin_ws/src/fuel/fuel_planner/plan_manage/src/planner_manager.cpp:826-829`:
```cpp
// Debug rapid change of yaw
if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI) {
  ROS_ERROR("Yaw change rapidly!");
  std::cout << "start yaw: " << start_yaw3d[0] << ", " << end_yaw3d[0] << std::endl;
}
// ... optimization proceeds anyway, unmodified, right below this block
```

**This check is diagnostic-only.** When FUEL's yaw-trajectory generator computes a required yaw change of ≥180°
between the start and end of a segment, it prints a warning and then **uses that boundary condition anyway** —
feeding it straight into the B-spline optimizer (`bspline_optimizers_[1]->optimize(...)`), which fits a smooth
curve to it. With `dt_yaw` as short as 0.09–0.4s in several of these events (this one: 0.137s for a ~π rad swing),
the resulting yaw trajectory implies an angular rate on the order of tens of rad/s — utterly unrealistic for a
quadrotor, and roughly two orders of magnitude above `heading_planner/max_yaw_rate` (configured to ~0.175 rad/s
in `algorithm.xml`, which this code path doesn't consult at all — it's a separate, simpler "look-forward" yaw
generator).

That command flows **unfiltered** through `traj_server` → `flight_envelope_guard.py` → MAVROS → PX4. The guard
validates X/Y/Z position and velocity in detail, but **never touches `yaw` or `yaw_dot` at all** —
`target.yaw = msg.yaw` is a straight pass-through (`flight_envelope_guard.py:238`). PX4 attempts to execute the
violent yaw command, and the vehicle destabilizes — matching the abrupt Z-drop from 1.2m to 0.3m observed within
seconds of the 83.336s error.

**This is not a one-off.** `grep -c "Yaw change rapidly" /tmp/fuel.log` → **20 occurrences** across the ~700s
run, clustered — five of them land between sim_time 72s and 96s, i.e. immediately before and after this exact
crash. The cluster coincides with a run of FUEL-internal `Replan: collision detected` events (5 in ~6 seconds
around t=72–78s per `/tmp/fuel.log`'s `checkTrajCollision` output) — consistent with the vehicle navigating a
tight space where consecutive viewpoints require sharp view-direction reversals, which is exactly what forces
large yaw deltas between segments.

## 4. Why it never recovers ("props still moving")

No crash/tip-over failsafe is configured anywhere in the PX4 airframe file (`FD_FAIL_P`, `FD_FAIL_R`, or PX4's
Fail Detector generally — none set, confirmed by grep in an earlier pass of this investigation). Once a violent
yaw command destabilizes and drops the vehicle, PX4 has no independent mechanism to detect "this is a crash,
disarm" — it just stays armed in OFFBOARD, motors idling, waiting for a setpoint it can no longer physically
reach. This explains the "props still moving" observation precisely: it isn't a soft controlled landing the
system could recover from, it's a real physical crash with no failsafe behind it, sitting there accepting
climb-back commands that a tipped/downed vehicle cannot execute.

## 5. Debugging/fix plan — implemented

1. **[Done, diagnostic-only]** `mission_telemetry_logger.py` now also logs `/mavros/extended_state`
   (`landed_state`: `ON_GROUND`/`IN_AIR`/etc.) and yaw for both EKF and FUEL target every 5s. Next run will show
   PX4's own belief about ground contact directly, instead of inferring it from Z alone.
2. **[Done]** Added yaw-rate slew limiting to `flight_envelope_guard.py` (`_slew_limit_yaw`, wired into `fuel_cb`
   where `target.yaw` is set). Tracks the previously-commanded yaw and steps toward each new target by at most
   `max_yaw_rate` (new param, `config/flight_envelope_guard.yaml`, default 1.5 rad/s — generous relative to normal
   exploration turning at `heading_planner/max_yaw_rate`≈0.175 rad/s, far below the ~20+ rad/s pathological spikes)
   per call, taking the shorter angular direction. This is the safety net: it bounds the commanded yaw stream
   regardless of what upstream in FUEL produces it. Verified: `python3 -m py_compile` and full AST parse clean.
3. **[Done]** Fixed the actual generation bug in `planner_manager.cpp:826` (`FastPlannerManager::planYawExplore`).
   Root cause of *why* the check ever triggers despite `calcNextYaw` bounding each individual step to ≤π: the
   `lookfwd` waypoint-chaining loop reassigns `last_yaw` repeatedly, so a sequence of individually-safe ≤180° steps
   can accumulate into a much larger total swing between the true start and the final boundary condition, which
   then has to be bridged within one fixed-duration B-spline segment. Changed the previously log-only branch to
   clamp `end_yaw3d[0]` to ±π from `start_yaw3d[0]` (preserving direction) before it's used to build the B-spline
   boundary condition — bounds the worst case to π/duration_ instead of unbounded. Minimal diff: normal-case
   behavior (condition false) is untouched; only the already-detected pathological branch changes. Verified:
   `catkin build plan_manage exploration_manager` — both succeed, only pre-existing unrelated warnings.
4. **[Done]** Enabled PX4's flight termination on FailureDetector trip. Checked PX4 v1.14.3 source directly:
   `FD_FAIL_P`/`FD_FAIL_R` already default to 60°, `FD_FAIL_P_TTRI`/`FD_FAIL_R_TTRI` to 0.3s — sane stock
   detection thresholds, already active. But `CBRK_FLIGHTTERM` defaults to `121212`, which is PX4's literal
   "disable flight termination even if FailureDetector triggers" value
   (`src/lib/circuit_breaker/circuit_breaker_params.c`) — that's precisely why a tipped vehicle stayed armed with
   idling motors instead of disarming. Added `param set-default CBRK_FLIGHTTERM 0` to the airframe file
   (breaker not tripped = termination logic active). `@reboot_required true` per PX4's own param doc, but
   `test_takeoff.sh` already does a full PX4 SITL restart every run, so this takes effect automatically on the
   next test. Verified: `sh -n` syntax check clean.
5. **[Deferred, lower priority, not yet the direct cause of this crash]** The virtual-ceiling fix from the prior
   investigation isn't airtight either: `/tmp/bridge.log` shows a genuine `OUT_OF_BOUNDS_Z_MAX` reject with
   `camera_init z=2.47` (0.57m above the 1.9 virtual ceiling) at sim_time 33-34s in this same run — unrelated in
   time to this crash, but confirms the B-spline optimizer's soft distance-cost against the virtual ceiling can
   still be overridden by other cost terms on occasion. Worth a follow-up pass once this fix is validated in a
   test run.

## Next step

Re-run `test_takeoff.sh`. Watch `mission_telemetry_logger.py`'s output for `landed_state` and confirm it stays
`IN_AIR` through what would previously have been a crash window; grep the new run's `/tmp/fuel.log` for
`Yaw change rapidly` — it should still appear (the underlying geometry that demands a large swing hasn't changed)
but should no longer coincide with an altitude drop, since it's now clamped rather than passed through raw.
Known test-suite gap not addressed here: `scripts/test_flight_envelope_guard.py` still has no coverage for the
new yaw-rate clamp (or the Z-margin change from the previous investigation) — worth closing before considering
this fully done.
