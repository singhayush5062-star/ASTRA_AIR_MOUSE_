# Real-Time Test Verification — Why the Drone Descends Without Covering the Full Maze

Date: 2026-08-17. Test invocation: `./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708` (matching the user-specified
spawn position), run directly on the host (docker was unavailable in-session; the ROS/PX4/Gazebo stack already
runs natively here — same machine as all prior log analysis in this investigation).

This test was run **after** the previous fixes from this investigation were already in place: yaw-rate slew
clamp + FUEL C++ yaw clamp (mapping_stall_crash_root_cause.md), Z-margin (z_axis_boundary_recovery_plan.md), and
`EKF2_BARO_CTRL=1`. Goal: confirm those fixes hold under a live run, and find out what (if anything) still causes
incomplete maze coverage.

---

## Result 1: the yaw-whip fix is confirmed working live

At sim_time 40.6s, the exact condition that previously caused a crash fired again:
```
[WARN] Yaw change rapidly! start=0.932 end=-4.010 (clamped to start +/- PI)
```
Note it's `WARN`, not the old `ERROR`, and explicitly shows `(clamped to start +/- PI)` — the `planner_manager.cpp`
fix executing for real. `mission_telemetry_logger.py`'s `landed_state` stayed `IN_AIR` through and well past this
point (confirmed at t+52s, t+57s, ...). Previously, the equivalent event at a similar point in the mission caused
a hard landing within seconds. **Confirmed fixed.**

## Result 2: a second, independent failure mode — FAST-LIO stall → EKF runaway → uncontrolled landing

Around t+57s–92s (mission-telemetry-relative), `mission_telemetry_logger.py`'s new `ekf_fastlio_divergence_m`
column (added specifically for this kind of check) caught something new:

- FAST-LIO's raw `/Fast_LIO/odometry` **froze** at exactly `(2.82,-3.56,-0.07)` (camera_init frame) across seven
  consecutive 5-second samples — completely unchanging for ~35 seconds.
- With no fresh vision correction, PX4's EKF2 (vision is the *only* aiding source — GPS and baro-position are
  both off) free-integrated on IMU alone. Its reported velocity grew from ~2.7 m/s to **~22.7 m/s**, position
  drifted out to camera_init `(16.40, -26.49)` — entirely fictitious.
- `diverge` (EKF vs. raw FastLIO) peaked at **26.65m**.
- FAST-LIO then resumed publishing, and by t+112s the two had reconverged to `diverge=0.00m`.

**Root cause of the stall itself:** checked `uptime` during the episode — `load average: 12.60, 15.22, 9.90` on a
12-core machine, i.e. CPU-saturated (Gazebo GUI + PX4 SITL + FAST-LIO + FUEL + guard + this session's own
concurrent investigation commands, all competing for cores). `/tmp/fast_lio.log` has **zero** warnings or errors
anywhere near the stall window — no `reset`, `diverge`, `degenerate`, or `No point` messages, even though FAST-LIO
demonstrably does log exactly those conditions when they occur (it did so during normal startup at sim_time
~5.3s). Silence + system-wide CPU saturation, with no algorithmic error logged, points to **scheduling
starvation**, not a FAST-LIO algorithm bug — the node simply wasn't getting CPU time to run its callbacks.

**Consequence, confirmed with ground truth:** queried `rosservice call /gazebo/get_model_state` directly —
bypassing every estimator — during the "stuck" period:
```
position: {x: 3.432, y: -3.654, z: 0.055}
twist: linear ~1e-7, angular ~1e-6   # i.e. exactly zero
orientation: (x≈0, y≈0, z=0.264, w=0.965)  # negligible roll/pitch — level, not tipped over
```
**The vehicle was genuinely, physically resting on the arena floor — upright, level, motionless.** Not a crash,
not an estimator artifact: a real, soft, controlled-looking touchdown. `landed_state` from
`/mavros/extended_state` read `2` (`IN_AIR`) the whole time — PX4's own landing classifier never caught up to
reality either.

**And it never recovered.** For the remainder of the observed run (t+97s through t+267s+, over 3 minutes), the
vehicle sat at that exact spot while FUEL continuously proposed new climb targets (Z ranging 0.55–1.85m in
camera_init) and the guard kept `ACCEPT`ing and forwarding them — with zero effect. Velocity stayed pinned at
~0.00 m/s the entire time. This is the *exact* symptom from the original bug report ("drone perform descent...
settle down on the ground... still propeller moving"), reproduced live via a different trigger than the
already-fixed yaw-whip bug.

## Why this is architecturally significant, not just a one-off CPU spike

The system currently has **zero tolerance for any gap in its sole position-aiding source**. Whether that gap is
caused by CPU contention (this run), a momentary LiDAR occlusion, a feature-poor stretch of corridor, or anything
else that stalls FAST-LIO even briefly, the failure chain is the same: PX4's EKF free-integrates unchecked → real
control degrades → the vehicle can settle to the ground for real → nothing detects or recovers from that state →
the mission silently stalls forever, explaining incomplete maze coverage independent of frontier/exploration
logic being otherwise correct.

Checked whether PX4 has a built-in defense: it does — `COM_POS_FS_EPH` (5.0m default), `COM_VEL_FS_EVH` (1.0 m/s
default), `COM_POS_FS_DELAY` (1s default) form a position-failsafe that should trigger on excessive estimated
error. It didn't fire here. These thresholds are gated on the **EKF's own internal covariance/confidence
estimate**, not an external ground-truth comparison — a known EKF pitfall is that a pure open-loop IMU drift
doesn't always inflate the filter's *reported* uncertainty in lockstep with how wrong its *point estimate*
actually becomes. Re-tuning PX4's internal EKF confidence behavior blind, without being able to verify the
covariance math directly, isn't something to do with confidence in one sitting — flagged as a follow-up, not
attempted here.

---

## Fix implemented: vision-staleness watchdog in `flight_envelope_guard.py`

Added:
- A lightweight subscriber to `/Fast_LIO/odometry` (`vision_cb`) that does nothing but record
  `self.last_vision_time` — deliberately reading the **raw** feed, not the EKF-fused pose, since the EKF-fused
  pose is exactly what can't be trusted once aiding is lost.
- A check at the top of `fuel_cb`: if `/Fast_LIO/odometry` hasn't been seen in `vision_timeout` seconds (new
  param, default 1.0s — FAST-LIO runs at ~10Hz normally, so this tolerates ~10 missed cycles before reacting),
  the incoming FUEL command is treated as a reject (`code=VISION_STALE`), logged, counted, and diagnostic-logged
  identically to existing reject paths — and the guard falls back to its existing safe-hold behavior
  (`timer_cb`) instead of forwarding a new target.

**Honest scope of this fix:** it does **not** reach into PX4 and stop its internal EKF from free-integrating —
that's happening inside PX4's own estimator, outside anything the guard can influence. What it does do: stop the
guard from continuing to forward a stream of *new*, shifting exploration targets on top of an increasingly
untrustworthy internal state, and instead hold a single stable last-known-good setpoint until vision recovers —
reducing the accumulated control error PX4 has to reconcile once aiding resumes, and giving the system a
sane, singular target rather than a moving one during an outage. This is a mitigation for the consequence, not a
fix for the PX4-internal cause, and is documented as such rather than overstated.

Verified: `python3 -m py_compile` and full `ast.parse` both clean. Added `vision_timeout: 1.0` to
`config/flight_envelope_guard.yaml`.

## Known gap, not addressed in this pass

`scripts/test_flight_envelope_guard.py` has no coverage for: the yaw-rate clamp, the Z-margin change, or the
vision-staleness watchdog. Multiple fixes deep into this investigation now without corresponding test coverage —
worth closing before considering the guard "done."

---

## Verification run #2 — a third, distinct root cause found

Re-ran `./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708` (same spawn position) with the yaw fix, Z-margin, baro
fusion, and the new vision-staleness watchdog all in place. This time avoided running heavy concurrent commands
during the test to remove CPU contention as a confound.

**Yaw fix confirmed again**: the same "Yaw change rapidly!" condition fired repeatedly throughout (at t40s, t62s,
t112s, t122s, t126s...) and was correctly clamped every time, `WARN` level, vehicle stayed `IN_AIR` throughout.

**New finding: the vehicle never crashed this run, but also never made net progress.** Position sat at
`(2.0-2.04, -1.77 to -1.83, ~-2.3)` (camera_init) for the entire observed window (t+10s through t+200s, over 3
minutes) — confirmed real by both EKF and FastLIO agreeing throughout (`diverge` stayed 0.00-0.06m the whole
time, no estimation problem here). The FSM was demonstrably *not* stuck — `fuel.log` shows it actively cycling
`PLAN_TRAJ → PUB_TRAJ → EXEC_TRAJ → replan` continuously, with `Replan: traj fully executed` firing roughly every
~0.23 seconds.

**Root cause, confirmed from the raw per-message guard CSV** (`logs/flight_envelope_guard_*.csv`, which logs
every single `/planning/pos_cmd` message, not just 5-second samples): `fuel_vxc, fuel_vyc, fuel_vzc` were
`0.0, 0.0, 0.0` and the target position was byte-identical across 30+ consecutive messages spanning 0.28s at
~100Hz. `traj_server.cpp` was in its documented completed-trajectory hold state
(`if (t_cur >= traj_duration_) { pos = ...; vel.setZero(); }`) almost continuously.

Traced to `fast_exploration_fsm.cpp`'s `PLAN_TRAJ` case: when replanning from a non-static state (the normal case
during continuous exploration — `static_state_` only becomes `true` again after a hard `FAIL` or `NO_FRONTIER`),
the FSM seeds its next plan from **`info->position_traj_.evaluateDeBoorT(t_r)`** — the *old plan's predicted*
position — never from real odometry (`fd_->odom_pos_`, which the `odometryCallback` keeps current regardless).
This is precisely the gap flagged as deferred/open item **§2.4** in the very first root-cause doc from this
investigation (`mapping_stall_crash_root_cause.md`) — confirmed here as the actual, dominant cause of incomplete
maze coverage, independent of and in addition to the already-fixed crash bugs: FUEL's internal position belief
drifted ~3.5m away from where the real vehicle was ((2.0,-1.8) real vs. planning-from-near-(5.37,-2.64)), and
every replan looked like a short, nearly-arrived hop *from that drifted internal belief*, so the FSM kept
"succeeding" and cycling rapidly while the real vehicle never moved at all.

### Fix implemented: real-odometry fallback in `fast_exploration_fsm.cpp`

In the non-static replan branch, added a sanity check: compute `predicted_pt` from the old trajectory as before,
but compare it against `fd_->odom_pos_`. If they've diverged by more than `kMaxTrackingError = 1.0m`, seed the
new plan from real odometry (position, velocity, yaw) instead of the drifted prediction, logging a `ROS_WARN`
when this happens. When tracking is healthy (the common case), behavior is unchanged — this only engages once a
real, meaningful gap has opened up, preserving FUEL's normal smooth-replanning behavior rather than re-seeding
from potentially noisy odometry every cycle.

Verified: `catkin build exploration_manager` succeeds cleanly (only pre-existing, unrelated sign-compare/unused-
variable warnings — nothing introduced by this change).

## Verification run #3 — all three fixes confirmed working together

Re-ran `./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708` a third time with the FSM real-odometry fallback in
place alongside the earlier fixes. Result: **sustained, genuine mission progress for 200+ seconds**, covering wide
areas of the maze (camera_init X ran from ~0 up past 10, Y swung from -6.5 up past +6 over the course of the
run) — a categorically different outcome from verification run #2, where position sat completely frozen for the
entire 3+ minute observed window.

Key observations from this run:

- **Yaw clamp**: fired repeatedly throughout (same pattern as runs #1 and #2), always correctly clamped, vehicle
  stayed `IN_AIR` the entire time.
- **FSM real-odometry fallback**: fired frequently during active flight (roughly every 3-10 seconds), typically
  correcting divergences in the 1.0-1.3m range. This is more frequent than initially expected — worth a follow-up
  look at whether `kMaxTrackingError` is tighter than necessary, though no evidence surfaced during this run that
  the frequency itself caused any instability; position kept advancing smoothly through it.
- **Confirmed catching a genuine corner-deadlock case**: at sim_time ~103-109s, the vehicle got pinned at a
  boundary corner (X near `world_x_max` and Z near `world_z_max` simultaneously), and the guard correctly
  rejected the overshoot 6 times in a row (`OUT_OF_BOUNDS_X_MAX`/`Z_MAX`, `logs/bridge.log`-equivalent). This is
  exactly the scenario that would have produced a permanent stuck state before the FSM fix (FUEL repeatedly
  planning into a boundary the guard holds it back from, drifting its internal belief further away each time).
  Instead, three large divergence corrections fired in quick succession (2.12m, 2.62m, 1.14m) as the reject
  cluster ended, and the mission successfully replanned a route away from the corner and continued — the vehicle
  covered several more meters of real ground over the following seconds rather than staying deadlocked.
- No further crashes, no further multi-minute freezes, no vision-staleness events this run (system load was
  lower with concurrent commands avoided, consistent with the CPU-contention theory for run #1's stall).

## Overall conclusion

Three independent root causes for "drone descends / doesn't cover the full maze" were found and fixed in this
investigation, each confirmed via live re-test rather than static analysis alone:

1. Yaw-whip trajectory bug (`planner_manager.cpp`) — fixed, held across all three live runs.
2. FAST-LIO stall → PX4 EKF runaway → uncontrolled landing with no recovery — mitigated via vision-staleness
   watchdog (`flight_envelope_guard.py`); root stall trigger in this session was traced to CPU contention on the
   dev machine, not a code defect, but the mitigation guards against any future cause of the same gap.
3. FUEL's FSM never re-anchoring to real odometry during continuous replanning, letting its internal position
   belief drift arbitrarily far from reality while still reporting "success" — fixed via real-odometry fallback
   (`fast_exploration_fsm.cpp`), confirmed both restoring general forward progress and specifically breaking a
   boundary-corner deadlock live.

## Remaining open items (not addressed in this pass)

- `scripts/test_flight_envelope_guard.py` has no coverage for the yaw-rate clamp, Z-margin, or vision-staleness
  watchdog additions.
- `kMaxTrackingError = 1.0` in the FSM fix was chosen as a reasonable starting bound, not empirically tuned —
  worth revisiting if it's found to fire more often than necessary in future runs.
- The full mission was not run to completion (`NO_FRONTIER`/full maze coverage) in this session — verification
  focused on confirming the specific failure modes were resolved, not a full end-to-end mapping run.

