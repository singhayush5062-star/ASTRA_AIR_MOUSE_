# Premature Landing / Incomplete Coverage — Root Cause #4 and Fix (2026-08-17)

## Context

The three fixes documented in `realtime_test_verification_2026-08-17.md` (yaw-whip clamp, vision-staleness
watchdog, FSM real-odometry fallback) were verified against live runs earlier the same day. A follow-up re-test
was requested because the drone was still coming down before covering the full maze. This doc covers that
re-test: what was actually observed (judged from real height + position telemetry, not just `armed`/`landed_state`
flags), the new root cause found, whether arena complexity is implicated, and the fix applied.

Test command (same as all prior runs, so results are directly comparable):
```
./scripts/test_takeoff.sh true 0.0 -6.5 0.1 1.5708
```

## Method: judging "premature landing" by height + position, not just armed/landed_state

Previous runs judged failure primarily off `landed_state`/`armed` from `/mavros/extended_state`. That flag never
actually flipped to `ON_GROUND` in any prior run, even during real distress, because PX4's own landing detector
requires conditions (low thrust + near-zero velocity sustained) that a still-armed, still-trying-to-hover vehicle
may never cleanly trigger. For this run, a dedicated watch script (`height_watch.py`) tailed the guard's diagnostic
CSV directly and judged the vehicle's state from the actual measured world-frame pose:
- height trend relative to its own rolling peak (flags a sustained drop of >0.5m below peak while under 1.0m)
- horizontal (x,y) position drift over a rolling 15s window (flags a stall — command activity with no real
  displacement)

This caught the real failure directly instead of waiting for a flag that may never assert.

## What happened in the re-test

**Phase 1 (t=0-85s): genuinely healthy.** Altitude oscillated between ~0.9m and ~1.76m — lower than the 1.5m
takeoff altitude but recovering each dip, never approaching the guard's floor. Horizontal position advanced
substantially and continuously, from the spawn room at world (0, -6.5) out past (9, -11) — real, wide-area
coverage. This is a materially different (and better) outcome than the two prior sessions' first ~90 seconds,
consistent with the three earlier fixes doing their job.

**Phase 2 (t~85-95s): a real physical crash.** Altitude dipped through the guard's floor region and did not
recover. A direct Gazebo ground-truth query (`/gazebo/get_model_state`) confirmed the vehicle was physically
resting on the ground at world (-1.74, 3.18, 0.055m) with near-zero velocity — an actual crash-landing, not a
sensor glitch.

**Phase 3 (t=95s onward): post-crash estimator runaway, previously undetected.** After the physical crash, FAST-
LIO's LiDAR-based localization — now looking at a degenerate, near-ground point cloud — lost tracking and began
free-integrating garbage. `/Fast_LIO/odometry` kept publishing at normal rate (so the existing vision-staleness
watchdog, which only checks *timing*, saw nothing wrong), but the *values* went unbounded: within ~60 seconds the
reported position drifted out to world y≈-80m, z≈-9m — physically impossible for a 15x15m arena. Two independent
consumers of this raw feed were poisoned:
- **PX4's EKF2** (via `relay_odometry.py` → `/mavros/vision_pose/pose`) — the divergence visible in
  `pose_world_*` in the guard's telemetry.
- **FUEL's exploration FSM** (`fast_exploration_fsm.cpp` subscribes to `/Fast_LIO/odometry` directly as
  `/odom_world`, bypassing the relay entirely) — `fd_->odom_pos_` itself became garbage
  (`start pos: -3660.13 -1003.72 -7867.04` appeared verbatim in the planner's own log).

This second point interacts directly with one of today's earlier fixes: the FSM's real-odometry fallback (fix #3
from the earlier verification pass) trusts `fd_->odom_pos_` unconditionally once it diverges from the predicted
trajectory by more than 1.0m. Once `fd_->odom_pos_` itself is the corrupted value, that fix guarantees the
kinodynamic search is permanently seeded from a physically impossible start position. The result was an endless,
silent failure loop — `open set empty, no path!` / `No path to next viewpoint` / `plan fail`, repeated **7,479
times** with an identical target, growing the log to 175k+ lines — while the vehicle (already on the ground)
sat motionless and no further useful commands were ever issued.

Why didn't `relay_odometry.py`'s existing jump/speed rejection filters catch this? They compare each sample only
against the *immediately preceding published* sample (reject if the single-step jump exceeds ~2.5m or implied
speed exceeds 6m/s). A slow, steady drift — a few centimeters per 20-50Hz callback — never trips a single-step
threshold, even though the *cumulative* drift over a minute is wildly implausible. This is a boiling-frog gap:
individually plausible steps, collectively impossible trajectory.

## Is this caused by arena complexity?

Partially, and indirectly — not in the sense of "a tunnel is too narrow to fly through," but in a real way:

- The arena (`nidar_arena` mesh, generated from a 2D image extrusion) is a genuine maze of full-height
  (0-2.44m) walls with `obstacles_inflation=0.199m`. Checked the region around the failure — walls extend floor
  to ceiling there, so this isn't a low-ceiling/tunnel issue.
- However, FUEL's own viewpoint sampling (`frontier_finder.cpp::sampleViewpoints`) sets every candidate
  viewpoint's altitude **exactly equal to the mean Z of the frontier cluster it's covering** — there is no
  independent vertical sampling ring and no minimum-altitude floor in candidate generation
  (`sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0)` — the Z direction component is
  literally zero). In a complex, tightly-walled arena, frontier boundaries detected near corners and narrow
  passages skew lower (the VLP-16's vertical FOV covers less usable range at close quarters to walls), which
  pulls the commanded altitude down to match — this is the mechanism behind the 0.9-1.76m oscillation seen even
  in the "healthy" first 85 seconds of this run, and is consistent with the same pattern seen in the previous
  session's failed run (a continuous ~30s altitude decay while stuck in a tight corner near the spawn room).
- Whether the specific *crash* at t~85-95s was a direct collision (misjudged clearance while diving to match a
  low frontier) or a coincidental FAST-LIO stall (the same CPU-contention-driven failure mode documented as root
  cause #2) could not be conclusively isolated from the available logs — `fast_lio.log` for this run doesn't
  contain an explicit failure message before the crash, and CPU load was moderate but non-trivial
  (~0.7-1.2 load average on 12 cores) during the run. Both explanations are consistent with "more likely to
  happen in the more complex/tighter regions of this arena than in open space."

## Fixes applied

**1. `fast_exploration_fsm.cpp` (`odometryCallback`) — reject implausible odometry at the source.**
Added an absolute sanity envelope (generous margin around the SDF map's own box: x∈[-3.5,16.5], y∈[-10,10],
z∈[-1,3] in camera_init frame). Any incoming `/Fast_LIO/odometry` sample outside this envelope is rejected
outright — `fd_->odom_pos_`/`odom_vel_`/`odom_yaw_` simply hold their last good value and a throttled
`ROS_ERROR` is logged. This is the direct fix for the poisoned-replan-seed loop: FUEL's FSM (and by extension the
real-odometry fallback from the earlier fix) can now never be handed a physically-impossible position, no matter
how gradually the underlying estimate drifted there. Verified via `catkin build exploration_manager` — succeeds
cleanly, only pre-existing unrelated warnings.

**2. `relay_odometry.py` — same class of check, defense in depth for PX4's EKF2.**
Added an absolute bounds check on the *raw* (pre relative-origin-adjustment) FAST-LIO position, same envelope as
above, checked before the existing single-step jump/speed filters. This stops a slowly-diverging estimate from
ever reaching `/mavros/vision_pose/pose`, protecting PX4's EKF2 independently of the FUEL-side fix — the two
consumers read the raw topic through entirely separate code paths, so both needed the same class of fix.
Verified via `python3 -m py_compile` — clean.

Both fixes are purely rejective (hold last good state / drop the sample) — they change no accepted-path behavior
at all, so normal healthy operation (including everything verified in the three earlier fixes) is unaffected.

## What this does and doesn't solve

- **Solves**: the silent, unbounded replan-seed poisoning loop and the EKF runaway that followed the crash —
  once this class of failure occurs, the system will now hold position and log loudly instead of computing
  nonsense forever.
- **Does not solve**: the initiating event itself (why the crash/stall happened at t~85-95s). That remains
  either a direct collision from the low-altitude-viewpoint-following behavior described above, or a recurrence
  of the CPU-contention-driven FAST-LIO stall from root cause #2 — most plausibly the former, given the
  vertical-tracking-to-frontier-height mechanism confirmed in the code, and given full-coverage confirmation was
  not obtained in either of today's two live runs.

## Recommended follow-up (originally not applied — now partially followed up, see next section)

- Add a minimum-altitude floor to `sampleViewpoints()` (clamp `sample_pos.z()` up to a `frontier/min_candidate_z`
  parameter before the visibility check) so FUEL never proposes a viewpoint below a safe cruise height, even
  when the frontier it's covering is itself low. This directly targets the arena-complexity-linked altitude-decay
  mechanism identified above, rather than just containing its downstream consequences.
- Re-run the same test with these two odometry-sanity fixes in place to confirm whether the mission now survives
  past t~90s in this specific spawn/arena configuration, and if a crash still occurs, capture whether it's
  collision-triggered (should now show a definite obstacle proximity in the SDF map right before impact) or
  stall-triggered (should show a `fast_lio.log` gap, matching root cause #2's signature).

## Verification run #4 — odometry-sanity fixes hold, but a fifth, distinct root cause found

Re-ran the same test after: (a) the two odometry-sanity fixes above, and (b) the separate altitude-band fix from
`height_hold_baro_fastlio_analysis_2026-08-17.md` (`frontier/min_candidate_z`/`max_candidate_z` = [1.2, 1.6]
camera_init). Judged live via the same height+position watch method.

**Good news — both fixes from this doc worked as intended.** No runaway/poisoning this time: `pose_world` and
FUEL's own `start pos` in `fuel.log` stayed within physically plausible values throughout, even once the vehicle
got into trouble again (no repeat of the -80m/-3660m divergence from run #3). The altitude band also worked for
the first ~60 seconds: cruise height held in the 0.9-1.74m range (tighter than the previous run's 0.9-1.76m swing,
with visibly less time spent at the extremes) while covering real ground (world position moved from spawn out to
multiple meters away).

**But a real physical crash happened again, in the same corner.** Around t~64s, the vehicle got wedged at world
≈ (-6.2, -6.2) — the same spawn-adjacent corner flagged as a "boundary-corner deadlock" in the original
same-day verification doc — with kinodynamic A* repeatedly failing (`open set empty, no path!`). Unlike run #3,
this stall did *not* immediately cascade into estimator garbage; instead, position and altitude held steady at a
physically plausible value (world z≈0.9m) for ~50+ seconds while FUEL kept retrying. A direct Gazebo ground-truth
query eventually confirmed the vehicle came to rest, motionless, at world (-6.28, -6.10, 0.055m) — a real
crash-landing at essentially the exact stuck location.

Checked the arena mesh (`mapdraw.stl`) directly around that coordinate: there is a real wall segment at x≈-6.69
and another at y≈-6.2, forming an actual corner right where the vehicle got wedged and eventually came down. This
is consistent with a genuine collision/clearance failure at a real pinch point in the arena geometry near the
spawn room, not a sensor or estimator artifact — **this is a fifth, distinct root cause**, not yet fixed:

FUEL's kinodynamic search and B-spline optimizer, when repeatedly failing to find a path out of a tight corner
(A* returns `NO_PATH`, `open set empty`), have no explicit "back away / retreat" behavior — they keep retrying
the same blocked target from the same wedged position indefinitely. If the real vehicle is pressed close enough
to the corner's geometry for `obstacles_inflation` (0.199m) to leave no feasible cell, nothing in the current
stack commands it to retreat to open space and re-approach — it just keeps failing to plan in place until either
it gets nudged into contact by residual velocity/wind-up, or (as seen here) simply never recovers cleanly.

**Not fixed in this pass** — flagging for the next iteration:
- A "stuck-and-retreat" behavior: after N consecutive `NO_PATH` results at the same location, command a
  short retreat along the corridor the vehicle arrived from (available from recent trajectory history) before
  re-attempting frontier search, rather than retrying the same blocked local search indefinitely.
- Alternatively/additionally, reduce `obstacles_inflation` specifically in known tight regions, or increase
  `candidate_rmax`/search radius so the kinodynamic search has more room to find an alternate approach angle to
  the same frontier without needing to enter the tightest part of the corner at all.
- This spawn-adjacent corner should be treated as a known problem spot for this specific arena/spawn combination
  — worth deciding whether to spawn further from it, or fix the corner-approach behavior generally before further
  testing focuses elsewhere.
