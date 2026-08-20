# Why routing barometer data through FAST-LIO is the wrong fix for altitude oscillation

## The question being answered

"Since we're now getting height data from the barometer, why is the drone still not holding a static
1.5m — it keeps moving up and down? Plan a method by which FAST-LIO publishes height from the barometer
so it can hold a static height. Correct me if I'm wrong."

**Short answer: the premise is wrong on two independent counts, not just architecture.** FAST-LIO does not
and structurally cannot easily consume barometer data, and even if it did, that would not make the drone hold
1.5m — height *estimation* accuracy and height *command* are two different subsystems, and the oscillation you're
seeing is coming entirely from the command side, which barometer fusion doesn't touch at all.

## Fact check: FAST-LIO has no barometer input today

Checked `catkin_ws/src/FAST_LIO/src/laserMapping.cpp` directly. The node's only sensor subscriptions are:
```cpp
ros::Subscriber sub_pcl = ... nh.subscribe(lid_topic, 200000, standard_pcl_cbk);  // LiDAR point cloud
ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);              // IMU
```
No barometer topic, no `baro`/`barometer` string anywhere in the package. FAST-LIO's estimator (an iterated
error-state Kalman filter, IKFoM) has a fixed measurement model built around LiDAR point-to-plane residuals plus
IMU preintegration. Barometric pressure is not one of the measurement types it's built to fuse — adding it would
mean writing a new residual/Jacobian term into the filter's core update step, a nontrivial modification to a
third-party SLAM package's internals, not a parameter you can flip on.

## Fact check: `EKF2_BARO_CTRL` already applied is on a completely different estimator, for a different purpose

EKF2_BARO_CTRL 1 (set earlier this session in the PX4 airframe config) controls **PX4's own onboard EKF2**,
which is a wholly separate estimator from FAST-LIO, running inside the flight controller. Its job is to fuse:
- vision (relayed from FAST-LIO's odometry via `relay_odometry.py` → `/mavros/vision_pose/pose`)
- IMU
- barometer (now enabled) as a cross-check specifically against vision height, to catch/limit vision drift

This makes PX4's height *estimate* more robust — it does **not** feed anything back into FAST-LIO, and it does
**not** change what altitude the drone is told to fly to. It's a pure estimation-quality improvement, already
in place, doing its job correctly and separately from the symptom you're describing.

## Where the commanded altitude actually comes from

Two layers, neither of which is barometer- or FAST-LIO-estimation-related:

1. **FUEL's exploration planner** decides *where* to fly, including Z, based on where the frontier (boundary
   between mapped and unmapped space) currently is. Confirmed in `frontier_finder.cpp::sampleViewpoints()`:
   ```cpp
   Vector3d sample_pos = frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0);
   ```
   The offset direction has a **zero Z component** — every candidate viewpoint's altitude is set to exactly the
   mean height of the frontier cluster it's trying to see. There is no independent vertical exploration layer and
   (until today's fix) no minimum-altitude floor. 1.5m only appears once, in `test_takeoff.sh`, as the initial
   **takeoff** target (`MIS_TAKEOFF_ALT`) — it is never re-asserted as a hold setpoint once exploration begins.
2. **Flight Envelope Guard** (`flight_envelope_guard.py`) only enforces hard *safety bounds* (currently world Z
   ∈ [0.5, 1.8] effective, after margin) — it rejects commands outside that band and holds the last safe
   position, but does not steer the vehicle back toward any particular altitude; it's a boundary, not a target.

So: the drone moves up and down because FUEL is intentionally chasing frontiers at whatever height they're
detected, and nothing in the stack currently pulls it back toward a preferred cruise altitude. Barometer fusion,
wherever it lives, cannot change this — it only affects how *accurately* the drone knows its current height, not
what height it's told to go to.

## Today's related fix (context)

As part of a separate crash investigation today (see `premature_landing_root_cause_2026-08-17.md`), a floor was
added: `frontier/min_candidate_z = 0.9` (camera_init frame, ~1.0m world) in `algorithm.xml`, enforced in
`sampleViewpoints()`. This stops candidates from going dangerously low but is a **floor, not a hold** — FUEL is
still free to climb well above it (up to `candidate_rmax`/box limits) whenever a frontier calls for it. It will
reduce the severity of dips but will not by itself produce steady, level flight at 1.5m.

## Options to actually get closer to a stable/level cruise altitude (design choice, not applied here)

Pick based on how much you're willing to trade off 3D exploration coverage for altitude stability — these are
listed from least to most restrictive:

1. **Narrow both ends of the frontier Z-sampling band** (recommended starting point). Add a symmetric
   `frontier/max_candidate_z` alongside today's `min_candidate_z`, clamping candidate altitude to e.g.
   [1.2, 1.7] camera_init instead of only flooring it. Cheap, localized change in the same function touched
   today. Tradeoff: frontiers well above/below that band become harder to fully cover from a viewpoint at their
   own height (coverage/visibility checks still run at the clamped altitude, so very tall/short frontier regions
   may see reduced `visib_num` and get rejected more often — acceptable if the arena's vertical extent is modest).
2. **Tighten the B-spline optimizer's Z corridor directly** (`bspline_opt`), rather than only constraining
   candidate viewpoints — this would also smooth out the transient Z excursions seen mid-trajectory (like the
   dip documented in `mapping_stall_crash_root_cause.md`) instead of only fixing the endpoints. More invasive:
   touches the trajectory optimization cost terms rather than just candidate generation.
3. **Add a soft altitude-hold bias term** to the exploration cost function (planner_manager/exploration_manager)
   that penalizes deviation from a preferred cruise Z when selecting between otherwise-similar-cost frontier
   targets. Most faithful to "prefer 1.5m, but still explore in 3D when needed" — also the most implementation
   effort, since it changes the TSP/cost-matrix scoring FUEL uses to pick between candidate viewpoints.
4. **Hard-clamp Z in the Flight Envelope Guard to a narrow band** (e.g. [1.4, 1.6]) instead of the current safety
   envelope ([0.5, 1.8]). Simplest to implement (config-only), but most aggressive: this would reject any FUEL
   command outside a 20cm band, likely causing frequent REJECT/hold-position events and probably stalling
   exploration in any region whose frontiers sit outside that band — not recommended as the primary fix, only as
   a last-resort safety cap if the softer options above prove insufficient.

**Recommendation**: start with option 1 (symmetric Z band on candidate sampling) since it directly targets the
mechanism causing the oscillation, is a small and easily-verified change, and preserves FUEL's ability to
actually finish mapping frontiers outside typical human-height range if the arena has any. Reserve option 4 for
containment only, not as the primary lever, since it acts after the fact on symptoms rather than on why FUEL
picked a low/high target in the first place.

## Implemented (2026-08-17)

Went with option 1. Added `frontier/max_candidate_z` alongside the `min_candidate_z` floor added earlier the same
day, both enforced in `FrontierFinder::sampleViewpoints()` (`active_perception/src/frontier_finder.cpp`):

```cpp
if (sample_pos.z() < min_candidate_z_)
  sample_pos.z() = min_candidate_z_;
else if (sample_pos.z() > max_candidate_z_)
  sample_pos.z() = max_candidate_z_;
```

Configured in `exploration_manager/launch/algorithm.xml`:
- `frontier/min_candidate_z = 1.2` (camera_init; ~1.3m world)
- `frontier/max_candidate_z = 1.6` (camera_init; ~1.7m world — kept 0.1m below the guard's 1.8m
  effective ceiling so ordinary trajectory overshoot around the clamped viewpoint doesn't trigger boundary
  rejects)

This replaces the earlier same-day `min_candidate_z = 0.9` floor-only value with the full band. Verified via
`catkin build active_perception exploration_manager` — succeeds cleanly, only pre-existing unrelated warnings.

Not yet re-verified live against a real test run at time of writing — see the next test pass for confirmation
that cruise altitude now stays inside [1.3, 1.7]m world instead of the 0.9-1.76m swing seen in the last live run.
