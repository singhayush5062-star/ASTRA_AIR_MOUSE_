# Plan: Collision-Retreat Behavior for FUEL (root cause #5)

## The problem, precisely

`FastExplorationFSM::safetyCallback` (`exploration_manager/src/fast_exploration_fsm.cpp:358`) runs at 20Hz
while `EXEC_TRAJ`. It calls `FastPlannerManager::checkTrajCollision` (`plan_manage/src/planner_manager.cpp:96`),
which walks forward along the *currently executing* trajectory (up to 6m lookahead) checking each future point's
occupancy. If any point is occupied, it logs `collision at: <point>` and returns unsafe, which immediately
transitions the FSM to `PLAN_TRAJ`.

The replan that follows seeds its start state from the vehicle's current position (real odometry or the
predicted-trajectory/real-odometry fallback added in the previous root-cause pass), then calls
`callExplorationPlanner()` — the **full** exploration pipeline: frontier detection, TSP viewpoint selection, and
`kinodynamicReplan` toward whatever viewpoint that selection lands on. Nothing in this path treats "we're
currently touching/next-to an obstacle" as special. Since the map/frontiers haven't meaningfully changed and the
vehicle hasn't moved away from the obstacle, viewpoint selection typically **re-picks the same target**, and the
new plan threads through the same tight space and collides again.

Confirmed live twice now: 15-24+ seconds of `collision at: ...` / `Replan: collision detected` repeating at
essentially the same coordinates, dozens of cycles, at two different locations in two different sessions
(spawn-adjacent corner ~world(-6.2,-6.2), and ~world(-1.5,5.9) in the most recent CPU-pinning verification run).
Both times this preceded the vehicle coming to rest on the ground with no recovery.

## Design

Add a **collision-cluster detector** that only engages once a *repeated* failure at the same spot is confirmed
(not on the first collision — that case already replans successfully most of the time and shouldn't change
behavior). Once triggered, fly a short **retreat leg** away from the obstacle using the existing trajectory
machinery, then resume normal exploration from the new position.

### 1. Surface the collision point (currently discarded)

`checkTrajCollision` computes `fut_pt` (the first occupied point found) but only prints it — the caller never
gets it. Change the signature to return it:
```cpp
// plan_manage/include/plan_manage/planner_manager.h
bool checkTrajCollision(double& distance, Eigen::Vector3d& collision_pt);
```
One call site (`safetyCallback`) to update.

### 2. Track a rolling window of recent collisions

New fields on `FSMData` (`exploration_manager/include/exploration_manager/expl_data.h`):
```cpp
std::deque<std::pair<ros::Time, Eigen::Vector3d>> recent_collisions_;
bool retreating_ = false;
Eigen::Vector3d retreat_target_;
```
In `safetyCallback`, on every collision: push `(now, collision_pt)`, drop entries older than
`fsm/collision_cluster_window` (default 2.0s). If the window's size reaches `fsm/collision_cluster_thresh`
(default 4) and we're not already retreating, compute a retreat target (below) and set `retreating_ = true`
before transitioning to `PLAN_TRAJ`.

### 3. Compute the retreat target

Use the ESDF gradient at the collision point — `EDTEnvironment::evaluateEDTWithGrad` (`plan_env/edt_environment.h:42`)
returns the local distance-field gradient, which by construction points from occupied space toward free space:
```cpp
double dist; Eigen::Vector3d grad;
edt_environment_->evaluateEDTWithGrad(collision_pt, ros::Time::now().toSec(), dist, grad);
Eigen::Vector3d dir = grad.normalized();  // fallback below if grad.norm() is ~0
```
Fallback (very close to a flat surface, gradient can be near-degenerate): use the reverse of current velocity
(`-fd_->odom_vel_.normalized()`), or failing that, the direction from `collision_pt` back to `fd_->odom_pos_`.

```cpp
retreat_target_ = fd_->odom_pos_ + dir * retreat_distance;  // default retreat_distance = 0.8m
```
Clamp to the SDF map's box (`sdf_map_->isInBox`) and to the existing `min_candidate_z_`/`max_candidate_z_` band
so the retreat itself can't create a new boundary/altitude violation.

### 4. Branch the PLAN_TRAJ handler

```cpp
case PLAN_TRAJ: {
  // ... existing start-state seeding (unchanged) ...

  if (fd_->retreating_) {
    // Direct point-to-point replan to the retreat target -- deliberately skips
    // callExplorationPlanner()/frontier-TSP selection, since re-picking the same viewpoint is
    // exactly the failure being fixed here.
    bool ok = planner_manager_->kinodynamicReplan(
        fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->retreat_target_, Eigen::Vector3d::Zero());
    if (ok) {
      transitState(PUB_TRAJ, "FSM[retreat]");
    } else {
      // Retreat target itself unreachable (rare -- pinned on all sides) -- fall back to normal
      // replanning rather than looping forever trying to reach an unreachable point.
      fd_->retreating_ = false;
    }
    break;
  }

  // ... existing callExplorationPlanner() path (unchanged) ...
}
```
Clear `fd_->retreating_ = false` and `fd_->recent_collisions_.clear()` once the retreat trajectory completes
(natural `EXEC_TRAJ` → `PLAN_TRAJ` transition on duration expiry) so exploration resumes from the new position
with a clean slate. Add a ~3s hard timeout on `retreating_` as a safety net in case the retreat trajectory itself
faults immediately.

### 5. New tunables (`algorithm.xml`, new `fsm/` params)

| Param | Default | Purpose |
|---|---|---|
| `fsm/collision_cluster_window` | 2.0s | how far back to look when counting recent collisions |
| `fsm/collision_cluster_thresh` | 4 | collisions within the window before retreat triggers |
| `fsm/retreat_distance` | 0.8m | how far to back off along the ESDF gradient |

## Why this design specifically

- **Reuses `kinodynamicReplan`** for the retreat leg — no new trajectory-generation path, so it inherits the
  same dynamic-feasibility and B-spline smoothing guarantees as every other trajectory in the system.
- **Gated behind a cluster threshold**, not a single collision — preserves today's behavior for the common case
  (one collision warning, successful replan around it on the next try), only engages once a repeat failure at
  the same spot is confirmed, matching what was actually observed (dozens of cycles, 15+ seconds).
- **ESDF-gradient retreat direction** is principled — guaranteed to point toward locally freer space, unlike a
  blind "reverse of arrival heading," which could walk straight back into the same obstacle on an oblique
  approach.
- **Bypasses frontier/TSP selection** specifically for the retreat leg, since re-picking the same viewpoint is
  the exact mechanism causing the loop — that's the one part of the existing flow this plan changes, not the
  general replanning logic.

## Verification plan

1. `catkin build exploration_manager plan_manage` — confirm clean build, same bar as every prior fix this
   session.
2. Live re-test at both confirmed trouble spots (spawn-corner ~world(-6.2,-6.2), and the newer one at
   ~world(-1.5,5.9)) — watch for a new `fuel.log` line when the cluster threshold trips (e.g.
   `[FSM] Collision cluster detected (N in T s) -- retreating to (...)`), and confirm via the guard's CSV /
   `mission_telemetry` that world position visibly moves away from the collision coordinates instead of
   oscillating in place, with exploration resuming afterward.
3. Watch specifically for two regressions to rule out: (a) the cluster threshold firing on ordinary,
   already-recoverable single collisions (would show up as retreats happening far more often than the ~15s+
   clusters seen in the failure logs), and (b) the retreat leg itself clipping a *different* nearby obstacle
   (would show up as a second `collision at:` immediately during the retreat trajectory) — if that happens, the
   ESDF-gradient fallback logic needs a second look before relying on the primary gradient direction alone.

## Not addressed by this plan

- Why the vehicle ends up close enough to an obstacle to trigger a collision-cluster in the first place (e.g.
  whether `obstacles_inflation` is too generous/tight for this arena's corridor widths, or whether frontier
  viewpoint selection is choosing viewpoints too close to walls). This plan treats the recovery behavior, not
  the trigger — worth a follow-up look if retreat clusters turn out to be frequent even after this fix.
