# FUEL Exploration Parameter Reference & Doorway-Exit Bug Analysis

Source of truth for every value below: `catkin_ws/src/fuel/fuel_planner/exploration_manager/launch/algorithm.xml`
(the file actually loaded by `nidar_fuel_upstream.launch` → `algorithm.xml` include). Code paths verified against
`active_perception/src/frontier_finder.cpp`, `active_perception/src/perception_utils.cpp`, and
`plan_env/src/sdf_map.cpp`. Arena corridor width (**1.0 m**) taken from
`PLANNING_DOCS/next_phase_work.md` and `PLANNING_DOCS/Phase_4_completion.md`.

No code or config was changed while producing this document.

---

## 1. The bug, in one sentence

**`frontier/candidate_rmin = 0.8` is almost as large as the 1.0 m corridor itself**, so once the drone is inside
a doorway/corridor, the planner cannot find *any* legal viewpoint on the far side of the second door — the
frontier there goes dormant and is never visited, even though the door is physically identical to the one the
drone just came through.

## 2. Why increasing `max_ray_length` / `max_dist` didn't help

These two params only affect **how far the map is built**, not **which viewpoints are considered useful for
exploration**. They live in different pipelines:

| Param | Pipeline | Effect |
|---|---|---|
| `sdf_map/max_ray_length` | Occupancy mapping (`sdf_map.cpp:286-293`) | Clamps how far a LiDAR ray is fused into the occupancy grid. Extends map completeness in open areas. |
| `perception_utils/max_dist` | Viewpoint scoring (`perception_utils.cpp:88`, used by `FrontierFinder::countVisibleCells` and `insideFOV`) | Hard cutoff for whether a frontier cell counts as "seen" by a candidate viewpoint. |

`countVisibleCells()` (`frontier_finder.cpp:755`) calls `percep_utils_->insideFOV(cell)` first — and that check
is bounded by `max_dist_`, **not** `max_ray_length_`. So raising `sdf_map/max_ray_length` from 4.5→6.5 only makes
the occupancy grid extend a bit further into rooms the drone can already see into; it does nothing for viewpoint
*selection*, which is the actual bottleneck in narrow corridors. That's why you saw no improvement — you tuned
the wrong stage of the pipeline for this symptom.

## 3. The doorway-exit failure mechanism, step by step

1. Drone enters Room A through Door 1, arena corridors/doorways are **1.0 m wide** (`next_phase_work.md`).
2. `sdf_map/obstacles_inflation = 0.199` inflates every wall/door-frame voxel by ~0.2 m on each side.
   Usable clear width through any 1 m doorway ≈ `1.0 − 2×0.199 = 0.60 m`, so the free centerline corridor is only
   ±0.30 m wide.
3. Room A's far wall (containing Door 2) becomes a **frontier** — a cluster of "known-free adjacent to unknown"
   voxels (`frontier_finder.cpp:113`, `expandFrontier`).
4. `sampleViewpoints()` (`frontier_finder.cpp:668`) tries to place a viewpoint **around Door 2's frontier**, on
   concentric rings starting at `candidate_rmin = 0.8` out to `candidate_rmax = 2.6`, at 15° angular steps
   (`candidate_dphi`).
5. Every ring point is checked against 3 rejection gates, **in order**:
   - `getInflateOccupancy(sample) == 1` → **collision** with inflated wall (`n_coll` counter)
   - `isNearUnknown(sample)` → candidate sits within `min_candidate_clearance = 0.15 m` of any still-unmapped
     voxel (`n_clearance` counter)
   - `visib_num <= min_visib_num (10)` → not enough frontier cells visible/unoccluded from that pose
     (`n_low_vis` counter)
6. With only ±0.30 m of clear width on either side of the doorway centerline, **any candidate sampled at
   `rc = 0.8 m` that isn't almost exactly along the corridor's long axis lands inside the inflated wall** — gate
   (a) kills it. The handful of candidates that do land in the corridor itself are usually still `isNearUnknown`
   because the space *beyond* Door 2 hasn't been mapped yet — gate (b) kills those too. This is a chicken-and-egg
   trap: you need a viewpoint past the door to map beyond it, but you need what's beyond it mapped to place a
   valid viewpoint there.
7. Result: `sampleViewpoints()` returns zero viewpoints for Door 2's frontier → it's pushed to
   `dormant_frontiers_` (`frontier_finder.cpp:410`) instead of `frontiers_`. The TSP/planning stage
   (`fast_exploration_manager.cpp`) only ever sees `frontiers_`, so **Door 2 is invisible to the planner** even
   though it's right there in the map.
8. A dormant frontier is only reconsidered if new map data changes its cells (`isFrontierChanged`,
   `frontier_finder.cpp:365`) — but nothing is driving the drone to generate that new data, since no viewpoint
   exists to send it there. This is a stable deadlock, not a transient delay — matches what you're seeing
   ("sometimes" because the exact drone heading/position when the frontier is first detected determines whether
   any of the 24 angular samples on the `rc=0.8` ring happens to fall in the ~0.6 m surviving gap).

Contributing history: `candidate_rmin` was `1.5` two commits ago, then `0.6`, and is now `0.8` — it has never
been chosen with the 1 m corridor width as an explicit constraint, it's been tuned by feel across commits.

---

## 4. Full parameter reference

### 4.1 Frontier / viewpoint sampling (`active_perception/frontier_finder.cpp`) — most relevant to this bug

| Parameter | Current | Role | Recommendation |
|---|---|---|---|
| `frontier/candidate_rmin` | **0.8** | Inner radius (m) of the ring on which candidate viewpoints are sampled around a frontier's average position. **The direct cause of the bug** — must be smaller than half the usable corridor width. | **Decrease.** With 0.60 m usable width, half-width is 0.30 m. Set `candidate_rmin` to **0.35–0.45 m** so at least some ring points at 15° increments survive the collision gate inside a 1 m doorway. |
| `frontier/candidate_rmax` | 2.6 | Outer sampling radius (m). Together with `candidate_rmin` and `candidate_rnum` defines the ring spacing (`dr = (rmax−rmin)/rnum`). | Leave, or lower slightly to ~2.0 once `rmin` drops (keeps ring spacing tight — see below). |
| `frontier/candidate_rnum` | 3 | Number of rings between rmin and rmax (4 radii sampled total: rmin, rmin+dr, ... rmax). | **Increase to 4–5** once `rmin` is lowered, so there are more radii close to the corridor centerline instead of jumping straight to open-room distances. |
| `frontier/candidate_dphi` | 15° | Angular step between sampled candidates on each ring. | Consider **decreasing to 10°** for corridors — more angular samples increases the odds one lands in the narrow surviving gap. Doubles/triples candidate count, minor CPU cost. |
| `frontier/min_candidate_clearance` | 0.15 | Radius (m) around a candidate that must be **fully known** (not unknown) for the candidate to be accepted (`isNearUnknown`). Directly fights against `candidate_rmin` in tight, freshly-discovered spaces — see step 6 above. | **Decrease to ~0.10 m** (i.e. 1 voxel at 0.1 m resolution) so candidates just past a freshly-seen doorway aren't auto-rejected for touching unknown space. Don't go to 0 — you'll get viewpoints with no safety buffer from the unknown/obstacle boundary. |
| `frontier/min_candidate_dist` | 0.5 | Minimum distance (m) from the *current drone position* for a viewpoint to be preferred in `getTopViewpointsInfo`/`getViewpointsInfo` (not part of frontier sampling itself). | Leave as-is; not implicated in this bug. |
| `frontier/min_visib_num` | 10 | Minimum number of frontier cells that must be visible & unoccluded from a candidate for it to be accepted as a viewpoint. | Leave, or drop to ~6–8 only if doorway viewpoints still get rejected after fixing `rmin`/clearance — a narrow doorway naturally limits line-of-sight cell count. |
| `frontier/min_view_finish_fraction` | 0.6 | Fraction of a frontier's cells that must become non-frontier before `isFrontierCovered()` (`frontier_finder.cpp:719`) considers it "done" and triggers replanning. | Leave. Governs replan cadence, not reachability. |
| `frontier/cluster_min` | 30 | Minimum voxel count for a region-grown group of frontier cells to be registered as a `Frontier` at all (`expandFrontier`, `frontier_finder.cpp:157`). Below this, the seed is discarded silently. | Leave — 30 voxels at 0.1 m resolution is a small, reasonable cluster; not the bottleneck here. Lowering it further would create noisy micro-frontiers. |
| `frontier/cluster_size_xy` / `cluster_size_z` | 2.0 / 10.0 | Max cluster extent (m) before `splitLargeFrontiers` divides it via PCA (`frontier_finder.cpp:179`). | Leave — arena-scale, not doorway-scale. |
| `frontier/down_sample` | 3 | Voxel-grid downsample factor applied to frontier cells before yaw/visibility computation (`downsample()`, uses `resolution × down_sample` as leaf size = 0.3 m). | Leave. |

### 4.2 Perception / visibility (`active_perception/perception_utils.cpp`)

| Parameter | Current | Role | Recommendation |
|---|---|---|---|
| `perception_utils/max_dist` | 4.5 | Hard cutoff (m) for whether a frontier cell counts as visible from a candidate viewpoint (`insideFOV`, `perception_utils.cpp:88`). This is the real "how far can a viewpoint see for planning purposes" knob — not `sdf_map/max_ray_length`. | Leave at 4.5 for corridor work — a doorway problem is a *reachability* problem, not a *range* problem. Only raise this together with `sdf_map/max_ray_length` if you specifically want long, open-hall frontiers detected earlier. |
| `perception_utils/is_lidar` | true | Switches `insideFOV` to 360° horizontal coverage gated only by `top_angle` elevation limit, instead of a camera-style 4-plane frustum. Correct for the VLP-16. | Leave. |
| `perception_utils/top_angle` | 0.56125 rad (~32°) | Vertical half-FOV for the LiDAR visibility cone. | Leave. |
| `perception_utils/left_angle` / `right_angle` | 3.14159 (unused when `is_lidar=true`, since 360° coverage bypasses the plane checks) | Legacy camera-FOV params, dead when LiDAR mode is on. | Leave — harmless dead params in this mode. |
| `perception_utils/vis_dist` | 1.0 | Only used for FOV **visualization** geometry (RViz markers), not planning logic. | Leave, cosmetic only. |

### 4.3 Occupancy mapping (`plan_env/sdf_map.cpp`)

| Parameter | Current | Role | Recommendation |
|---|---|---|---|
| `sdf_map/obstacles_inflation` | 0.199 | Inflates every occupied voxel by this radius (m) to build `occupancy_buffer_inflate_`, which is what all collision checks (frontier sampling, A*, B-spline) test against (`sdf_map.cpp:434`, `inf_step = ceil(inflation/resolution)`). Directly eats into the 1 m doorway — see §3 step 2. | This is a genuine physical safety margin (~half the drone's footprint) and shouldn't be cut just to make doorways easier — that risks wall clips. **Don't change it as the primary fix**; fix `candidate_rmin` instead. If doorway traversal is still marginal after that, consider 0.15 instead of 0.199 only if the drone's actual airframe radius allows it. |
| `sdf_map/max_ray_length` | 6.5 (was 4.5) | Caps raycast fusion distance during occupancy updates (`sdf_map.cpp:286-293`). Governs map completeness range, unrelated to viewpoint selection (§2). | Fine to leave at 6.5 for general mapping completeness; it isn't hurting anything, it just isn't the fix for this bug either. |
| `sdf_map/min_ray_length` | 0.5 | Near-clip for raycast fusion — points closer than this are pulled to this distance before fusion. | Leave — prevents self-occlusion noise near the LiDAR body. |
| `sdf_map/local_bound_inflate` | 0.5 | Inflates the "local update box" used for incremental ESDF/ inflate updates (`sdf_map.cpp:319`). | Leave — performance/consistency knob, not related to doorway logic. |
| `sdf_map/local_map_margin` | 50 (voxels) | Margin used when computing local bounding box for map updates. | Leave. |
| `sdf_map/resolution` | 0.1 | Voxel size (m). Every distance-based param above (`min_candidate_clearance`, `obstacles_inflation`) is implicitly quantized to multiples of this. | Leave — 0.1 m is fine resolution for a 1 m corridor; don't coarsen it. |
| `sdf_map/p_hit` / `p_miss` / `p_min` / `p_max` / `p_occ` | 0.65/0.35/0.12/0.90/0.80 | Log-odds occupancy fusion probabilities. Controls how many consistent hits/misses are needed before a voxel flips occupied/free. | Leave — standard values, not implicated in doorway deadlock. |

### 4.4 Trajectory / safety-adjacent (context, not primary suspects)

| Parameter | Current | Role | Note |
|---|---|---|---|
| `optimization/dist0` | 0.7 | B-spline optimizer repulsive-cost threshold (m) — points closer than this to an obstacle accrue quadratic penalty cost (`bspline_optimizer.cpp:301`). | In a corridor with only ~0.30 m clearance to each wall, **every** trajectory point is inside this penalty zone on both sides simultaneously — this doesn't block a solution but does add constant optimization pressure toward the centerline. Not a hard blocker like `candidate_rmin`, but a contributing "corridor is uncomfortable" factor worth knowing if trajectories look overly hesitant even after frontiers are found. |
| `manager/clearance_threshold` | 0.2 | Passed to `planner_manager.cpp` as `pp_.clearance_`, a general safety clearance used in path feasibility checks. | Consistent with `obstacles_inflation`; leave. |
| `astar/margin` (kinodynamic path search) | not set in `algorithm.xml` (defaults to `-1.0` in `astar.cpp:130`) | Extra clearance margin during A* geometric search collision checks (`astar.cpp:80`). | Worth noting it's **unset** — falls back to a sentinel default. Not part of this bug but flagging since it's silently using a fallback rather than an explicit value. |

---

## 5. Recommended change set for the doorway-exit bug

In priority order, only touching `frontier/*` params (all in `exploration_manager/launch/algorithm.xml`):

1. `frontier/candidate_rmin`: `0.8` → **`0.4`** (primary fix — must be under half the usable corridor width of 0.30 m... realistically 0.4 still slightly exceeds it, but combined with more angular samples and rings it gives many more chances to land in the surviving gap; if doorway exits are still missed, try `0.3`).
2. `frontier/candidate_rnum`: `3` → **`5`** (denser rings between the new smaller rmin and rmax).
3. `frontier/candidate_dphi`: `15°` → **`10°`** (denser angular sampling per ring).
4. `frontier/min_candidate_clearance`: `0.15` → **`0.10`** (stop rejecting doorway candidates for touching freshly-seen unknown space).

Leave `sdf_map/obstacles_inflation`, `sdf_map/max_ray_length`, and `perception_utils/max_dist` alone — none of
them are the mechanism behind this specific failure, per the trace in §2–3.

This document makes no code or config changes — it's an analysis only, for you to apply deliberately and test
incrementally (change `candidate_rmin` alone first, re-test the same door, then layer in the others if it's still
not enough).
