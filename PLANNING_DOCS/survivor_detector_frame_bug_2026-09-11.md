# Survivor detector: reported positions fall outside the arena (open bug)

**Date discovered:** 2026-09-11
**Discovered during:** v3 sim run (`20260911_103244`) after the camera_link axis
fix landed and coverage / landing verification.
**Status:** open. Deliberately not fixed in this session so the session's
scope (landing + placement + Phase 5) could land clean.

## The symptom

Run `20260911_103244` (v3 after the axis fix) produced 98+ confirmed survivor
detections whose reported `position.x / .y` values span:

```
survivor_6 at world (2, 4)  — 5 detections at (0.59..3.65, 1.00..1.93) — err 2.5-3.3 m
survivor_1 at world (2,-4)  — 6 detections at (7.15..17.86 in x)      — err 7-16 m
survivor_2 at world (0,-2)  — 2 detections at (0.93..1.22, -0.86..+0.55) — err 1.7-2.7 m
several "OUTSIDE the 7x7 grid" warnings (cell 12,3 / 11,7 / 8,0 / 9,3)
```

All `cell=MISMATCH` against ground truth. Not a random error either — the
values pattern consistently, so the transform is wrong in a specific way, not
noisy.

## Not the axis conversion

That was fixed last session and `scripts/verify_camera_link_axes.py` still
passes. Reported positions are self-consistent (repeatable per survivor) —
they're just in the wrong frame.

## Very likely root cause

`survivor_detector.py:_backproject_ground()` transforms the ray from
`camera_link` to `map` via tf2 and intersects the z=0 plane. It then feeds the
resulting `(x, y)` into the grid math:

```python
grid_x = floor((position.x - origin_x) / cell_size)   # origin_x = -7.0
grid_y = floor((position.y - origin_y) / cell_size)   # origin_y = -7.0
```

The grid math assumes `map` is the *world* frame (bounded by ±7.5 m in this
arena). But `map` in this stack is planted at `world (pad.center.x,
pad.center.y, spawn_z)` and rotated 90 deg — see
`launch/nidar_fuel_upstream.launch`'s `world_to_map_tf`:

```
args="0 -9.5 0.26 1.5707963 0 0 world map"
```

That transform is `world -> map`, not `map -> world`. So `map` here IS the
camera_init frame (the odometry frame), *not* the world frame. Ray hits
resolve to `map` (== camera_init) coordinates but the grid math wants world.

The camera_init → world relationship for the standard spawn (yaw = 90 deg,
pad at (0, -9.5), spawn_z = 0.26) is:

```
world_x = -camera_init_y + pad.center.x
world_y =  camera_init_x + pad.center.y
```

That's exactly the conversion `flight_envelope_guard.py:244-246` already
carries, and the telemetry logger's `world=(...)` line uses. The detector
never applies it.

## Fix (one edit + one test)

1. In `_backproject_ground()` after computing `hit`, add:

   ```python
   # Convert map (== camera_init) coordinates to WORLD before applying the
   # grid math. The grid config is authored in world coordinates.
   spawn_x = rospy.get_param('/nidar/launch_pad/center/x', 0.0)
   spawn_y = rospy.get_param('/nidar/launch_pad/center/y', -9.5)
   world_x = -hit[1] + spawn_x
   world_y =  hit[0] + spawn_y
   world_z = hit[2]
   return np.array([world_x, world_y, world_z])
   ```

   Or, better, transform camera_link -> **world** in the tf2 lookup (needs a
   `map -> world` inverse or a direct `world` frame chained through the
   odometry). `map` and `world` are one static rotation apart, so tf2 can
   resolve either direction as long as both are published.

2. Extend `verify_fix_parity.sh` with a runtime check that ties a synthetic
   ground-truth survivor position to the grid math: seed a survivor at world
   (2, -4), simulate a detection at the drone's position, assert the reported
   `grid_x, grid_y` snaps to `(4, 1)`. Absent that assertion, the same class
   of bug reappears the next time the world/map static TF changes.

3. Update `verify_camera_link_axes.py` (or add `verify_detector_frame.py`)
   to also print the effective world-frame position for a hard-coded synthetic
   pixel + camera pose, so the axis convention AND the world-frame conversion
   are both eyeballable from one script.

## What is affected

- The `[SURVIVOR]` terminal line's `grid=(gx, gy)` and `pos=(x, y, z)` are
  wrong. `cell=MISMATCH` is not evidence the detector missed the target;
  it's evidence the frame conversion is wrong.
- `/survivors` (latched, consumed by `grid_visualizer.py`) publishes bad
  positions. Phase 5's grid tag markers will land on the wrong cells until
  this is fixed.
- The ground-truth accuracy printouts (`err=NN.NNm cell=MISMATCH`) are
  correct in that they *reflect* the reported values, but they misdiagnose
  the failure as "poor localisation" when the underlying detections are
  actually reasonable.

## What is NOT affected

- Landing (this session's other fix). The EDM's setpoints are already in
  camera_init and the world→map TF is used for visualisation and the flight
  envelope, not for closing the loop on position.
- Coverage reporting. `/sdf_map/coverage` fields are m² / % values, not
  positions.
- The Phase 5 2D map (`/map_2d`). Its OccupancyGrid header carries
  `frame_id: map` and the arena bounds it slices come from
  `/nidar/arenas/<active>/bounds`, which are world coordinates. The
  OccupancyGrid consumer applies the `world_to_map_tf` when rendering.
  This is only tangled-up when a grid cell's *label* needs to name a world
  location, which the map slicer does not do.
