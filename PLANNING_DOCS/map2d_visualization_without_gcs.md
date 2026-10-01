# Visualising the live 2D map without a GCS

**Added:** 2026-09-11
**Applies to:** Phase 5 output topics `/map_2d`, `/grid_markers`,
`/survivor_tags` published by `nidar_mission/scripts/map_2d_slicer.py` and
`grid_visualizer.py`.

The Phase 6 GCS (rosbridge + Foxglove) is not built yet. Until it is, the
same three topics can be viewed locally with RViz — no external network,
no browser panel, works offline, mirrors what the GCS will eventually show.

## Quick start

While `test_takeoff.sh` is running (any run — sim, GUI or headless), in
another terminal:

```
cd ~/NIDAR
./scripts/view_map2d.sh
```

That launches RViz with the top-down layout in [config/nidar_map2d.rviz](../config/nidar_map2d.rviz).
Fixed frame is `world`. Panels rendered:

| Layer            | Source topic          | What it shows                                    |
|------------------|-----------------------|--------------------------------------------------|
| Map (2D slice)   | `/map_2d`             | OccupancyGrid — walls at 100, floor at 0, unknown at -1 |
| Grid overlay     | `/grid_markers`       | 2 m competition grid lines + `A1..G7` cell labels |
| Survivor tags    | `/survivor_tags`      | red sphere + text tag at each confirmed survivor |
| World Grid       | (RViz built-in)       | 1 m reference grid so distances read at a glance |
| FAST-LIO cloud   | `/cloud_registered`   | optional (default off) — full 3D lidar for cross-check |

## Pre-flight check

If nothing shows up in RViz, run:

```
./scripts/view_map2d.sh --check
```

It prints the publish rate for each required topic and the `world → map`
TF. Common failures:

| Symptom                                           | Fix                                                       |
|---------------------------------------------------|-----------------------------------------------------------|
| `MISS /map_2d`                                    | `map_2d_slicer` isn't running — check `nidar_mission.launch` |
| `MISS /grid_markers`                              | `/arena_grid/*` rosparams not loaded — verify `arena_grid.yaml` |
| `/map_2d (no messages)` but topic exists          | `/sdf_map/occupancy_all` not publishing (FUEL not started yet) |
| RViz opens but view is empty                      | Fixed frame is wrong — check top-left panel says `world`  |
| Everything empty on a fresh sim                   | Wait ~5 s after takeoff — FUEL needs to see one lidar scan before publishing occupancy |

## Alternative one-off inspections

Commands that don't need RViz:

```
# Print grid cell IDs the map slicer is emitting into
rostopic echo /grid_markers/markers[0]/pose/position -n 1

# Check the /map_2d publish rate
rostopic hz /map_2d

# Dump a single OccupancyGrid to a .png (needs `pip install pypng`)
rostopic echo -n 1 /map_2d > /tmp/map.yaml

# Record everything relevant for offline replay
rosbag record /map_2d /grid_markers /survivor_tags /tf /tf_static /survivors -O /tmp/map2d.bag
# ... later
rosbag play /tmp/map2d.bag
./scripts/view_map2d.sh
```

## Why RViz specifically

- **Offline-safe.** Competition brief §7 disallows internet / cloud / public
  WiFi. RViz runs entirely on the operator laptop. Foxglove Studio would work
  too but adds a rosbridge_server dependency this repo does not yet ship.
- **Same topics the GCS will read.** When Phase 6 lands, the three topics
  named above will be the payload the GCS bridge forwards. Testing with
  RViz today catches display-side bugs (frame_id mismatch, marker `ns`/`id`
  collisions, stale latched publishes) that would only surface at the GCS
  boundary otherwise.
- **Top-down orthographic camera by default.** The 2D floorplan reads
  naturally — the operator sees a map, not a perspective 3D scene.

## What is deliberately NOT rendered

- **The drone's live 3D pose.** Would need a URDF/Robot Model display; adds
  visual noise on a floorplan view. Read `world → base_link` in RViz's TF
  panel if a pose check is needed.
- **The 3D voxel volume from FUEL.** Available on `/sdf_map/occupancy_all`
  but it's dense (~100k points) and rendering it saturates the same laptop
  the sim runs on. The 2D slice conveys the same "which cells are known"
  information at a fraction of the cost.
- **The camera image.** That's Phase 6 GCS work; RViz's image display can
  add `/camera/image_raw` if wanted, but the point-cloud slice already
  answers "is the drone stuck?" without it.
