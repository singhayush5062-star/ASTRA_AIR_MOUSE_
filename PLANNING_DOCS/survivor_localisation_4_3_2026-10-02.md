# Survivor 3D localisation (phase plan §4.3) — what was wrong, what was measured, what changed

**Date:** 2026-10-02 · **Package:** `nidar_perception` · **Status:** implemented, unit-tested, tested against Gazebo ground truth in a bare sim; flight result at the end.

## 1. Symptom

Tags published on `/survivors` landed in the wrong grid cell. Across the two flights of 2026-10-02 only **1 of 9** confirmed tags was in the cell of the survivor it was nearest to, with errors of 1–4.4 m, a consistent bias toward the arena centre, and some tags outside the arena. Phase plan §4 exit criterion is "each localised within one grid cell of ground truth" (cells are 2 m).

## 2. Method

A bare Gazebo (no PX4, FAST-LIO or TF chain) with `x500_vlp16`, gravity off. The vehicle is teleported to random poses 1.8–6 m from each survivor; for each pose the real camera frame goes through the real `best.pt`; Gazebo's `/gazebo/link_states` supplies the **true** camera pose. Detector variants are then scored against the survivors' true world positions. Scripts were throwaway; the recorded detections are committed as `nidar_perception/test/data/gazebo_detections.json` (37 real YOLO boxes).

## 3. Findings

1. **The pixel → ray mapping was wrong.** `camera_link` in this stack is Gazebo's sensor frame, and Gazebo renders pixels with the standard camera model (optical axis +X, image right = −Y, image down = −Z). The detector used `[oz, +ox, +oy]`. It agrees with the truth only at the image centre: median 37° error, up to 90°. With `[oz, −ox, −oy]` the angle to the truth ray is 0.000° (median).
2. **The raw camera image is upside-down.** The SDF composes a 180° roll into the sensor, so sensor +Z points down. Rotating a frame by 180° shows an upright, standing T-pose mannequin. Consequence for the old "bbox bottom = feet" idea: the bottom of the box is the **head**, and intersecting a head ray with the floor overshoots by metres (median error 5.0 m even with perfect poses).
   *(Correction: an earlier reading of the first images concluded the survivors lie flat and float above the floor. That was wrong; it came from looking at the upside-down frames.)*
3. **Aiming at the bbox centre on a body-height plane works.** On detections that pass the gates (n=18): median 0.39 m, max 0.69 m at `h = 0.35 m`; best of 0.26–0.45 m. 100 % within one cell.
4. **Border-truncated boxes are the outliers.** 10 of the 11 detections more than a cell off touched the image edge; the last had a "range" of 53 m (nearly horizontal ray). Gates: border margin 6 px, minimum side 20 px, range cap 8 m.
5. **The `/cloud_registered` raycast the doc suggests does not help here.** For 25 of 37 detections the lidar returned **no** point within 0.9 m of the survivor (a ±15° VLP-16 fan passes over a body this low until ~4.5 m); where it did, the points were mostly wall points, and the cloud centroid was worse than the plane estimate (median 0.91 m vs 0.59 m). A cloud line-of-sight check rejected nothing the border/range gates did not. Not implemented; can be revisited if the lidar mounting or the survivor model changes.
6. **YOLO does not need the image de-rotated.** Same 40 poses: ≥1 detection on 15 (raw) vs 14 (rotated) poses, similar confidence. The mapping in (1) is independent of the camera roll, so nothing needs rotating.
7. The check that should have caught this, `scripts/verify_camera_link_axes.py`, labelled +Y "image right" and +Z "image down" but only asserted +X.

## 4. Changes

| File | Change |
|---|---|
| `nidar_perception/scripts/survivor_detector.py` | `pixel_ray_camera_link` (corrected mapping), `intersect_plane`, `box_is_usable` as pure functions; `_localise` aims the bbox centre at `z = target_height_m`; border / size / range / out-of-arena gates; published tags keep refining (republish when the cell changes or every 5th observation); sim-only `[LOC-CHECK]` assertion line vs the Gazebo survivors every 20 s |
| `nidar_perception/launch/detector.launch`, `nidar_config/.../mission_config.yaml` | `target_height_m` 0.35, `border_margin_px` 6, `min_box_px` 20, `arena_margin_m` 0.3 |
| `nidar_perception/test/` | 16 unit tests incl. a ground-truth replay of the recorded detections and a test that fails if the old mapping returns. Run: `python3 -m unittest discover -s catkin_ws/src/nidar_perception/test -v` (sourced ROS env) |
| `scripts/verify_camera_link_axes.py` | states the true convention, reports image orientation |
| `nidar_qa/scripts/verify_fix_parity.sh` | pins the new functions and fails if the old mapping reappears |

`/survivors` keeps its contract: same message, same frame (`world`), `position.z = 0`.

## 5. Verification

* Unit tests: 16/16.
* Real detector node in the bare sim, fed a perfect TF from Gazebo truth, vehicle teleported around the survivors: **4/4 tags in the correct cell, median error 0.40 m**, 0 wrong-cell, 0 out-of-arena. (Only 4 of 6 survivors were tagged because the random poses never gave the other two three clean views; that is a viewing question, not a localisation one.)
* Full flight: see §6.

## 6. Limits to keep in mind

* `target_height_m = 0.35` was fitted on 18 detections of the sim's `sitting.dae` mannequins. A real dummy needs its own, measured value.
* Only about half of raw detections pass the border gate, so a survivor needs three *clean* views before it is confirmed.
* Coverage of survivors (≥ 5 of 6 detected) depends on the flight path and camera view and is not addressed here.
