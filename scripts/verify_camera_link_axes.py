#!/usr/bin/env python3
"""Verify the camera_link TF chain against survivor_detector.py's pixel->ray model.

Any edit to the x500_vlp16 SDF's <link name="camera_link"> or its child <sensor name="camera">
can silently break the survivor detector's back-projection. This script re-derives the
camera_link axes as they appear in base_link from the two SDF poses, and prints:

    * where each camera_link axis points (in base_link)
    * whether the look direction (+X) is the expected -15 deg tilt        (ASSERTED)
    * which way the raw image is oriented                                  (REPORTED)

camera_link is Gazebo's SENSOR frame. Gazebo renders pixels against it with its standard camera
model (optical axis +X, image right = -Y, image down = -Z) whatever roll the SDF gives the
sensor, and survivor_detector.pixel_ray_camera_link uses exactly that model. So the detector is
correct for any camera roll; what the roll changes is only whether the raw image is upright.
With the current SDF poses the sensor is rolled 180 deg, so the raw image is UPSIDE-DOWN
(verified 2026-10-02 by rotating a frame: the survivors are then upright). YOLO does as well on
it as on a de-rotated frame, so the detector does not rotate it.

(Until 2026-10-02 this script labelled +Y "image right" and +Z "image down" but asserted only
+X, so a wrong y/z convention in the detector passed this check. The labels below are the
Gazebo convention, and the pixel-ray unit tests in nidar_perception/test pin the mapping.)

Run without arguments; the SDF poses are hard-coded here on purpose so a divergence from the SDF
is caught by inspecting this file's constants against the SDF, not by a runtime tf lookup.

If your SDF poses differ from the constants below, update BOTH here and in
catkin_ws/src/nidar_qa/scripts/verify_fix_parity.sh (which is what CI greps).
"""
import math
import sys

import numpy as np


# From x500_vlp16.sdf's <link name="camera_link"><pose>...</pose></link>
# format: x y z roll pitch yaw
LINK_POSE_RPY = (3.1332, -0.0015, -3.0737)

# From x500_vlp16.sdf's <sensor name="camera"><pose>...</pose></sensor>,
# same RPY format, relative to the camera_link
SENSOR_POSE_RPY = (0.0, -0.259724, -3.0715)

# What survivor_detector.py assumes camera_link's +X points at, in base_link.
# For a camera tilted -15 deg down and pointing along drone forward:
EXPECTED_LOOK = (math.cos(math.radians(15.0)), 0.0, -math.sin(math.radians(15.0)))
EXPECTED_TOLERANCE = 1e-3


def rot_zyx(yaw, pitch, roll):
    """Return R = Rz(yaw) * Ry(pitch) * Rx(roll). Matches tf's RPY convention."""
    cz, sz = np.cos(yaw), np.sin(yaw)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cx, sx = np.cos(roll), np.sin(roll)
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return Rz @ Ry @ Rx


def main():
    R_link = rot_zyx(LINK_POSE_RPY[2], LINK_POSE_RPY[1], LINK_POSE_RPY[0])
    R_sensor = rot_zyx(SENSOR_POSE_RPY[2], SENSOR_POSE_RPY[1], SENSOR_POSE_RPY[0])
    R = R_link @ R_sensor

    print('camera_link derived axes (in base_link):')
    axes = {'+X (look direction)':      [1, 0, 0],
            '+Y (sensor left; right = -Y)': [0, 1, 0],
            '+Z (sensor up; down = -Z)':    [0, 0, 1]}
    for label, axis in axes.items():
        v = R @ np.array(axis)
        print('  %-22s -> (%+.4f, %+.4f, %+.4f)' % (label, v[0], v[1], v[2]))

    look = R @ np.array([1.0, 0.0, 0.0])
    err = math.sqrt(sum((look[i] - EXPECTED_LOOK[i]) ** 2 for i in range(3)))
    print()
    print('Expected look direction (from -15 deg tilt): (%.4f, %.4f, %.4f)'
          % EXPECTED_LOOK)
    print('Deviation from expected: %.6f' % err)

    if err > EXPECTED_TOLERANCE:
        print()
        print('FAIL: camera_link look direction differs from expected by more '
              'than %.4f. Either the SDF poses have drifted or the constants '
              'in this file are stale. Update BOTH the SDF and the constants '
              'above; the detector\'s axis swap depends on this convention.'
              % EXPECTED_TOLERANCE)
        return 1

    print()
    z_up = (R @ np.array([0.0, 0.0, 1.0]))[2]      # base_link +Z is up
    print('Image orientation: sensor +Z has an up-component of %+.3f in base_link -> the raw image '
          'is %s.' % (z_up, 'UPRIGHT' if z_up > 0 else 'UPSIDE-DOWN (expected with the current SDF)'))
    print()
    print('OK: look direction matches. The detector uses the sensor-frame camera model, which is '
          'correct for any camera roll.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
