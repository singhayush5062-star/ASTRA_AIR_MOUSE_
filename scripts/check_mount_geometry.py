#!/usr/bin/env python3
"""Check camera/TFmini/landing-gear mounting geometry from the SDF alone -- no Gazebo, no PX4,
no ROS. Parses the actual <pose> values out of iris_vlp16_cam.sdf and prints every clearance
that mattered on 2026-09-06 (camera/TFmini buried in the fuselage, then legs sinking into the
pad and blinding the rangefinder). Run this after every manual pose edit, before spawning
anything -- it runs in well under a second.

Usage:
    python3 scripts/check_mount_geometry.py
"""
import math
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SDF = os.path.join(REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
                    'gazebo-classic', 'sitl_gazebo-classic', 'models', 'iris_vlp16_cam',
                    'iris_vlp16_cam.sdf')

BASE_LINK_BOTTOM = -0.055  # iris base_link collision: 0.47 x 0.47 x 0.11 box, centred -> -0.055
RNG_FLOOR = 0.10           # TFmini's own minimum valid range (model.sdf <range><min>)
CAMERA_STL = os.path.join(REPO, 'simulation', 'PX4-Autopilot-v1.14.3', 'Tools', 'simulation',
                          'gazebo-classic', 'sitl_gazebo-classic', 'models', 'iris_vlp16_cam',
                          'meshes', 'siyi_a8_mini.stl')


def camera_z_extent(roll, pitch):
    """Rotated z-extent of the camera mesh in link-local coordinates.

    The camera mount carries a real roll+pitch (it was inverted on 2026-09-06 to swap which
    face meets the belly), and roll/pitch do NOT commute: a fixed axis-aligned bounding box
    from the unrotated STL is the wrong answer once roll != 0. This rotates every vertex with
    the actual SDF composition (v' = Ry(pitch) @ Rx(roll) @ v, roll applied first) and takes
    the true min/max, matching how Gazebo will actually place the mesh.
    """
    import struct
    f = open(CAMERA_STL, 'rb')
    f.read(80)
    n = struct.unpack('<I', f.read(4))[0]
    data = f.read(n * 50)
    verts = []
    for i in range(n):
        rec = data[i * 50:i * 50 + 50]
        for j in range(3):
            x, y, z = struct.unpack_from('<fff', rec, 12 + j * 12)
            verts.append((x, y, z))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    zmin, zmax = float('inf'), float('-inf')
    for x, y, z in verts:
        # Rx(roll)
        y1, z1 = y * cr - z * sr, y * sr + z * cr
        x1 = x
        # Ry(pitch)
        z2 = -x1 * sp + z1 * cp
        zmin = min(zmin, z2)
        zmax = max(zmax, z2)
    return zmin, zmax


def pose_of(text, anchor):
    m = re.search(re.escape(anchor) + r'.*?<pose>([-0-9. ]+)</pose>', text, re.S)
    if not m:
        return None
    return [float(v) for v in m.group(1).split()]


def all_leg_poses(text):
    return [[float(v) for v in m.group(1).split()]
            for m in re.finditer(r'<link name="leg_\d">\s*<pose>([-0-9. ]+)</pose>', text)]


def leg_lengths(text):
    return [float(m.group(1)) for m in
            re.finditer(r'leg_\d_collision.*?<length>([-0-9.]+)</length>', text, re.S)]


def main():
    if not os.path.exists(SDF):
        print("SDF not found: %s" % SDF)
        return 1
    text = open(SDF).read()

    cam = pose_of(text, '<link name="camera_link">')
    tfm = pose_of(text, '<uri>model://tfmini_lidar</uri>')
    legs = all_leg_poses(text)
    lens = leg_lengths(text)

    print("=" * 72)
    print("Mounting geometry, read live from:\n  %s" % os.path.relpath(SDF, REPO))
    print("=" * 72)

    if cam:
        cam_z, cam_roll, cam_pitch = cam[2], cam[3], cam[4]
        zmin, zmax = camera_z_extent(cam_roll, cam_pitch)
        cam_bottom = cam_z + zmin
        cam_top = cam_z + zmax
        print("\nCAMERA  pose = %s" % cam)
        print("  mesh rotated (roll=%.4f, pitch=%.4f) spans link-local z %.4f .. %.4f"
              % (cam_roll, cam_pitch, zmin, zmax))
        print("  -> base_link-frame:  bottom %+.4f   top %+.4f" % (cam_bottom, cam_top))
        if cam_top > BASE_LINK_BOTTOM + 0.005:
            print("  !! top face is ABOVE the fuselage belly (%.4f) by %.4f m -- "
                  "the mesh is drawn INSIDE the airframe." % (BASE_LINK_BOTTOM,
                  cam_top - BASE_LINK_BOTTOM))
        else:
            print("  OK: sits %.4f m clear below the belly (%.4f)"
                  % (BASE_LINK_BOTTOM - cam_top, BASE_LINK_BOTTOM))
    else:
        cam_bottom = None
        print("\nCAMERA: could not find camera_link pose (file changed shape?)")

    if tfm:
        tfm_z = tfm[2]
        # collision box is 0.035 m tall, centred on the link
        tfm_bottom = tfm_z - 0.0175
        print("\nTFMINI  pose = %s" % tfm)
        print("  collision box 0.035 m, centred -> bottom %+.4f" % tfm_bottom)
        d_offset = -tfm_z  # distance below base_link, i.e. EKF2_RNG_POS_D should equal this
        print("  distance below base_link (what EKF2_RNG_POS_D should be) = %.4f" % d_offset)
    else:
        tfm_z = None
        print("\nTFMINI: could not find mount pose (file changed shape?)")

    if legs and lens:
        foot_depths = [pose[2] - length / 2.0 for pose, length in zip(legs, lens)]
        print("\nLEGS  (%d found)" % len(legs))
        for i, (pose, length, foot) in enumerate(zip(legs, lens, foot_depths)):
            print("  leg_%d  centre z=%+.4f  length=%.3f  -> foot at %+.4f"
                  % (i, pose[2], length, foot))
        foot = foot_depths[0]
        if any(abs(f - foot) > 1e-6 for f in foot_depths):
            print("  !! legs are not all the same depth -- check for a partial edit")

        print("\n  Rest height above whatever surface the feet touch = %.4f m" % (-foot))
        if tfm_z is not None:
            parked_range = (-foot) - (-tfm_z)
            margin_mm = (parked_range - RNG_FLOOR) * 1000
            verdict = "OK" if parked_range > RNG_FLOOR else "*** SENSOR BLIND, WILL CRASH ***"
            print("  TFmini range while parked  = rest_height - %.4f = %.4f m" % (-tfm_z,
                  parked_range))
            print("  margin above its %.2f m floor = %+.1f mm   %s" % (RNG_FLOOR, margin_mm,
                  verdict))
            if parked_range <= RNG_FLOOR + 0.02:
                print("  (measured 2026-09-06: contact sag alone ate ~39 mm off the nominal\n"
                      "   foot depth, so keep at least 30-40 mm of margin here, not just >0.)")
        if cam_bottom is not None:
            clearance = (-foot) - (-cam_bottom)
            print("  camera protected by legs: feet are %.4f m below the camera bottom"
                  % clearance)
            if clearance <= 0:
                print("  !! legs are SHORTER than the camera -- it touches down first")
    else:
        print("\nLEGS: none found (or lengths not parsed) -- landing gear not present?")

    print("\n" + "-" * 72)
    print("Belly clearance needed in mission_config.yaml (vehicle.belly_clearance):")
    if legs and lens:
        print("  >= %.4f m (the deepest point below base_link, i.e. the feet)" % (-foot))
    print("Run  python3 catkin_ws/src/nidar_config/scripts/apply_mission_config.py\n"
          "after changing belly_clearance, to push spawn_world_z out to every consumer.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
