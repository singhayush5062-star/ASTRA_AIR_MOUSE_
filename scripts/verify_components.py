#!/usr/bin/env python3
"""Verify every payload component on the parked vehicle, and the LiDAR calibration.

Run against scripts/spawn_vehicle_only.sh (bare Gazebo, no PX4). Checks the things the
Gazebo GUI cannot show you, each of which has silently broken a flight at least once:

  1. prop clearance   analytic, from the real x500 prop collision boxes AND visual meshes
  2. rest pose        settled height and attitude (PX4 refuses to arm past ~10 deg)
  3. TFmini           beam DIRECTION, not just the number - a sideways beam still reads
  4. camera           optical axis direction, which no GUI widget displays
  5. LiDAR            live returns, and whether any of them come off the vehicle itself
  6. calibration      FAST-LIO extrinsic_T/R vs the SDF pose it is supposed to describe

Usage:  python3 scripts/verify_components.py
"""
import math
import os
import re
import sys

import rospy
import yaml
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import Image, PointCloud2, Range
import sensor_msgs.point_cloud2 as pc2

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
SDF = os.path.join(REPO, 'simulation/PX4-Autopilot-v1.14.3/Tools/simulation/gazebo-classic/'
                         'sitl_gazebo-classic/models/x500_vlp16/x500_vlp16.sdf')
FASTLIO = os.path.join(REPO, 'config/fast_lio/nidar_sim.yaml')

# x500 prop geometry, read off x500.sdf / the prop meshes
HUB_R = math.hypot(0.174, 0.174)
COLL_TIP = HUB_R + 0.2792307692307692 / 2.0
COLL_TOP = 0.06 + 0.0008461538461538462 / 2.0
VIS_TIP, VIS_TOP = 0.3637, 0.0694
TAN15 = math.tan(math.radians(15.0))

results = []


def check(name, ok, detail):
    results.append((name, ok, detail))
    print("  %-22s %s  %s" % (name, "PASS" if ok else "FAIL", detail))


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def quat_rpy(q):
    r = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    p = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    return r, p, y


def sdf_pose(text, anchor):
    m = re.search(re.escape(anchor) + r'.*?<pose>([-0-9.eE ]+)</pose>', text, re.S)
    return [float(v) for v in m.group(1).split()] if m else None


def main():
    text = open(SDF).read()
    lidar_pose = sdf_pose(text, '<link name="velodyne_link">')
    cam_link = sdf_pose(text, '<link name="camera_link">')
    cam_sens = sdf_pose(text, '<sensor name="camera" type="camera">')
    radius = float(re.search(r'velodyne_visual.*?<radius>([0-9.]+)</radius>', text, re.S).group(1))

    print("=" * 78)
    print("1. PROP CLEARANCE  (analytic, no simulation needed)")
    print("=" * 78)
    zl = lidar_pose[2]
    m_coll = zl - COLL_TIP * TAN15 - COLL_TOP
    m_vis = zl - VIS_TIP * TAN15 - VIS_TOP
    print("  lidar z %.4f, visual radius %.4f" % (zl, radius))
    print("  -15 deg ray at collision tip r=%.4f : %+.1f mm over the blade" % (COLL_TIP, m_coll * 1e3))
    print("  -15 deg ray at visual    tip r=%.4f : %+.1f mm over the blade" % (VIS_TIP, m_vis * 1e3))
    check("prop clearance", m_coll > 0.005 and m_vis > 0.005,
          "min margin %+.1f mm (need > 0)" % (min(m_coll, m_vis) * 1e3))

    # The standoff mast must actually reach from the centre plate to the sensor. Both ends,
    # because a mast that is merely the right LENGTH but centred wrong floats just as badly.
    PLATE_TOP, BODY_LEN = 0.032, 0.0717
    mast_z = sdf_pose(text, '<visual name="velodyne_mast_visual">')
    mast_len = float(re.search(r'velodyne_mast_visual.*?<length>([0-9.]+)</length>',
                               text, re.S).group(1))
    mast_lo = zl + mast_z[2] - mast_len / 2.0
    mast_hi = zl + mast_z[2] + mast_len / 2.0
    print("  mast spans body z %.4f .. %.4f  (plate top %.4f, sensor bottom %.4f)"
          % (mast_lo, mast_hi, PLATE_TOP, zl - BODY_LEN / 2.0))
    check("mast support", abs(mast_lo - PLATE_TOP) < 1e-3
          and abs(mast_hi - (zl - BODY_LEN / 2.0)) < 1e-3,
          "foots on the plate and meets the sensor, no gap at either end")

    rospy.init_node('verify_components', anonymous=True, disable_signals=True)
    print("\n" + "=" * 78)
    print("2-5. LIVE CHECKS  (waiting for Gazebo topics)")
    print("=" * 78)
    try:
        ls = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout=30)
    except Exception as exc:
        print("  no /gazebo/link_states (%s)" % exc)
        print("  start it first:  scripts/spawn_vehicle_only.sh 0.30 false x500_vlp16")
        return 1
    # let it settle onto the pad
    rospy.sleep(4.0)
    ls = rospy.wait_for_message('/gazebo/link_states', LinkStates, timeout=10)
    pose = dict(zip(ls.name, ls.pose))
    base = next((p for n, p in pose.items() if n.endswith('::base_link')), None)
    br, bp, by = quat_rpy(base.orientation)
    tilt = math.degrees(math.hypot(br, bp))
    cfg = yaml.safe_load(open(os.path.join(REPO,
          'catkin_ws/src/nidar_config/config/mission_config.yaml')))['nidar']
    pad_top = cfg['launch_pad']['thickness']
    rest = base.position.z - pad_top
    print("  base_link z %.4f  rest above pad %.4f  roll %+.3f deg  pitch %+.3f deg"
          % (base.position.z, rest, math.degrees(br), math.degrees(bp)))
    check("rest attitude", tilt < 2.0, "tilt %.2f deg (PX4 arms below ~10)" % tilt)
    check("belly clearance", abs(rest - cfg['vehicle']['belly_clearance']) < 0.02,
          "measured %.4f vs configured %.4f" % (rest, cfg['vehicle']['belly_clearance']))

    # --- TFmini: value AND direction ---
    rng = rospy.wait_for_message('/tfmini/range', Range, timeout=15)
    tf_link = next((p for n, p in pose.items() if 'tfmini' in n), None)
    tr, tp, ty = quat_rpy(tf_link.orientation)
    # the beam is the sensor's +X after the sensor's own pose; recompose it here
    sens = sdf_pose(text.split('model://tfmini_lidar')[0] +
                    open(os.path.join(REPO, 'simulation/custom_models/tfmini_lidar/model.sdf')).read(),
                    '<sensor name="tfmini_lidar"') or [0, 0, 0, 0, 0, 0]
    R = mul(rpy_to_mat(tr, tp, ty), rpy_to_mat(*sens[3:6]))
    beam = [R[0][0], R[1][0], R[2][0]]
    el = math.degrees(math.asin(max(-1.0, min(1.0, beam[2]))))
    expect = base.position.z - tf_link.position.z + (tf_link.position.z - pad_top)
    print("  TFmini beam world vector (%+.4f %+.4f %+.4f)  elevation %+.2f deg"
          % (beam[0], beam[1], beam[2], el))
    check("tfmini direction", el < -88.0, "%.2f deg (must be -90 = straight down)" % el)
    check("tfmini range", 0.10 < rng.range < 0.30,
          "%.4f m, sensor floor %.2f, margin %+.0f mm" % (rng.range, rng.min_range,
                                                          (rng.range - rng.min_range) * 1e3))

    # --- camera: optical axis ---
    Rl = rpy_to_mat(*cam_link[3:6])
    Rc = mul(Rl, rpy_to_mat(*cam_sens[3:6]))
    ax = [Rc[0][0], Rc[1][0], Rc[2][0]]
    cel = math.degrees(math.asin(max(-1.0, min(1.0, ax[2]))))
    caz = math.degrees(math.atan2(ax[1], ax[0]))
    print("  camera optical axis (%+.4f %+.4f %+.4f)  elevation %+.2f  azimuth %+.2f"
          % (ax[0], ax[1], ax[2], cel, caz))
    check("camera aim", abs(cel + 15.0) < 1.0 and abs(caz) < 2.0,
          "want elevation -15 (down), azimuth 0 (forward)")
    try:
        img = rospy.wait_for_message('/camera/image_raw', Image, timeout=15)
        check("camera stream", img.width == cfg['camera']['width'],
              "%dx%d %s" % (img.width, img.height, img.encoding))
    except Exception as exc:
        check("camera stream", False, str(exc))

    # --- LiDAR: returns, fields, and self-hits ---
    pc = rospy.wait_for_message('/velodyne_points', PointCloud2, timeout=20)
    fields = [f.name for f in pc.fields]
    pts = list(pc2.read_points(pc, field_names=('x', 'y', 'z'), skip_nans=True))
    n = len(pts)
    # anything returning from inside the prop annulus, in the lidar's own frame
    self_hits = 0
    nearest = 1e9
    for x, y, z in pts:
        r = math.hypot(x, y)
        nearest = min(nearest, math.sqrt(x * x + y * y + z * z))
        if 0.09 < r < 0.42 and -(zl - 0.04) > z > -(zl + 0.02):
            self_hits += 1
    farthest = 0.0
    for x, y, z in pts:
        farthest = max(farthest, math.sqrt(x * x + y * y + z * z))
    smin = float(re.search(r'velodyne-VLP16.*?<range>\s*<min>([0-9.]+)</min>',
                           text, re.S).group(1))
    smax = float(re.search(r'velodyne-VLP16.*?<range>.*?<max>([0-9.]+)</max>',
                           text, re.S).group(1))
    print("  /velodyne_points  %d points, fields %s" % (n, ','.join(fields)))
    print("  returns span %.3f .. %.3f m; sensor window %.2f .. %.1f m"
          % (nearest, farthest, smin, smax))
    check("range window", abs(smin - cfg['lidar']['min_range']) < 1e-6
          and abs(smax - cfg['lidar']['max_range']) < 1e-6,
          "SDF %.2f..%.1f vs config %.2f..%.1f"
          % (smin, smax, cfg['lidar']['min_range'], cfg['lidar']['max_range']))
    check("max range honoured", farthest <= smax + 0.05,
          "farthest return %.3f m, cap %.1f m" % (farthest, smax))
    check("lidar returns", n > 5000, "%d points per scan" % n)
    check("no prop self-hits", self_hits == 0,
          "%d returns inside the prop annulus" % self_hits)

    print("\n" + "=" * 78)
    print("6. LIDAR CALIBRATION  (FAST-LIO extrinsic vs the SDF it describes)")
    print("=" * 78)
    fl = yaml.safe_load(open(FASTLIO))
    T = fl['mapping']['extrinsic_T']
    Rm = fl['mapping']['extrinsic_R']
    sdf_T = lidar_pose[:3]
    dT = max(abs(a - b) for a, b in zip(T, sdf_T))
    print("  SDF velodyne_link pose  [%.4f, %.4f, %.4f]  rpy %.4f %.4f %.4f"
          % tuple(lidar_pose))
    print("  FAST-LIO extrinsic_T    %s" % T)
    print("  FAST-LIO extrinsic_R    %s" % Rm)
    check("extrinsic_T", dT < 1e-3, "max component error %.4f m" % dT)
    ident = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rot_ok = max(abs(a - b) for a, b in zip(Rm, ident)) < 1e-6
    check("extrinsic_R", rot_ok and max(abs(v) for v in lidar_pose[3:6]) < 1e-6,
          "identity, and velodyne_link carries no rotation")
    # cross-check against Gazebo's own transform rather than trusting the SDF text
    lid = next((p for nm, p in pose.items() if nm.endswith('velodyne_link')), None)
    if lid:
        meas = lid.position.z - base.position.z
        check("extrinsic vs Gazebo", abs(meas - T[2]) < 2e-3,
              "Gazebo reports the lidar %.4f m above base_link" % meas)
    check("blind vs geometry", fl['preprocess']['blind'] < nearest,
          "blind %.2f m, nearest real return %.3f m" % (fl['preprocess']['blind'], nearest))
    check("scan_line", fl['preprocess']['scan_line'] == cfg['lidar']['beams'],
          "%d rings" % fl['preprocess']['scan_line'])

    print("\n" + "=" * 78)
    bad = [n for n, ok, _ in results if not ok]
    print("SUMMARY: %d/%d checks passed" % (len(results) - len(bad), len(results)))
    if bad:
        print("FAILED: %s" % ', '.join(bad))
    print("=" * 78)
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
