#!/usr/bin/env python3
"""Score a recorded /map_2d against the real arena geometry (phase plan §5.1).

    catkin_ws/src/nidar_qa/scripts/map_accuracy.py logs/bags/map2d_<run>_0.bag [--png out.png]

Ground truth is the arena mesh the simulation loads (nidar_sim/models/<arena>/meshes/<arena>.stl,
placed with model.sdf's <pose> and <scale>), sliced at several heights in the flight band and
rasterised onto the map's own grid. Scores, all with a tolerance of --tol metres (default 0.10):

  wall precision   share of mapped wall cells that lie on a real wall
  wall recall      share of real wall cells (inside the explored area) that the map shows as wall
  false free       share of mapped free cells that lie on a real wall (should be ~0)
  explored         m^2 of floor mapped free

--png writes a picture: mapped walls white, mapped free dark grey, real walls the map missed red.
"""
import argparse
import os
import re
import struct

import cv2
import numpy as np
import rosbag
import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
SRC = os.path.join(ROOT, 'catkin_ws', 'src')


def load_stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:300]:
        v = [list(map(float, l.split()[1:4])) for l in d.decode().splitlines()
             if l.strip().startswith('vertex')]
        return np.array(v).reshape(-1, 3, 3)
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50],
                      dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return a['v'].astype(float)


def arena_triangles():
    cfg = yaml.safe_load(open(os.path.join(SRC, 'nidar_config', 'config', 'mission_config.yaml')))
    name = cfg['nidar']['arena']['active']
    model = os.path.join(SRC, 'nidar_sim', 'models', name)
    sdf = open(os.path.join(model, 'model.sdf')).read()
    pose = [float(v) for v in re.search(r'<pose>([^<]+)</pose>', sdf).group(1).split()[:3]]
    scale = [float(v) for v in re.search(r'<scale>([^<]+)</scale>', sdf).group(1).split()]
    tri = load_stl(os.path.join(model, 'meshes', name + '.stl')) * np.array(scale) + np.array(pose)
    return name, tri


def truth_raster(tri, x0, y0, w, h, res, heights):
    m = np.zeros((h, w), np.uint8)
    cell = lambda p: (int(np.floor((p[0] - x0) / res)), int(np.floor((p[1] - y0) / res)))
    for z in heights:
        for t in tri:
            zs = t[:, 2]
            if zs.min() > z or zs.max() < z:
                continue
            pts = [t[i] + (z - t[i][2]) / (t[(i + 1) % 3][2] - t[i][2]) * (t[(i + 1) % 3] - t[i])
                   for i in range(3) if (t[i][2] - z) * (t[(i + 1) % 3][2] - z) < 0]
            if len(pts) >= 2:
                cv2.line(m, cell(pts[0]), cell(pts[1]), 1, 1)
    return m.astype(bool)


def last_map(bag):
    msg = None
    for _, m, _ in rosbag.Bag(bag).read_messages(topics=['/map_2d']):
        msg = m
    return msg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--tol', type=float, default=0.10)
    ap.add_argument('--png')
    a = ap.parse_args()

    msg = last_map(a.bag)
    if msg is None:
        raise SystemExit('no /map_2d in %s' % a.bag)
    i = msg.info
    w, h, res = i.width, i.height, i.resolution
    x0, y0 = i.origin.position.x, i.origin.position.y
    g = np.array(msg.data, np.int16).reshape(h, w)
    name, tri = arena_triangles()
    truth = truth_raster(tri, x0, y0, w, h, res, (0.5, 1.0, 1.5, 2.0))

    k = max(1, int(round(a.tol / res)))
    ker = np.ones((2 * k + 1, 2 * k + 1), np.uint8)
    wall = g == 100
    free = g == 0
    truth_near = cv2.dilate(truth.astype(np.uint8), ker).astype(bool)
    wall_near = cv2.dilate(wall.astype(np.uint8), ker).astype(bool)
    # recall only over walls next to explored floor: walls the drone could have seen
    explored = cv2.dilate((free | wall).astype(np.uint8), np.ones((9, 9), np.uint8)).astype(bool)
    observable = truth & explored
    precision = (wall & truth_near).sum() / max(1, wall.sum())
    recall = (observable & wall_near).sum() / max(1, observable.sum())
    false_free = (free & truth).sum() / max(1, free.sum())

    print('map %s  (%d x %d @ %.2f m, arena %s, tolerance %.2f m)' % (os.path.basename(a.bag), w, h,
                                                                       res, name, a.tol))
    print('  wall precision %.3f   wall recall %.3f   false free %.4f   explored %.1f m2'
          % (precision, recall, false_free, free.sum() * res * res))
    if a.png:
        img = np.full((h, w, 3), 13, np.uint8)
        img[free] = (40, 40, 40)
        img[wall] = (230, 230, 230)
        img[truth & ~wall_near] = (0, 0, 255)
        sc = 760.0 / max(w, h)          # same picture size whatever the map resolution
        cv2.imwrite(a.png, cv2.resize(img[::-1], (int(round(w * sc)), int(round(h * sc))),
                                      interpolation=cv2.INTER_NEAREST))
        print('  picture: %s' % a.png)


if __name__ == '__main__':
    main()
