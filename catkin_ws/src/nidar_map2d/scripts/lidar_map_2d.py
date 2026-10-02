#!/usr/bin/env python3
"""Live 2D floorplan built directly from the LiDAR (Mission Brief §1, §5: a 2D map of the explored
area, generated and continuously updated during flight).

Publishes /map_2d (nav_msgs/OccupancyGrid, frame `world`): -1 unknown, 0 free, 100 wall.

Why not slice FUEL's map (map_2d_slicer.py, kept for comparison)? FUEL's /sdf_map/occupancy_all
carries ONLY occupied voxels (see plan_env/src/map_ros.cpp), so a slice of it never shows the
explored floor: it is wall fragments on an unknown background, not a floorplan. FUEL's map box
also stops 0.7 m inside the arena, so the outer walls are never mapped. Measured against the
arena mesh on three runs, only 26-50 % of the real walls appeared in that map.

This node does standard 2D occupancy-grid mapping on FAST-LIO's registered scans:
  * every return in the wall band (z in [hit_z_min, hit_z_max] above the floor) marks its cell
    occupied;
  * the laser ray from the sensor to every return (wall, floor or anything else) marks the cells
    it crossed as free -- the beam got through, so nothing blocks that column at the heights it
    traversed;
  * evidence is accumulated as log-odds, so a cell seen as wall many times survives an odd
    grazing ray, and a transient return is cleared by later rays.
The result is a floorplan of exactly what the drone has seen, walls and floor, including the
outer walls and the launch pad, at map resolution (default 0.05 m). Its accuracy is the accuracy
of FAST-LIO's pose.
"""
import time

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

# Log-odds increments (a hit is stronger evidence than a pass-through) and clamp.
L_OCC = 0.85
L_FREE = -0.40
L_MIN, L_MAX = -2.0, 3.5
OCC_THRESH = 0.5     # > : wall
FREE_THRESH = -0.2   # < : free (a single clear pass is enough to call it floor)


# -- Pure functions (unit-tested in test/test_lidar_map_2d.py) -----------------------------------

class GridSpec(object):
    """World-aligned grid: cell (ix, iy) covers [x0 + ix*res, x0 + (ix+1)*res) x [y0 + iy*res, ...)."""

    def __init__(self, x0, y0, width, height, res):
        self.x0, self.y0, self.width, self.height, self.res = x0, y0, int(width), int(height), res

    def cells(self, xy):
        """Linear cell index for each (x, y) row, and a mask of those inside the grid."""
        ix = np.floor((xy[:, 0] - self.x0) / self.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.y0) / self.res).astype(np.int64)
        ok = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        return iy * self.width + ix, ok


def ray_cells(grid, origin_xy, ends_xy, stop_short):
    """Unique linear indices of the cells crossed by 2D segments origin -> end.

    Each segment is sampled every half cell and stops `stop_short` metres before its end (so a
    wall cell is not cleared by the ray that hit it).
    """
    if len(ends_xy) == 0:
        return np.empty(0, np.int64)
    d = ends_xy - origin_xy
    length = np.hypot(d[:, 0], d[:, 1])
    usable = length - stop_short
    keep = usable > 0
    if not np.any(keep):
        return np.empty(0, np.int64)
    d, length, usable = d[keep], length[keep], usable[keep]
    step = grid.res * 0.5
    n = int(np.ceil(usable.max() / step)) + 1
    t = np.arange(n) * step                                   # metres along the ray
    inside = t[None, :] <= usable[:, None]                    # (rays, samples)
    unit = d / length[:, None]
    px = origin_xy[0] + unit[:, 0:1] * t[None, :]
    py = origin_xy[1] + unit[:, 1:2] * t[None, :]
    pts = np.stack([px[inside], py[inside]], axis=1)
    idx, ok = grid.cells(pts)
    return np.unique(idx[ok])


def integrate_scan(logodds, grid, sensor_xyz, points_world, hit_z_min, hit_z_max, max_range,
                   min_range=0.3, max_free_rays=4000, rng=None):
    """Update `logodds` (flat float32 array, grid.width*grid.height) with one registered scan.

    sensor_xyz    sensor position, world frame
    points_world  (N, 3) returns, world frame (z measured from the floor)
    Returns (n_hit_cells, n_free_cells) for diagnostics.
    """
    if len(points_world) == 0:
        return 0, 0
    s = np.asarray(sensor_xyz, float)
    xy = points_world[:, :2]
    rng_2d = np.hypot(xy[:, 0] - s[0], xy[:, 1] - s[1])
    valid = (rng_2d >= min_range) & (rng_2d <= max_range) & np.isfinite(rng_2d)
    pts = points_world[valid]
    if len(pts) == 0:
        return 0, 0
    is_hit = (pts[:, 2] >= hit_z_min) & (pts[:, 2] <= hit_z_max)

    # free space: rays to every return (subsampled; the cells are what matters, not the rays)
    free_pts = pts[:, :2]
    if len(free_pts) > max_free_rays:
        rng = rng or np.random
        free_pts = free_pts[rng.choice(len(free_pts), max_free_rays, replace=False)]
    free = ray_cells(grid, s[:2], free_pts, stop_short=grid.res * 1.5)

    hit_idx, ok = grid.cells(pts[is_hit, :2])
    hits = np.unique(hit_idx[ok])

    logodds[free] += L_FREE
    logodds[hits] += L_OCC          # after the free pass: a cell hit in this scan stays a wall
    np.clip(logodds, L_MIN, L_MAX, out=logodds)
    return len(hits), len(free)


def to_occupancy(logodds):
    out = np.full(logodds.shape, -1, np.int8)
    out[logodds < FREE_THRESH] = 0
    out[logodds > OCC_THRESH] = 100
    return out


# -- ROS node ---------------------------------------------------------------------------------------

class LidarMap2D(object):
    def __init__(self):
        rospy.init_node('lidar_map_2d', anonymous=False)
        active = rospy.get_param('/nidar/arena/active', 'arina_nidar')
        b = rospy.get_param('/nidar/arenas/%s/bounds' % active,
                            {'x_min': -7.5, 'x_max': 7.5, 'y_min': -7.5, 'y_max': 7.5})
        pad = rospy.get_param('/nidar/launch_pad/center', {'x': 0.0, 'y': -9.5})
        margin = float(rospy.get_param('~margin', 0.5))
        res = float(rospy.get_param('~resolution', 0.05))
        # Cover the arena (outer walls included) and the launch pad south of the entry.
        x0 = min(float(b['x_min']), float(pad['x']) - 1.0) - margin
        x1 = max(float(b['x_max']), float(pad['x']) + 1.0) + margin
        y0 = min(float(b['y_min']), float(pad['y']) - 1.0) - margin
        y1 = max(float(b['y_max']), float(pad['y']) + 1.0) + margin
        self.grid = GridSpec(x0, y0, round((x1 - x0) / res), round((y1 - y0) / res), res)
        self.logodds = np.zeros(self.grid.width * self.grid.height, np.float32)

        self.hit_z_min = float(rospy.get_param('~hit_z_min', 0.30))   # above floor clutter
        self.hit_z_max = float(rospy.get_param('~hit_z_max', 2.20))   # below the net / ceiling
        self.max_range = float(rospy.get_param('~max_range', 12.0))
        self.max_rate = float(rospy.get_param('~max_scan_rate', 5.0))
        self.max_free_rays = int(rospy.get_param('~max_free_rays', 4000))
        self.publish_rate = float(rospy.get_param('~publish_rate', 1.0))
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.cloud_frame = rospy.get_param('~cloud_frame', 'camera_init')

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.T = None                  # world <- cloud frame, static
        self.sensor = None             # latest odometry position, cloud frame
        self.last_scan = 0.0
        self.scans = 0
        self.dirty = False
        self.stats = []

        self.pub = rospy.Publisher('/map_2d', OccupancyGrid, latch=True, queue_size=1)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self._odom_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param('~cloud_topic', '/cloud_registered'), PointCloud2,
                         self._cloud_cb, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._publish)
        rospy.Timer(rospy.Duration(20.0), self._report)
        rospy.loginfo('[map_2d] LiDAR 2D mapper ready: %d x %d @ %.2f m, x=[%.1f, %.1f] y=[%.1f, %.1f], '
                      'walls z=[%.2f, %.2f] m', self.grid.width, self.grid.height, res, x0, x1, y0, y1,
                      self.hit_z_min, self.hit_z_max)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.sensor = (p.x, p.y, p.z)

    def _world_T(self):
        if self.T is None:
            try:
                tr = self.tf_buf.lookup_transform(self.world_frame, self.cloud_frame, rospy.Time(0))
            except Exception:
                return None
            q = tr.transform.rotation
            T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[:3, 3] = [tr.transform.translation.x, tr.transform.translation.y,
                        tr.transform.translation.z]
            self.T = T.astype(np.float64)
        return self.T

    def _cloud_cb(self, msg):
        now = rospy.get_time()
        if now - self.last_scan < 1.0 / self.max_rate or self.sensor is None or not msg.data:
            return
        T = self._world_T()
        if T is None:
            rospy.logwarn_throttle(10.0, '[map_2d] waiting for TF %s -> %s', self.world_frame,
                                   self.cloud_frame)
            return
        self.last_scan = now
        t0 = time.time()
        off = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                       'offsets': [off['x'], off['y'], off['z']], 'itemsize': msg.point_step})
        raw = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
        P = np.stack([raw['x'], raw['y'], raw['z']], axis=1).astype(np.float64)
        P = P[np.isfinite(P).all(axis=1)]
        Pw = P.dot(T[:3, :3].T) + T[:3, 3]
        sw = T[:3, :3].dot(self.sensor) + T[:3, 3]
        n_hit, n_free = integrate_scan(self.logodds, self.grid, sw, Pw, self.hit_z_min,
                                       self.hit_z_max, self.max_range,
                                       max_free_rays=self.max_free_rays)
        self.scans += 1
        self.dirty = True
        self.stats.append((time.time() - t0, len(Pw), n_hit, n_free))

    def _publish(self, _event):
        if not self.dirty:
            return
        self.dirty = False
        g = self.grid
        meta = MapMetaData()
        meta.resolution = g.res
        meta.width = g.width
        meta.height = g.height
        meta.origin = Pose()
        meta.origin.position.x = g.x0
        meta.origin.position.y = g.y0
        meta.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        meta.map_load_time = rospy.Time.now()
        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id=self.world_frame)
        msg.info = meta
        msg.data = to_occupancy(self.logodds).tolist()   # int8[]: a list (bytes would be uint8)
        self.pub.publish(msg)

    def _report(self, _event):
        if not self.stats:
            return
        s = np.array(self.stats)
        self.stats = []
        occ = to_occupancy(self.logodds)
        rospy.loginfo('[map_2d] %d scans so far | last 20 s: %d scans, %.0f ms/scan (max %.0f), '
                      '%.0f pts/scan | map: %d wall, %d free cells', self.scans, len(s),
                      1000 * s[:, 0].mean(), 1000 * s[:, 0].max(), s[:, 1].mean(),
                      int((occ == 100).sum()), int((occ == 0).sum()))


if __name__ == '__main__':
    try:
        LidarMap2D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
