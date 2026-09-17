#!/usr/bin/env python3
"""Live 2D occupancy slice for the GCS.

Phase 5 (§5.1 of PLANNING_DOCS/nidar_phase_plan_to_mission_complete.md).

Subscribes to /sdf_map/occupancy_all -- the PointCloud2 FUEL/SDFMap already
publishes at map cadence, one point per voxel with a fourth field carrying
occupancy state (0 free, 1 occupied, 2 unknown; see fast_planner sdf_map.cpp).

Slices the voxel volume at [z_min, z_max] around cruise altitude, projects
each cell into a 2D nav_msgs/OccupancyGrid, and publishes /map_2d at
publish_rate Hz. Values in the OccupancyGrid follow the ROS convention:
    -1  unknown
     0  free
   100  occupied

Why this exists:
    The competition brief §5 requires a live 2D map generated DURING flight,
    legible as a floorplan. FUEL's own occupancy_all is 3D and voxel-shaped,
    not consumable by a stock nav_msgs viewer, so a slicer is the smallest
    bridge to a standard OccupancyGrid.

Design decisions that matter:
    * Slice band [0.3, 1.9] m by DEFAULT covers walls but rejects the floor
      (avoids marking every tile as an obstacle) and the ceiling/net (avoids
      marking every point as blocked). Configurable via ~z_slice_min/max.
    * OccupancyGrid resolution matches the SDFMap resolution (0.10 m) so
      there is no cell-mapping ambiguity. Overriding via ~resolution is
      allowed for a coarser map that ships smaller to the GCS.
    * Publish rate is CONFIGURABLE. Default 2 Hz keeps GCS bandwidth low
      (phase plan §6.2 caps combined GCS bandwidth at 4 Mbps; a 400x400 cell
      grid at 1 byte/cell is 160 kB/msg, so 2 Hz = 320 kB/s = 2.5 Mbps
      already, leaving headroom for the camera).
    * frame_id is "map" so the world_to_map static TF (published by
      launch/nidar_fuel_upstream.launch) puts this grid in world coordinates
      directly. Nothing here does its own frame math.

Related Phase 5 nodes:
    * grid_visualizer.py -- publishes the 2m competition grid overlay and
      per-survivor tag markers on top of this OccupancyGrid.
"""

import struct

import numpy as np
import rospy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class Map2DSlicer(object):

    def __init__(self):
        rospy.init_node('map_2d_slicer', anonymous=False)

        # Arena bounds -- fall back to the standard 15x15 m arena so the grid
        # is a sane shape even before mission_config.yaml is loaded.
        active = rospy.get_param('/nidar/arena/active', 'arina_nidar')
        bounds = rospy.get_param('/nidar/arenas/%s/bounds' % active,
                                 {'x_min': -7.5, 'x_max': 7.5,
                                  'y_min': -7.5, 'y_max': 7.5})
        self.x_min = float(bounds['x_min'])
        self.x_max = float(bounds['x_max'])
        self.y_min = float(bounds['y_min'])
        self.y_max = float(bounds['y_max'])

        # Cell size. Default matches SDFMap's map_resolution.
        self.resolution = float(rospy.get_param('~resolution',
                                                rospy.get_param('/nidar/planner/map_resolution', 0.1)))
        self.width = int(round((self.x_max - self.x_min) / self.resolution))
        self.height = int(round((self.y_max - self.y_min) / self.resolution))

        # Slice band. Wide enough to catch a doorway lintel at 2 m, narrow
        # enough to reject the net at 2.44 m.
        self.z_min = float(rospy.get_param('~z_slice_min', 0.3))
        self.z_max = float(rospy.get_param('~z_slice_max', 1.9))

        # Publish cadence.
        self.publish_rate = float(rospy.get_param('~publish_rate', 2.0))

        # PointCloud2 field layout (from SDFMap): float32 x, y, z, f. Offsets
        # 0, 4, 8, 12 (16 bytes point step). f encodes occupancy state:
        # 0=free, 1=occupied, 2=unknown. Read the whole cloud as a numpy
        # structured array in one shot -- 5-10x faster than iterating.
        self._point_dt = np.dtype([('x', np.float32),
                                   ('y', np.float32),
                                   ('z', np.float32),
                                   ('f', np.float32)])
        self._cloud_topic = rospy.get_param('~cloud_topic', '/sdf_map/occupancy_all')

        # Latest map data -- built by _cloud_cb, published by _tick.
        self._latest_grid = None
        self._latest_stamp = None

        self._pub = rospy.Publisher('/map_2d', OccupancyGrid, latch=True, queue_size=1)
        rospy.Subscriber(self._cloud_topic, PointCloud2, self._cloud_cb,
                         queue_size=1, buff_size=2 ** 24)

        rospy.loginfo('[map_2d] Map2DSlicer ready. Publishing /map_2d '
                      '(%d x %d @ %.2f m) at %.1f Hz from %s (slice z=[%.2f, %.2f])',
                      self.width, self.height, self.resolution, self.publish_rate,
                      self._cloud_topic, self.z_min, self.z_max)

        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._tick)

    def _cloud_cb(self, msg):
        """Slice the voxel cloud into a 2D grid.

        The cloud may hold ~100k points at map extent; the whole computation
        below is numpy-vectorised so this stays cheap on the sim laptop.
        """
        if not msg.data:
            return

        # 16-byte point step. Reinterpret the whole buffer as our dtype.
        n = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=self._point_dt, count=n)
        xs = raw['x'].astype(np.float32)
        ys = raw['y'].astype(np.float32)
        zs = raw['z'].astype(np.float32)
        fs = raw['f'].astype(np.int32)

        # Only points inside the slab AND inside the arena affect the grid.
        in_slab = (zs >= self.z_min) & (zs <= self.z_max)
        in_arena = (xs >= self.x_min) & (xs < self.x_max) \
                   & (ys >= self.y_min) & (ys < self.y_max)
        keep = in_slab & in_arena
        if not np.any(keep):
            return

        # Cell indices. y is the row-major MAJOR axis in ROS OccupancyGrid
        # (data[y * width + x]).
        gx = ((xs[keep] - self.x_min) / self.resolution).astype(np.int32)
        gy = ((ys[keep] - self.y_min) / self.resolution).astype(np.int32)
        # Clamp defensively -- floating-point noise on the boundary otherwise
        # produces an out-of-range index.
        gx = np.clip(gx, 0, self.width - 1)
        gy = np.clip(gy, 0, self.height - 1)
        st = fs[keep]  # 0=free, 1=occupied, 2=unknown

        grid = -np.ones(self.width * self.height, dtype=np.int8)  # unknown

        # Occupied wins over free wins over unknown in a slab: if any voxel
        # in the (x, y) column above the floor is occupied, the 2D cell is
        # a wall; else if any is free, the cell is floor.
        idx = gy * self.width + gx
        # Free first (so occupied can overwrite it in the vectorised pass below).
        grid[idx[st == 0]] = 0
        grid[idx[st == 1]] = 100

        self._latest_grid = grid
        self._latest_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

    def _tick(self, _event):
        if self._latest_grid is None:
            return

        # OccupancyGrid uses a Pose in the map frame for the grid origin
        # (lower-left corner). No rotation; identity quaternion.
        origin = Pose()
        origin.position.x = self.x_min
        origin.position.y = self.y_min
        origin.position.z = 0.0
        origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

        meta = MapMetaData()
        meta.map_load_time = self._latest_stamp
        meta.resolution = self.resolution
        meta.width = self.width
        meta.height = self.height
        meta.origin = origin

        msg = OccupancyGrid()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='map')
        msg.info = meta
        msg.data = self._latest_grid.tolist()

        self._pub.publish(msg)


if __name__ == '__main__':
    try:
        Map2DSlicer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
