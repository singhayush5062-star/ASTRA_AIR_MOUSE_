#!/usr/bin/env python3
"""Coverage reporter: prints a terminal line at every 20% coverage growth.

Subscribes to /sdf_map/coverage, a Float64MultiArray FUEL's MapROS publishes
every map_ros/coverage_interval. The schema (see map_ros.cpp:207-210) is:

    data[0]  free_n         voxel count
    data[1]  occ_n
    data[2]  unk_n
    data[3]  free_area      m^2
    data[4]  known_area     m^2
    data[5]  unknown_area   m^2
    data[6]  denom          arena_area_m2 if set, else observable
    data[7]  pct            free / (free + unknown), the coverage percentage
    data[8]  left           unknown_area
    data[9]  layers
    data[10] elapsed_s      time since MapROS started

The line printed at each 20/40/60/80/100 threshold is unmissable in stdout and
carries both the percentage AND the m^2 breakdown, so an operator watching only
the terminal has full situational awareness even before the GCS is available.

For per-grid-cell breakdown, subscribes to /sdf_map/occupancy_all (a
sensor_msgs/PointCloud2 of free+occupied voxels, no unknowns) and buckets each
point into the arena grid cell it falls in. That gives a "which cells are still
completely unknown" line that names the survivor-hosting cells the drone has
not yet reached.
"""
import struct

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray


class CoverageReporter(object):

    def __init__(self):
        rospy.init_node('coverage_reporter', anonymous=False)

        # Thresholds, in ascending order. We report the FIRST time coverage
        # crosses each -- never revisit, so a wobble around 40% doesn't spam.
        self.thresholds = [20.0, 40.0, 60.0, 80.0, 95.0]
        self._crossed = set()

        # Grid definition (for the per-cell breakdown printed alongside).
        # Falls back silently if the params are missing so this node stays
        # useful even before arena_grid.yaml is loaded.
        try:
            self.gx0 = float(rospy.get_param('/arena_grid/origin_x'))
            self.gy0 = float(rospy.get_param('/arena_grid/origin_y'))
            self.cs = float(rospy.get_param('/arena_grid/cell_size'))
            self.nx = int(rospy.get_param('/arena_grid/cells_x'))
            self.ny = int(rospy.get_param('/arena_grid/cells_y'))
            self._have_grid = True
        except KeyError:
            rospy.logwarn('[coverage] /arena_grid/* not loaded; per-cell '
                          'breakdown will be suppressed')
            self._have_grid = False

        # Latest cell-level occupancy snapshot; refreshed by _cloud_cb and
        # sampled at each 20% report.
        self._cell_free_count = None
        self._cloud_topic = rospy.get_param('~cloud_topic', '/sdf_map/occupancy_all')
        rospy.Subscriber(self._cloud_topic, PointCloud2, self._cloud_cb,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/sdf_map/coverage', Float64MultiArray,
                         self._cov_cb, queue_size=10)

        rospy.loginfo('[coverage] Coverage reporter ready. Watching '
                      '/sdf_map/coverage for 20/40/60/80/95%% crossings.')
        # First-run heartbeat: makes "coverage reporter is alive" visible even
        # before the map has anything to say (very early in a run pct is 0).

    def _cov_cb(self, msg):
        # Guard against a short array from a future/upstream schema shift.
        if len(msg.data) < 11:
            return
        free_area = float(msg.data[3])
        unknown_area = float(msg.data[5])
        denom_area = float(msg.data[6])
        pct = float(msg.data[7])
        left_area = float(msg.data[8])
        elapsed_s = float(msg.data[10])
        observable = free_area + unknown_area

        for thr in self.thresholds:
            if pct >= thr and thr not in self._crossed:
                self._crossed.add(thr)
                self._report(thr, pct, free_area, unknown_area, observable,
                             denom_area, left_area, elapsed_s)

    def _cloud_cb(self, msg):
        if not self._have_grid:
            return
        # PointCloud2 with fields x,y,z,f (float32, offset 0/4/8/12; a fourth
        # field carries occupancy state but we only need the xy footprint).
        # Reading via struct is 5-10x faster than sensor_msgs.point_cloud2 and
        # this runs at map cadence (~2 Hz).
        step = msg.point_step
        data = memoryview(msg.data)
        counts = np.zeros((self.nx, self.ny), dtype=np.int32)
        # Bounds precomputed
        x0, y0, cs = self.gx0, self.gy0, self.cs
        nx, ny = self.nx, self.ny
        # Reject cells above the arena or below the floor
        for k in range(0, msg.width * msg.height * step, step):
            x, y = struct.unpack_from('ff', data, k)
            i = int((x - x0) // cs)
            j = int((y - y0) // cs)
            if 0 <= i < nx and 0 <= j < ny:
                counts[i, j] += 1
        self._cell_free_count = counts

    def _report(self, thr, pct, free_area, unknown_area, observable,
                denom_area, left_area, elapsed_s):
        # The prominent milestone line -- easy to grep and to spot in a scroll.
        rospy.loginfo(
            '[Coverage] %.0f%% REACHED at t=%.1fs | pct=%.2f%% | '
            'free=%.1f m^2 | unknown=%.1f m^2 (still to map) | '
            'observable=%.1f m^2 | denom=%.1f m^2',
            thr, elapsed_s, pct, free_area, unknown_area, observable, denom_area)

        # Per-cell breakdown: how many grid cells have ANY free-space samples
        # yet, so an operator can see which parts of the arena remain dark.
        if self._have_grid and self._cell_free_count is not None:
            counts = self._cell_free_count
            total = self.nx * self.ny
            seen = int(np.count_nonzero(counts))
            rospy.loginfo(
                '[Coverage]   grid: %d/%d cells have >=1 free-space sample '
                '(%.0f%%). Still-dark cells:',
                seen, total, 100.0 * seen / total)
            # List the still-dark cells, up to 12, so the terminal line stays
            # legible. Beyond 12 we just report the count.
            dark = []
            for j in range(self.ny):
                for i in range(self.nx):
                    if counts[i, j] == 0:
                        dark.append((i, j))
            if not dark:
                rospy.loginfo('[Coverage]     (none)')
            else:
                shown = dark[:12]
                more = len(dark) - len(shown)
                cells_str = ', '.join('(%d,%d)' % ij for ij in shown)
                rospy.loginfo('[Coverage]     %s%s',
                              cells_str,
                              (' + %d more' % more) if more > 0 else '')


def main():
    CoverageReporter()
    rospy.spin()


if __name__ == '__main__':
    main()
