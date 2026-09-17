#!/usr/bin/env python3
"""Grid overlay and survivor-tag markers for RViz / Foxglove.

Phase 5 (§5.2 of PLANNING_DOCS/nidar_phase_plan_to_mission_complete.md).

Two things live in one node because they share the arena grid config and
both publish a MarkerArray on top of the same 2D map:

    /grid_markers      -- 2 m competition grid lines + cell IDs (A1..G7 style)
    /survivor_tags     -- per-survivor marker + text label showing the grid
                          cell the tag lives in

The grid overlay is a static publication -- one MarkerArray at startup,
republished on a low-rate timer so a late-joining GCS gets it without a
transient race. The survivor tags update whenever /survivors changes; the
publication itself is latched, and includes the FULL current set every time
(never a delta) so a GCS that joined mid-mission sees every confirmed tag.

Cell IDs use the phase plan's A1..N14 example convention: rows A..G for the
7-row grid, columns 1..7 for the 7-column grid. The mapping is:
    label = row_letter[gy] + column_str[gx + 1]
so world (0, 0) sits in grid (3, 3) which labels as "D4".
"""

import string

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# Optional: /survivors is a nidar_mission SurvivorArray if the messages are
# built; fall back gracefully to no-op if the message package hasn't been
# generated (this node still publishes the grid overlay).
try:
    from nidar_mission.msg import SurvivorArray
    _HAS_SURVIVOR_MSG = True
except Exception:
    _HAS_SURVIVOR_MSG = False


class GridVisualizer(object):

    def __init__(self):
        rospy.init_node('grid_visualizer', anonymous=False)

        self.frame = rospy.get_param('~frame_id', 'map')
        self.origin_x = float(rospy.get_param('/arena_grid/origin_x', -7.0))
        self.origin_y = float(rospy.get_param('/arena_grid/origin_y', -7.0))
        self.cell = float(rospy.get_param('/arena_grid/cell_size', 2.0))
        self.nx = int(rospy.get_param('/arena_grid/cells_x', 7))
        self.ny = int(rospy.get_param('/arena_grid/cells_y', 7))
        self.z = float(rospy.get_param('~overlay_z', 0.02))  # sits just above floor

        self._pub_grid = rospy.Publisher('/grid_markers', MarkerArray,
                                         latch=True, queue_size=1)
        self._pub_tags = rospy.Publisher('/survivor_tags', MarkerArray,
                                         latch=True, queue_size=1)

        if _HAS_SURVIVOR_MSG:
            rospy.Subscriber('/survivors', SurvivorArray, self._survivors_cb,
                             queue_size=5)
            rospy.loginfo('[grid_viz] subscribing to /survivors (SurvivorArray)')
        else:
            rospy.logwarn('[grid_viz] nidar_mission.msg.SurvivorArray not '
                          'available; survivor tag publisher disabled')

        # Publish the static grid overlay once now and again every 5 s so any
        # GCS joining mid-flight receives it. MarkerArray is not natively
        # latched at the marker level.
        self._publish_grid_overlay()
        rospy.Timer(rospy.Duration(5.0), lambda _e: self._publish_grid_overlay())

        # An empty tag set at startup keeps the /survivor_tags topic alive
        # even before any confirmed detection lands.
        self._pub_tags.publish(MarkerArray())

        rospy.loginfo('[grid_viz] Grid overlay ready: %dx%d cells @ %.2f m '
                      '(frame=%s)', self.nx, self.ny, self.cell, self.frame)

    # ---- Grid overlay ---------------------------------------------------

    def _cell_label(self, gx, gy):
        """Return e.g. "D4" for the arena grid cell (gx=3, gy=3). The row
        letter runs A..G (south to north), the column number 1..7 (west to
        east). The phase plan §5.2 example uses A1..N14; this arena is a
        7x7 grid, so it stops at G7.
        """
        row = string.ascii_uppercase[gy] if gy < 26 else 'Z'
        return '%s%d' % (row, gx + 1)

    def _publish_grid_overlay(self):
        markers = MarkerArray()
        marker_id = 0

        # One vertical line per grid column (gx = 0..nx) and one horizontal
        # line per grid row (gy = 0..ny). LINE_LIST lets us put both in one
        # marker apiece, cheaper than one Marker per line.
        vlines = Marker()
        vlines.header.frame_id = self.frame
        vlines.header.stamp = rospy.Time.now()
        vlines.ns = 'grid_lines'
        vlines.id = marker_id
        marker_id += 1
        vlines.type = Marker.LINE_LIST
        vlines.action = Marker.ADD
        vlines.scale.x = 0.03  # line width
        vlines.color = ColorRGBA(0.7, 0.7, 0.7, 0.7)
        vlines.pose.orientation.w = 1.0
        y0 = self.origin_y
        y1 = self.origin_y + self.ny * self.cell
        for gx in range(self.nx + 1):
            x = self.origin_x + gx * self.cell
            vlines.points.append(Point(x, y0, self.z))
            vlines.points.append(Point(x, y1, self.z))
        x0 = self.origin_x
        x1 = self.origin_x + self.nx * self.cell
        for gy in range(self.ny + 1):
            y = self.origin_y + gy * self.cell
            vlines.points.append(Point(x0, y, self.z))
            vlines.points.append(Point(x1, y, self.z))
        markers.markers.append(vlines)

        # Cell ID text at each cell centre. Text markers are one Marker each
        # in Rviz's data model, so this is unavoidable.
        for gy in range(self.ny):
            for gx in range(self.nx):
                t = Marker()
                t.header.frame_id = self.frame
                t.header.stamp = rospy.Time.now()
                t.ns = 'grid_labels'
                t.id = marker_id
                marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = self.origin_x + (gx + 0.5) * self.cell
                t.pose.position.y = self.origin_y + (gy + 0.5) * self.cell
                t.pose.position.z = self.z + 0.05
                t.pose.orientation.w = 1.0
                t.scale.z = 0.25
                t.color = ColorRGBA(0.5, 0.5, 0.5, 0.8)
                t.text = self._cell_label(gx, gy)
                markers.markers.append(t)

        self._pub_grid.publish(markers)

    # ---- Survivor tags --------------------------------------------------

    def _survivors_cb(self, msg):
        """Republish the FULL current set of survivor tags. The GCS renders
        MarkerArray by NS+ID, so a delta (only-new tags) would leave stale
        tags on-screen forever if a survivor was removed.
        """
        markers = MarkerArray()
        for s in msg.survivors:
            # Symbol: red sphere at survivor position, slightly above the
            # overlay so it doesn't Z-fight the grid lines.
            sphere = Marker()
            sphere.header.frame_id = self.frame
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'survivor'
            sphere.id = int(s.id)
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(s.position.x)
            sphere.pose.position.y = float(s.position.y)
            sphere.pose.position.z = 0.5
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35
            sphere.color = ColorRGBA(1.0, 0.15, 0.15, 0.9)
            markers.markers.append(sphere)

            # Text label with cell ID and confidence.
            gx = int(getattr(s, 'grid_x', 0))
            gy = int(getattr(s, 'grid_y', 0))
            in_grid = 0 <= gx < self.nx and 0 <= gy < self.ny
            label = self._cell_label(gx, gy) if in_grid else '(off-grid)'
            txt = Marker()
            txt.header.frame_id = self.frame
            txt.header.stamp = rospy.Time.now()
            txt.ns = 'survivor_label'
            txt.id = int(s.id)
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = float(s.position.x)
            txt.pose.position.y = float(s.position.y)
            txt.pose.position.z = 1.4
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.30
            txt.color = ColorRGBA(1.0, 0.9, 0.0, 1.0)
            txt.text = 'S%d %s (%.0f%%)' % (int(s.id), label, 100.0 * float(s.confidence))
            markers.markers.append(txt)

        self._pub_tags.publish(markers)
        rospy.loginfo_throttle(5.0, '[grid_viz] published %d survivor tag(s)',
                               len(msg.survivors))


if __name__ == '__main__':
    try:
        GridVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
