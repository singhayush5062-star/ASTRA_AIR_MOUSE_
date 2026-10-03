#!/usr/bin/env python3
"""Re-stamp FUEL's visualisation markers with the frame they are really in, for RViz.

FUEL hard-codes header.frame_id = "world" on every marker it publishes, but in this stack it
plans on FAST-LIO odometry (nidar_planner/launch/nidar_fuel_upstream.launch sets
exploration_node/map_ros/frame_id = camera_init), so the coordinates are in `camera_init`.
The real arena `world` frame is yawed 90 deg and offset 9.5 m from it (world -> map ->
camera_init), so RViz with fixed frame `world` would draw FUEL's paths in the wrong place.

Patching FUEL is not an option: the same "world" string is in ~15 places, and traj_server's
/planning/pos_cmd (consumed by the flight stack) carries it too. This node only touches
copies of the display topics:

    /planning_vis/trajectory   -> /fuel_vis/trajectory    planned B-spline (where it flies next)
    /planning/travel_traj      -> /fuel_vis/travel_traj   trajectory FUEL has executed so far
    /planning_vis/frontier     -> /fuel_vis/frontier      frontier clusters
    /planning_vis/viewpoints   -> /fuel_vis/viewpoints    viewpoints of the exploration tour

Messages are relayed as raw bytes with only the header frame_id spliced, so even
travel_traj (tens of thousands of points at 10 Hz) costs almost nothing, and nothing is
forwarded while no one subscribes.
"""

import struct

import rospy
from visualization_msgs.msg import Marker

TOPICS = [
    ('/planning_vis/trajectory', 'trajectory'),
    ('/planning/travel_traj', 'travel_traj'),
    ('/planning_vis/frontier', 'frontier'),
    ('/planning_vis/viewpoints', 'viewpoints'),
]


def restamp(raw, frame_id):
    """Serialized std_msgs/Header is seq, stamp.secs, stamp.nsecs (3 x uint32) then frame_id
    as uint32 length + bytes. Replace the frame_id, keep everything else byte-for-byte."""
    n = struct.unpack_from('<I', raw, 12)[0]
    return raw[:12] + struct.pack('<I', len(frame_id)) + frame_id + raw[16 + n:]


class RawMarker(Marker):
    """A Marker that serializes as the bytes it was given (type and md5sum are Marker's)."""
    __slots__ = ['_raw']
    _has_header = False   # stops rospy writing header.seq into a message that has no fields

    def __init__(self, raw):
        self._raw = raw

    def serialize(self, buff):
        buff.write(self._raw)


def main():
    rospy.init_node('fuel_vis_relay')
    frame = rospy.get_param('~frame_id',
                            rospy.get_param('/exploration_node/map_ros/frame_id', 'camera_init'))
    prefix = rospy.get_param('~prefix', '/fuel_vis')
    frame_b = frame.encode()
    subs = []
    for src, name in TOPICS:
        pub = rospy.Publisher('%s/%s' % (prefix, name), Marker, queue_size=10)

        def cb(msg, pub=pub):
            if pub.get_num_connections():
                pub.publish(RawMarker(restamp(msg._buff, frame_b)))

        subs.append(rospy.Subscriber(src, rospy.AnyMsg, cb, queue_size=10))
    rospy.loginfo('[fuel_vis_relay] FUEL markers -> %s/* with frame_id %s', prefix, frame)
    rospy.spin()


if __name__ == '__main__':
    main()
