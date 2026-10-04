#!/usr/bin/env python3
"""Livox Mid-360 -> FAST-LIO relay (hardware only).

Two jobs, both done on the raw serialized bytes (no per-point Python work):

1. TYPE. The Mid-360 needs livox_ros_driver2, which publishes livox_ros_driver2/CustomMsg.
   This repo's FAST-LIO (and the simulation) is built against livox_ros_driver/CustomMsg. The
   two message definitions are byte-for-byte identical (Header, timebase, point_num, lidar_id,
   rsvd[3], CustomPoint[]{offset_time, x, y, z, reflectivity, tag, line}), so the scan is
   republished unchanged under FAST-LIO's type: /livox/lidar -> /nidar/livox/lidar.

2. TIME. Without PTP the Mid-360 stamps scans and IMU samples with its own clock (time since
   power-on), while every other ROS node (MAVROS, the camera, tf) uses the Jetson's clock.
   FAST-LIO itself only needs scans and IMU on one clock -- they share the LiDAR's -- but its
   odometry and tf would then be stamped seconds-since-boot, and nothing downstream could look
   up a transform at a camera image's time. So both streams are shifted onto ROS time by one
   common offset, estimated as the minimum (receive time - sensor stamp) over a sliding window:
   the minimum is the least-delayed sample, so jitter does not enter, and the window lets it
   follow the slow drift between the two crystals. /livox/imu -> /nidar/livox/imu.
   If the sensor stamps are already within 1 s of ROS time (PTP sync, hardware.yaml
   lidar.time_sync: ptp), nothing is shifted.
"""
import collections
import struct

import rospy

# Header layout in a serialized ROS1 message: uint32 seq, uint32 secs, uint32 nsecs, string id.
STAMP_OFFSET = 4
SYNCED_TOLERANCE_S = 1.0


def read_stamp(buf):
    secs, nsecs = struct.unpack_from('<II', buf, STAMP_OFFSET)
    return secs + nsecs * 1e-9


def write_stamp(buf, t):
    """Overwrite the header stamp of a serialized message (bytearray) with time t (seconds)."""
    secs = int(t)
    nsecs = int(round((t - secs) * 1e9))
    if nsecs >= 1000000000:
        secs, nsecs = secs + 1, nsecs - 1000000000
    struct.pack_into('<II', buf, STAMP_OFFSET, secs, nsecs)


class ClockMapper(object):
    """Sensor clock -> ROS time: offset = min over the window of (arrival - sensor stamp)."""

    def __init__(self, window_s=10.0, mode='auto'):
        self.window_s = window_s
        self.mode = mode            # auto | restamp | none
        self.samples = collections.deque()
        self.offset = None
        self.active = None          # None until decided: True = shifting stamps

    def observe(self, sensor_t, arrival_t):
        d = arrival_t - sensor_t
        self.samples.append((arrival_t, d))
        while self.samples and arrival_t - self.samples[0][0] > self.window_s:
            self.samples.popleft()
        self.offset = min(s[1] for s in self.samples)
        if self.active is None:
            if self.mode == 'none':
                self.active = False
            elif self.mode == 'restamp':
                self.active = True
            else:
                self.active = abs(d) > SYNCED_TOLERANCE_S

    def to_ros(self, sensor_t):
        if not self.active or self.offset is None:
            return sensor_t
        return sensor_t + self.offset


def relabeled(type_name, md5, full_text):
    """An AnyMsg subclass carrying raw bytes under a concrete message type."""
    return type('Relabeled_' + type_name.replace('/', '_'), (rospy.AnyMsg,), {
        '_type': type_name, '_md5sum': md5, '_full_text': full_text, '_has_header': False})


class LivoxBridge(object):
    def __init__(self):
        from livox_ros_driver.msg import CustomMsg as FastLioCustomMsg
        from sensor_msgs.msg import Imu

        mode = rospy.get_param('~time_sync', 'restamp')
        self.clock = ClockMapper(window_s=float(rospy.get_param('~offset_window_s', 10.0)),
                                 mode={'ptp': 'none', 'restamp': 'auto', 'none': 'none'}.get(mode, 'auto'))
        self.cloud_cls = relabeled(FastLioCustomMsg._type, FastLioCustomMsg._md5sum,
                                   FastLioCustomMsg._full_text)
        self.imu_cls = relabeled(Imu._type, Imu._md5sum, Imu._full_text)
        self.pub_cloud = rospy.Publisher(rospy.get_param('~out_lidar', '/nidar/livox/lidar'),
                                         self.cloud_cls, queue_size=5)
        self.pub_imu = rospy.Publisher(rospy.get_param('~out_imu', '/nidar/livox/imu'),
                                       self.imu_cls, queue_size=400)
        rospy.Subscriber(rospy.get_param('~in_lidar', '/livox/lidar'), rospy.AnyMsg,
                         self.cloud_cb, queue_size=5, buff_size=2 ** 24, tcp_nodelay=True)
        rospy.Subscriber(rospy.get_param('~in_imu', '/livox/imu'), rospy.AnyMsg, self.imu_cb,
                         queue_size=400, tcp_nodelay=True)
        self.announced = False
        self.counts = {'lidar': 0, 'imu': 0}
        # 1 Hz liveness for the mission commander / GCS health (never the scans themselves).
        from std_msgs.msg import String
        self.String = String
        self.pub_status = rospy.Publisher('/nidar/livox/status', String, queue_size=1)
        self.last_status = rospy.get_time()
        rospy.Timer(rospy.Duration(1.0), self.status)
        rospy.Timer(rospy.Duration(5.0), self.report)

    def _forward(self, raw, cls, pub, learn):
        now = rospy.get_time()
        buf = bytearray(raw._buff)
        t = read_stamp(buf)
        if learn:
            self.clock.observe(t, now)
        if self.clock.active is None:
            return  # first IMU sample decides the mode; nothing to forward until then
        if self.clock.active:
            write_stamp(buf, self.clock.to_ros(t))
        out = cls()
        out._buff = bytes(buf)
        pub.publish(out)

    def imu_cb(self, raw):
        # IMU at 200 Hz carries the least-delayed samples: it drives the offset estimate.
        self.counts['imu'] += 1
        self._forward(raw, self.imu_cls, self.pub_imu, learn=True)

    def cloud_cb(self, raw):
        self.counts['lidar'] += 1
        self._forward(raw, self.cloud_cls, self.pub_cloud, learn=self.clock.active is None)

    def status(self, _evt):
        import json
        now = rospy.get_time()
        dt = max(1e-3, now - self.last_status)
        self.last_status = now
        hz = {k: round(v / dt, 1) for k, v in self.counts.items()}
        self.counts = {'lidar': 0, 'imu': 0}
        self.pub_status.publish(self.String(data=json.dumps({
            'lidar_hz': hz['lidar'], 'imu_hz': hz['imu'],
            'restamping': bool(self.clock.active),
            'clock_offset_s': None if self.clock.offset is None else round(self.clock.offset, 4)})))

    def report(self, _evt):
        if self.clock.active is None:
            rospy.logwarn_throttle(15.0, '[livox_bridge] no /livox/imu yet -- is the Mid-360 '
                                         'driver up and the LiDAR reachable (ping)?')
            return
        if not self.announced:
            self.announced = True
            if self.clock.active:
                rospy.loginfo('[livox_bridge] LiDAR clock is %.3f s off ROS time: re-stamping '
                              'scans + IMU onto ROS time (use PTP to avoid).', self.clock.offset)
            else:
                rospy.loginfo('[livox_bridge] LiDAR stamps already on ROS time (PTP): forwarding '
                              'as-is.')


if __name__ == '__main__':
    rospy.init_node('livox_bridge')
    LivoxBridge()
    rospy.spin()
