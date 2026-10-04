#!/usr/bin/env python3
"""TFmini Plus -> /tfmini/range (sensor_msgs/Range), the topic FAST-LIO pins its altitude to and
flight_envelope_guard.py watches (range_timeout). In the simulation Gazebo publishes it; on the
real drone this node does, from wherever the sensor is wired (hardware.yaml rangefinder.source):

  fcu     TFmini on a PX4 serial port (SENS_TFMINI_CFG). PX4 sends DISTANCE_SENSOR, MAVROS
          publishes it (/mavros/distance_sensor/<name>), this node republishes it.
  serial  TFmini on the Jetson (USB-UART or 40-pin UART). This node reads the sensor's 9-byte
          frames itself: 0x59 0x59 dist_L dist_H strength_L strength_H temp_L temp_H checksum,
          distance in cm (the TFmini Plus default output).

Readings outside [range_min, range_max] or below min_strength are dropped rather than
published: FAST-LIO already rejects out-of-band ranges, but a silent sensor must look silent to
the guard's range_timeout, not like a valid altitude.
"""
import math
import struct

import rospy
from sensor_msgs.msg import Range

FRAME_LEN = 9
HEADER = b'\x59\x59'


def parse_frames(buf):
    """Pull complete TFmini frames out of buf (bytearray, consumed in place).
    Returns [(distance_m, strength)] for frames with a valid checksum."""
    out = []
    while True:
        i = buf.find(HEADER)
        if i < 0:
            del buf[:max(0, len(buf) - 1)]   # keep a trailing 0x59 that may start a header
            return out
        if i:
            del buf[:i]
        if len(buf) < FRAME_LEN:
            return out
        frame = bytes(buf[:FRAME_LEN])
        if sum(frame[:8]) & 0xFF != frame[8]:
            del buf[:1]                      # false header inside data: resync one byte on
            continue
        del buf[:FRAME_LEN]
        dist_cm, strength = struct.unpack_from('<HH', frame, 2)
        out.append((dist_cm / 100.0, strength))


def valid(distance, strength, rmin, rmax, min_strength):
    return (rmin <= distance <= rmax and strength >= min_strength and strength != 65535
            and not math.isnan(distance))


class RangefinderNode(object):
    def __init__(self):
        p = lambda k, d: rospy.get_param('/hardware/rangefinder/' + k, d)  # noqa: E731
        self.source = p('source', 'fcu')
        self.rmin = float(p('range_min', 0.1))
        self.rmax = float(p('range_max', 12.0))
        self.min_strength = int(p('min_strength', 100))
        self.frame_id = rospy.get_param('~frame_id', 'tfmini_link')
        self.pub = rospy.Publisher('/tfmini/range', Range, queue_size=10)
        self.msg = Range(radiation_type=Range.INFRARED, field_of_view=math.radians(3.6),
                         min_range=self.rmin, max_range=self.rmax)
        self.msg.header.frame_id = self.frame_id
        if self.source == 'fcu':
            topic = p('mavros_topic', '/mavros/distance_sensor/hrlv_ez4_pub')
            rospy.Subscriber(topic, Range, self.mavros_cb, queue_size=5)
            rospy.loginfo('[rangefinder] TFmini via PX4: %s -> /tfmini/range', topic)
        elif self.source == 'serial':
            self.port = p('serial_port', '/dev/ttyUSB0')
            self.baud = int(p('serial_baud', 115200))
            rospy.loginfo('[rangefinder] TFmini on %s @ %d -> /tfmini/range', self.port, self.baud)
        else:
            raise rospy.ROSInitException('hardware/rangefinder/source must be fcu or serial')

    def publish(self, distance, stamp=None):
        self.msg.header.stamp = stamp or rospy.Time.now()
        self.msg.range = distance
        self.pub.publish(self.msg)

    def mavros_cb(self, msg):
        # MAVROS has no signal strength; PX4's driver already drops weak returns.
        if self.rmin <= msg.range <= self.rmax:
            self.publish(msg.range, msg.header.stamp if msg.header.stamp.to_sec() > 0 else None)

    def run_serial(self):
        import serial
        buf = bytearray()
        dropped = 0
        while not rospy.is_shutdown():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.1) as ser:
                    rospy.loginfo('[rangefinder] %s open', self.port)
                    while not rospy.is_shutdown():
                        buf.extend(ser.read(ser.in_waiting or 1))
                        for distance, strength in parse_frames(buf):
                            if valid(distance, strength, self.rmin, self.rmax, self.min_strength):
                                self.publish(distance)
                            else:
                                dropped += 1
                                rospy.logwarn_throttle(
                                    10.0, '[rangefinder] dropping reading %.2f m strength %d '
                                          '(%d so far): out of band or weak return'
                                    % (distance, strength, dropped))
            except (OSError, ValueError) as e:  # unplugged / permissions: retry, loudly
                rospy.logerr_throttle(5.0, '[rangefinder] %s: %s -- retrying' % (self.port, e))
                rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('rangefinder')
    node = RangefinderNode()
    if node.source == 'serial':
        node.run_serial()
    else:
        rospy.spin()
