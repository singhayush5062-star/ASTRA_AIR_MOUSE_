#!/usr/bin/env python3

import math
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class FlightPathPublisher:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/Fast_LIO/odometry")
        self.path_topic = rospy.get_param("~path_topic", "/flight_path")
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.max_points = int(rospy.get_param("~max_points", 10000))
        self.min_translation = float(rospy.get_param("~min_translation", 0.03))
        self.min_yaw = float(rospy.get_param("~min_yaw", 0.02))

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=5, latch=True)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.poses = deque(maxlen=max(2, self.max_points))
        self.last_pose = None

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=200)
        rospy.loginfo(
            "[flight_path_publisher] odom_topic=%s path_topic=%s frame_id=%s max_points=%d",
            self.odom_topic,
            self.path_topic,
            self.frame_id,
            self.max_points,
        )

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _angle_diff(a, b):
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return abs(d)

    def _should_append(self, pose):
        if self.last_pose is None:
            return True

        dx = pose.pose.position.x - self.last_pose.pose.position.x
        dy = pose.pose.position.y - self.last_pose.pose.position.y
        dz = pose.pose.position.z - self.last_pose.pose.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        yaw_now = self._yaw_from_quat(pose.pose.orientation)
        yaw_prev = self._yaw_from_quat(self.last_pose.pose.orientation)
        dyaw = self._angle_diff(yaw_now, yaw_prev)

        return dist >= self.min_translation or dyaw >= self.min_yaw

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        if self._should_append(pose):
            self.poses.append(pose)
            self.last_pose = pose
            self.path_msg.header.stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
            self.path_msg.header.frame_id = self.frame_id
            self.path_msg.poses = list(self.poses)
            self.path_pub.publish(self.path_msg)


def main():
    rospy.init_node("flight_path_publisher")
    FlightPathPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
