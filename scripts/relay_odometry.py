#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odometry_callback(msg):
    pose_msg = PoseStamped()
    pose_msg.header = msg.header
    # MAVROS expects the frame_id to be 'map' or 'odom'
    pose_msg.header.frame_id = "map"
    pose_msg.pose = msg.pose.pose
    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('odometry_relay_node')
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    rospy.Subscriber('/Fast_LIO/odometry', Odometry, odometry_callback)
    rospy.loginfo("Relaying /Fast_LIO/odometry to /mavros/vision_pose/pose")
    rospy.spin()
