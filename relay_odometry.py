#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

def odometry_callback(msg):
    pose_msg = PoseStamped()
    # Update stamp to current ROS time to prevent PX4 EKF2 latency rejection
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"
    pose_msg.pose = msg.pose.pose
    pub_pose.publish(pose_msg)

    # Optional Covariance Pose
    cov_msg = PoseWithCovarianceStamped()
    cov_msg.header = pose_msg.header
    cov_msg.pose = msg.pose
    pub_cov.publish(cov_msg)

if __name__ == '__main__':
    rospy.init_node('odometry_relay_node', anonymous=True)
    pub_pose = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    pub_cov = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=10)
    rospy.Subscriber('/Fast_LIO/odometry', Odometry, odometry_callback, queue_size=10)
    rospy.loginfo("Relaying /Fast_LIO/odometry to /mavros/vision_pose/pose at high frequency")
    rospy.spin()
