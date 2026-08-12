#!/usr/bin/env python3
"""
Minimal FUEL to MAVROS Trajectory Adapter
------------------------------------------
Responsibilities:
1. Subscribes to upstream FUEL trajectory commands (/planning/pos_cmd).
2. Converts PositionCommand setpoints directly to MAVROS PositionTarget messages.
3. Streams setpoints continuously at 20Hz for PX4 OFFBOARD mode stability.
4. Pure passthrough: passes FUEL X/Y/Z/yaw commands through to PX4.
"""

import sys
import os
if '/opt/ros/noetic/lib/python3/dist-packages' not in sys.path:
    sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
catkin_py = '/home/developer/NIDAR/catkin_ws/devel/lib/python3/dist-packages'
if os.path.exists(catkin_py) and catkin_py not in sys.path:
    sys.path.insert(0, catkin_py)

import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget, State
from geometry_msgs.msg import PoseStamped

def euler_from_quaternion(q):
    x, y, z, w = q[0], q[1], q[2], q[3]
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class MinimalFuelAdapter:
    def __init__(self):
        rospy.init_node('minimal_fuel_adapter', anonymous=False)

        self.default_altitude = rospy.get_param('~default_altitude', 1.5)

        self.current_state = None
        self.current_pose = None
        self.latest_fuel_cmd = None
        self.latest_fuel_time = rospy.Time(0)

        # Subscribers
        self.sub_state = rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        self.sub_fuel = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.fuel_cb, queue_size=10)

        # Publisher
        self.pub_target = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # 20Hz continuous setpoint timer required for OFFBOARD mode stability
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_cb)

        rospy.loginfo("[MinimalFuelAdapter] Initialized. Ready to stream FUEL setpoints to MAVROS.")

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def fuel_cb(self, msg):
        self.latest_fuel_cmd = msg
        self.latest_fuel_time = rospy.Time.now()

    def timer_cb(self, event):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        target.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        now = rospy.Time.now()
        cmd_valid = (
            self.latest_fuel_cmd is not None and
            (now - self.latest_fuel_time).to_sec() < 1.0
        )

        if cmd_valid:
            # Pass FUEL position, velocity, and yaw commands directly
            cmd = self.latest_fuel_cmd
            target.position.x = cmd.position.x
            target.position.y = cmd.position.y
            target.position.z = cmd.position.z

            target.velocity.x = cmd.velocity.x
            target.velocity.y = cmd.velocity.y
            target.velocity.z = cmd.velocity.z

            target.yaw = cmd.yaw
        else:
            # Pre-flight / Hover fallback
            if self.current_pose is not None:
                target.position.x = self.current_pose.pose.position.x
                target.position.y = self.current_pose.pose.position.y
                target.position.z = max(self.default_altitude, self.current_pose.pose.position.z)
                q = self.current_pose.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                target.yaw = yaw
            else:
                target.position.x = 0.0
                target.position.y = 0.0
                target.position.z = self.default_altitude
                target.yaw = 1.5708

        self.pub_target.publish(target)


if __name__ == '__main__':
    try:
        node = MinimalFuelAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
