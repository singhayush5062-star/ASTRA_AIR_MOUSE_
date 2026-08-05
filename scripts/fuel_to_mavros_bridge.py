#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class FuelToMavrosBridge:
    def __init__(self):
        rospy.init_node('fuel_to_mavros_bridge', anonymous=True)

        self.completed = False
        self.current_pose = None
        self.latest_pos_cmd = None

        self.sub_completed = rospy.Subscriber(
            '/exploration_completed',
            Bool,
            self.completed_callback,
            queue_size=1
        )

        self.sub_cmd = rospy.Subscriber(
            '/planning/pos_cmd', 
            PositionCommand, 
            self.pos_cmd_callback, 
            queue_size=10
        )

        self.sub_pose = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )

        self.pub_setpoint_raw = rospy.Publisher(
            '/mavros/setpoint_raw/local', 
            PositionTarget, 
            queue_size=10
        )
        
        self.pub_setpoint_pose = rospy.Publisher(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            queue_size=10
        )

        # Continuous setpoint timer running at 20Hz for rock-solid OFFBOARD stability
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        rospy.loginfo("FUEL to MAVROS Setpoint Bridge initialized with continuous 20Hz setpoint streaming.")

    def completed_callback(self, msg):
        if msg.data and not self.completed:
            rospy.loginfo("[Bridge] Exploration completed signal received. Deactivating bridge relay.")
            self.completed = True

    def pose_callback(self, msg):
        self.current_pose = msg

    def pos_cmd_callback(self, msg):
        if not self.completed:
            self.latest_pos_cmd = msg

    def timer_callback(self, event):
        if self.completed:
            return

        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        pose = PoseStamped()
        pose.header = target.header

        if self.latest_pos_cmd is not None:
            # Position from latest FUEL command
            target.position.x = self.latest_pos_cmd.position.x
            target.position.y = self.latest_pos_cmd.position.y
            target.position.z = min(1.5, max(1.0, self.latest_pos_cmd.position.z))

            # Velocity
            target.velocity.x = self.latest_pos_cmd.velocity.x
            target.velocity.y = self.latest_pos_cmd.velocity.y
            target.velocity.z = self.latest_pos_cmd.velocity.z

            # Yaw
            target.yaw = self.latest_pos_cmd.yaw

            # Type mask: Position + Yaw (Ignore velocity, acceleration, yaw_rate)
            target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

            q = quaternion_from_euler(0, 0, self.latest_pos_cmd.yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
        else:
            if self.current_pose is None:
                return

            # Initial Standby / Hover Setpoint at 1.5m
            target.position.x = self.current_pose.pose.position.x
            target.position.y = self.current_pose.pose.position.y
            target.position.z = max(1.5, self.current_pose.pose.position.z)

            q = [
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ]
            _, _, yaw = euler_from_quaternion(q)
            target.yaw = yaw

            # Type mask: Position + Yaw
            target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048
            pose.pose.orientation = self.current_pose.pose.orientation

        pose.pose.position = target.position

        self.pub_setpoint_raw.publish(target)
        self.pub_setpoint_pose.publish(pose)

if __name__ == '__main__':
    try:
        bridge = FuelToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
