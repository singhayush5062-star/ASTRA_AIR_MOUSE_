#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

class FuelToMavrosBridge:
    def __init__(self):
        rospy.init_node('fuel_to_mavros_bridge', anonymous=True)

        self.sub_cmd = rospy.Subscriber(
            '/planning/pos_cmd', 
            PositionCommand, 
            self.pos_cmd_callback, 
            queue_size=10
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

        rospy.loginfo("FUEL to MAVROS Setpoint Bridge initialized.")

    def pos_cmd_callback(self, msg):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Position
        target.position.x = msg.position.x
        target.position.y = msg.position.y
        target.position.z = msg.position.z

        # Velocity
        target.velocity.x = msg.velocity.x
        target.velocity.y = msg.velocity.y
        target.velocity.z = msg.velocity.z

        # Yaw
        target.yaw = msg.yaw

        # Type mask: Ignore acceleration/force, use position + velocity + yaw
        # Bitmask breakdown:
        # 1: Ignore px, 2: Ignore py, 4: Ignore pz
        # 8: Ignore vx, 16: Ignore vy, 32: Ignore vz
        # 64: Ignore ax, 128: Ignore ay, 256: Ignore az
        # 512: Use force, 1024: Ignore yaw, 2048: Ignore yaw_rate
        target.type_mask = 64 + 128 + 256  # Ignore acceleration only (use pos, vel, yaw)

        self.pub_setpoint_raw.publish(target)

        # Also publish simple PoseStamped setpoint for compatibility
        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position = target.position
        # Yaw to Quaternion conversion
        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, msg.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pub_setpoint_pose.publish(pose)

if __name__ == '__main__':
    try:
        bridge = FuelToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
