#!/usr/bin/env python3
"""
Flight Envelope Guard Node
==========================
Project: NIDAR 2026 AirMouse

Responsibility:
- Sits between FUEL trajectory generation (/planning/pos_cmd) and MAVROS execution (/mavros/setpoint_raw/local).
- Strictly validates incoming FUEL trajectory commands against permitted 3D flight volume bounds.
- Enforces hard physical arena boundaries (X, Y) and Z altitude envelope.
- Detects boundary crossing attempts (e.g. crossing Y_MIN gate boundary).
- If VALID: passes setpoint through unchanged to PX4/MAVROS and updates last_valid_command.
- If INVALID: rejects setpoint, holds last_valid_command, and logs throttled diagnostics.
- Maintains continuous 20Hz setpoint streaming to preserve PX4 OFFBOARD state.

DO NOT add planning, trajectory optimization, A*, 2D mapping, or custom obstacle avoidance to this node.
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
from std_msgs.msg import String


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


class FlightEnvelopeGuard:
    def __init__(self):
        rospy.init_node('flight_envelope_guard', anonymous=False)

        # Load parameter namespace (support both private node params and flight_envelope_guard namespace)
        param_ns = '~'
        if not rospy.has_param('~x_min') and rospy.has_param('~flight_envelope_guard/x_min'):
            param_ns = '~flight_envelope_guard/'
        elif not rospy.has_param('~x_min') and rospy.has_param('/flight_envelope_guard/x_min'):
            param_ns = '/flight_envelope_guard/'

        self.x_min = rospy.get_param(param_ns + 'x_min', -7.0)
        self.x_max = rospy.get_param(param_ns + 'x_max', 7.0)
        self.y_min = rospy.get_param(param_ns + 'y_min', -7.0)
        self.y_max = rospy.get_param(param_ns + 'y_max', 7.0)
        self.z_min = rospy.get_param(param_ns + 'z_min', 1.45)
        self.z_max = rospy.get_param(param_ns + 'z_max', 1.55)
        self.boundary_margin = rospy.get_param(param_ns + 'boundary_margin', 0.0)
        self.publish_rate = rospy.get_param(param_ns + 'publish_rate', 20.0)
        self.default_altitude = rospy.get_param(param_ns + 'default_altitude', 1.5)

        # Calculate effective boundaries incorporating safety margin
        self.eff_x_min = self.x_min + self.boundary_margin
        self.eff_x_max = self.x_max - self.boundary_margin
        self.eff_y_min = self.y_min + self.boundary_margin
        self.eff_y_max = self.y_max - self.boundary_margin
        self.eff_z_min = self.z_min + self.boundary_margin
        self.eff_z_max = self.z_max - self.boundary_margin

        rospy.loginfo(
            f"[FlightEnvelopeGuard] Envelope Initialized:\n"
            f"  X: [{self.eff_x_min:.2f}, {self.eff_x_max:.2f}] m\n"
            f"  Y: [{self.eff_y_min:.2f}, {self.eff_y_max:.2f}] m\n"
            f"  Z: [{self.eff_z_min:.2f}, {self.eff_z_max:.2f}] m\n"
            f"  Margin: {self.boundary_margin:.2f} m | Rate: {self.publish_rate:.1f} Hz"
        )

        # State tracking
        self.current_state = None
        self.current_pose = None
        self.last_valid_command = None
        self.last_valid_pos = None  # (x, y, z) tuple
        self.last_valid_time = rospy.Time(0)

        # Diagnostics counters
        self.accepted_count = 0
        self.rejected_count = 0

        # ROS Subscribers
        self.sub_state = rospy.Subscriber('/mavros/state', State, self.state_cb, queue_size=1)
        self.sub_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb, queue_size=1)
        self.sub_fuel = rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.fuel_cb, queue_size=10)

        # ROS Publishers
        self.pub_target = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.pub_status = rospy.Publisher('/flight_envelope_guard/status', String, queue_size=10)

        # Setpoint streaming timer to maintain continuous PX4 OFFBOARD connection
        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_cb)

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def validate_command(self, cmd):
        """
        Validate position command against flight envelope and boundary crossings.
        Returns: (is_valid: bool, reason: str)
        """
        x, y, z = cmd.position.x, cmd.position.y, cmd.position.z

        # 1. Z altitude checks
        if z < self.eff_z_min:
            return False, f"Z below min (requested={z:.2f}m, allowed=[{self.eff_z_min:.2f},{self.eff_z_max:.2f}]m)"
        if z > self.eff_z_max:
            return False, f"Z above max (requested={z:.2f}m, allowed=[{self.eff_z_min:.2f},{self.eff_z_max:.2f}]m)"

        # 2. X boundary checks
        if x < self.eff_x_min:
            return False, f"X below min (requested={x:.2f}m, allowed=[{self.eff_x_min:.2f},{self.eff_x_max:.2f}]m)"
        if x > self.eff_x_max:
            return False, f"X above max (requested={x:.2f}m, allowed=[{self.eff_x_min:.2f},{self.eff_x_max:.2f}]m)"

        # 3. Y boundary checks
        if y < self.eff_y_min:
            return False, f"Y below min (requested={y:.2f}m, allowed=[{self.eff_y_min:.2f},{self.eff_y_max:.2f}]m)"
        if y > self.eff_y_max:
            return False, f"Y above max (requested={y:.2f}m, allowed=[{self.eff_y_min:.2f},{self.eff_y_max:.2f}]m)"

        # 4. Trajectory boundary crossing check relative to last valid position
        if self.last_valid_pos is not None:
            prev_x, prev_y, _ = self.last_valid_pos
            # Detect crossing South gate / Y_MIN boundary
            if prev_y >= self.eff_y_min and y < self.eff_y_min:
                return False, f"Arena boundary crossing detected (Y_MIN: prev_y={prev_y:.2f} -> curr_y={y:.2f})"
            # Detect crossing North boundary
            if prev_y <= self.eff_y_max and y > self.eff_y_max:
                return False, f"Arena boundary crossing detected (Y_MAX: prev_y={prev_y:.2f} -> curr_y={y:.2f})"
            # Detect crossing West boundary
            if prev_x >= self.eff_x_min and x < self.eff_x_min:
                return False, f"Arena boundary crossing detected (X_MIN: prev_x={prev_x:.2f} -> curr_x={x:.2f})"
            # Detect crossing East boundary
            if prev_x <= self.eff_x_max and x > self.eff_x_max:
                return False, f"Arena boundary crossing detected (X_MAX: prev_x={prev_x:.2f} -> curr_x={x:.2f})"

        return True, "ACCEPT"

    def fuel_cb(self, msg):
        is_valid, reason = self.validate_command(msg)

        if is_valid:
            self.accepted_count += 1
            # Build PositionTarget message from FUEL command
            target = PositionTarget()
            target.header.frame_id = "map"
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            target.type_mask = (
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

            target.position.x = msg.position.x
            target.position.y = msg.position.y
            target.position.z = msg.position.z

            target.velocity.x = msg.velocity.x
            target.velocity.y = msg.velocity.y
            target.velocity.z = msg.velocity.z

            target.yaw = msg.yaw

            self.last_valid_command = target
            self.last_valid_pos = (msg.position.x, msg.position.y, msg.position.z)
            self.last_valid_time = rospy.Time.now()

            rospy.loginfo_throttle(
                5.0,
                f"[FlightEnvelopeGuard] ACCEPT x={msg.position.x:.2f} y={msg.position.y:.2f} z={msg.position.z:.2f}"
            )
            self.pub_status.publish(f"ACCEPT: x={msg.position.x:.2f}, y={msg.position.y:.2f}, z={msg.position.z:.2f}")
        else:
            self.rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                f"[FlightEnvelopeGuard] REJECT: {reason} | holding last valid command"
            )
            self.pub_status.publish(f"REJECT: {reason}")

    def timer_cb(self, event):
        now = rospy.Time.now()
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        target.type_mask = (
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        has_recent_valid = (
            self.last_valid_command is not None and
            (now - self.last_valid_time).to_sec() < 2.0
        )

        if has_recent_valid:
            # Publish last valid command
            target.position = self.last_valid_command.position
            target.velocity = self.last_valid_command.velocity
            target.yaw = self.last_valid_command.yaw
        else:
            # Pre-flight / Fallback initialization behavior
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
        node = FlightEnvelopeGuard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
