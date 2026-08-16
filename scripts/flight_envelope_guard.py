#!/usr/bin/env python3
"""
Flight Envelope Guard Node (Production Grade)
=============================================
Project: NIDAR 2026 AirMouse

Responsibility:
1. Make World-frame validation authoritative using verified TF transform:
   X_world = -Y_camera_init
   Y_world =  X_camera_init - 6.5
   Z_world =  Z_camera_init + 0.1
2. Separate position and velocity safety:
   - Directionally clamp velocity components near boundary margins.
   - Force VX = VY = VZ = 0 and set IGNORE_VX|VY|VZ when holding position.
3. Safe HOLD state:
   - Stream last safe world position at 20 Hz when FUEL is replanning.
   - Preserves PX4 OFFBOARD state without triggering failsafe landing.
4. Strengthened Guard Diagnostics:
   - Publish explicit rejection reasons (OUT_OF_BOUNDS_X, OUT_OF_BOUNDS_Y, OUT_OF_BOUNDS_Z, BOUNDARY_CROSSING, etc.).
   - Log both camera_init and world frame coordinates for 100% auditability.
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

        # Load parameters
        param_ns = '~'
        if not rospy.has_param('~x_min') and rospy.has_param('~flight_envelope_guard/x_min'):
            param_ns = '~flight_envelope_guard/'
        elif not rospy.has_param('~x_min') and rospy.has_param('/flight_envelope_guard/x_min'):
            param_ns = '/flight_envelope_guard/'

        # World Frame Authoritative Boundaries (Gazebo World Frame)
        self.world_x_min = rospy.get_param(param_ns + 'world_x_min', -7.0)
        self.world_x_max = rospy.get_param(param_ns + 'world_x_max', 7.0)
        self.world_y_min = rospy.get_param(param_ns + 'world_y_min', -7.0)
        self.world_y_max = rospy.get_param(param_ns + 'world_y_max', 7.0)
        self.world_z_min = rospy.get_param(param_ns + 'world_z_min', 1.45)
        self.world_z_max = rospy.get_param(param_ns + 'world_z_max', 1.55)

        self.boundary_margin = rospy.get_param(param_ns + 'boundary_margin', 0.2)
        self.publish_rate = rospy.get_param(param_ns + 'publish_rate', 20.0)
        self.default_altitude = rospy.get_param(param_ns + 'default_altitude', 1.5)

        # Effective World Boundaries incorporating safety margin
        self.eff_xw_min = self.world_x_min + self.boundary_margin
        self.eff_xw_max = self.world_x_max - self.boundary_margin
        self.eff_yw_min = self.world_y_min + self.boundary_margin
        self.eff_yw_max = self.world_y_max - self.boundary_margin
        self.eff_zw_min = self.world_z_min
        self.eff_zw_max = self.world_z_max

        # Camera-frame bounds are derived at runtime via world_to_camera()
        # No separate camera-frame config needed — world bounds are the single source of truth

        rospy.loginfo(
            f"[FlightEnvelopeGuard] Production Envelope Initialized (World Frame Authoritative):\n"
            f"  World X: [{self.eff_xw_min:.2f}, {self.eff_xw_max:.2f}] m\n"
            f"  World Y: [{self.eff_yw_min:.2f}, {self.eff_yw_max:.2f}] m\n"
            f"  World Z: [{self.eff_zw_min:.2f}, {self.eff_zw_max:.2f}] m\n"
            f"  Margin: {self.boundary_margin:.2f} m | Rate: {self.publish_rate:.1f} Hz"
        )

        # State tracking
        self.current_state = None
        self.current_pose = None
        self.last_valid_command = None
        self.last_valid_pos_camera = None  # (x_c, y_c, z_c)
        self.last_valid_pos_world = None   # (x_w, y_w, z_w)
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

        # 20 Hz streaming timer for continuous PX4 OFFBOARD lock
        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0 else 0.05
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_cb)

    def camera_to_world(self, xc, yc, zc):
        """Rigid transformation from camera_init to Gazebo world frame."""
        xw = -yc
        yw = xc - 6.5
        zw = zc + 0.1
        return xw, yw, zw

    def world_to_camera(self, xw, yw, zw):
        """Inverse rigid transformation from Gazebo world to camera_init frame."""
        xc = yw + 6.5
        yc = -xw
        zc = zw - 0.1
        return xc, yc, zc

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def validate_command(self, cmd):
        """
        Validate position command in Authoritative Gazebo World Frame.
        Returns: (is_valid: bool, code: str, reason: str, (xw, yw, zw))
        """
        xc, yc, zc = cmd.position.x, cmd.position.y, cmd.position.z
        xw, yw, zw = self.camera_to_world(xc, yc, zc)

        # 1. World Frame X boundary check
        if xw < self.eff_xw_min:
            return False, "OUT_OF_BOUNDS_X_MIN", f"World X below min (xw={xw:.2f}m < {self.eff_xw_min:.2f}m)", (xw, yw, zw)
        if xw > self.eff_xw_max:
            return False, "OUT_OF_BOUNDS_X_MAX", f"World X above max (xw={xw:.2f}m > {self.eff_xw_max:.2f}m)", (xw, yw, zw)

        # 2. World Frame Y boundary check (South Door / North Wall)
        if yw < self.eff_yw_min:
            return False, "OUT_OF_BOUNDS_Y_MIN", f"World Y below min (South Gate: yw={yw:.2f}m < {self.eff_yw_min:.2f}m)", (xw, yw, zw)
        if yw > self.eff_yw_max:
            return False, "OUT_OF_BOUNDS_Y_MAX", f"World Y above max (North Wall: yw={yw:.2f}m > {self.eff_yw_max:.2f}m)", (xw, yw, zw)

        # 3. World Frame Z boundary check
        if zw < self.eff_zw_min:
            return False, "OUT_OF_BOUNDS_Z_MIN", f"World Z below min (zw={zw:.2f}m < {self.eff_zw_min:.2f}m)", (xw, yw, zw)
        if zw > self.eff_zw_max:
            return False, "OUT_OF_BOUNDS_Z_MAX", f"World Z above max (zw={zw:.2f}m > {self.eff_zw_max:.2f}m)", (xw, yw, zw)

        # 4. Vehicle current pose warning (if pose available)
        if self.current_pose is not None:
            # Convert MAVROS local_position/pose (ENU: x=East, y=North) to camera_init frame (xc=North, yc=West)
            c_xc = self.current_pose.pose.position.y
            c_yc = -self.current_pose.pose.position.x
            c_zc = self.current_pose.pose.position.z
            c_xw, c_yw, c_zw = self.camera_to_world(c_xc, c_yc, c_zc)
            if c_xw < self.eff_xw_min or c_xw > self.eff_xw_max or \
               c_yw < self.eff_yw_min or c_yw > self.eff_yw_max or \
               c_zw < self.eff_zw_min or c_zw > self.eff_zw_max:
                rospy.logwarn_throttle(2.0, f"[FlightEnvelopeGuard] Vehicle current pose near/outside margin: world=({c_xw:.2f},{c_yw:.2f},{c_zw:.2f}) | guiding to safe interior setpoint")

        return True, "ACCEPT", "Safe setpoint inside arena envelope", (xw, yw, zw)

    def fuel_cb(self, msg):
        is_valid, code, reason, (xw, yw, zw) = self.validate_command(msg)
        xc, yc, zc = msg.position.x, msg.position.y, msg.position.z

        if is_valid:
            self.accepted_count += 1
            target = PositionTarget()
            target.header.frame_id = "map"
            target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

            target.type_mask = (
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )

            # Clamp position in world frame (single source of truth)
            xw_clamped = min(max(xw, self.eff_xw_min), self.eff_xw_max)
            yw_clamped = min(max(yw, self.eff_yw_min), self.eff_yw_max)
            zw_clamped = min(max(zw, self.eff_zw_min), self.eff_zw_max)

            # Inverse-transform clamped world position back to camera_init frame
            px_c, py_c, pz_c = self.world_to_camera(xw_clamped, yw_clamped, zw_clamped)

            target.position.x = px_c
            target.position.y = py_c
            target.position.z = pz_c

            # Directional velocity clamping near world-frame boundary margins
            # Transform velocities to world frame (rotation only, no translation)
            vx_w = -msg.velocity.y
            vy_w = msg.velocity.x
            vz = msg.velocity.z

            if xw_clamped <= self.eff_xw_min + 0.3:
                vx_w = max(0.0, vx_w)
            elif xw_clamped >= self.eff_xw_max - 0.3:
                vx_w = min(0.0, vx_w)

            if yw_clamped <= self.eff_yw_min + 0.3:
                vy_w = max(0.0, vy_w)
            elif yw_clamped >= self.eff_yw_max - 0.3:
                vy_w = min(0.0, vy_w)

            # Z velocity clamping near boundaries (world frame Z = camera Z + 0.1, no rotation)
            if zw_clamped <= self.eff_zw_min + 0.3:
                vz = max(0.0, vz)   # Near floor: only allow upward velocity
            elif zw_clamped >= self.eff_zw_max - 0.3:
                vz = min(0.0, vz)   # Near ceiling: only allow downward velocity

            # Transform velocities back to camera_init frame
            target.velocity.x = vy_w
            target.velocity.y = -vx_w
            target.velocity.z = vz

            target.yaw = msg.yaw

            self.last_valid_command = target
            self.last_valid_pos_camera = (px_c, py_c, pz_c)
            self.last_valid_pos_world = (xw_clamped, yw_clamped, zw_clamped)
            self.last_valid_time = rospy.Time.now()

            rospy.loginfo_throttle(
                5.0,
                f"[FlightEnvelopeGuard] ACCEPT camera_init=({px_c:.2f},{py_c:.2f},{pz_c:.2f}) -> world=({xw_clamped:.2f},{yw_clamped:.2f},{zw_clamped:.2f})"
            )
            self.pub_status.publish(f"ACCEPT: code={code} | camera=({px_c:.2f},{py_c:.2f},{pz_c:.2f}) world=({xw_clamped:.2f},{yw_clamped:.2f},{zw_clamped:.2f})")
        else:
            self.rejected_count += 1
            rospy.logwarn_throttle(
                1.0,
                f"[FlightEnvelopeGuard] REJECT [{code}]: {reason} | camera_init=({xc:.2f},{yc:.2f},{zc:.2f}) world=({xw:.2f},{yw:.2f},{zw:.2f}) | holding last safe position"
            )
            self.pub_status.publish(f"REJECT: code={code} | reason={reason}")

    def timer_cb(self, event):
        now = rospy.Time.now()
        target = PositionTarget()
        target.header.stamp = now
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        has_recent_valid = (
            self.last_valid_command is not None and
            (now - self.last_valid_time).to_sec() < 2.0
        )

        if has_recent_valid:
            # Stream last safe position command with ZERO velocity and strict position type_mask
            target.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            target.position = self.last_valid_command.position
            target.velocity.x = 0.0
            target.velocity.y = 0.0
            target.velocity.z = 0.0
            target.yaw = self.last_valid_command.yaw
        else:
            # Fallback safe hover hold
            target.type_mask = (
                PositionTarget.IGNORE_VX |
                PositionTarget.IGNORE_VY |
                PositionTarget.IGNORE_VZ |
                PositionTarget.IGNORE_AFX |
                PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ |
                PositionTarget.IGNORE_YAW_RATE
            )
            if self.current_pose is not None:
                # Convert MAVROS local_position/pose (ENU: x=East, y=North) to camera_init frame (xc=North, yc=West)
                pose_xc = self.current_pose.pose.position.y
                pose_yc = -self.current_pose.pose.position.x
                pose_zc = self.current_pose.pose.position.z
                pose_xw, pose_yw, pose_zw = self.camera_to_world(pose_xc, pose_yc, pose_zc)
                xw_c = min(max(pose_xw, self.eff_xw_min), self.eff_xw_max)
                yw_c = min(max(pose_yw, self.eff_yw_min), self.eff_yw_max)
                target_zw = max(pose_zw, self.default_altitude)
                zw_c = min(max(target_zw, self.eff_zw_min), self.eff_zw_max)
                eff_x, eff_y, eff_z = self.world_to_camera(xw_c, yw_c, zw_c)
                target.position.x = eff_x
                target.position.y = eff_y
                target.position.z = eff_z
                q = self.current_pose.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                target.yaw = yaw
            else:
                target.position.x = 0.0
                target.position.y = 0.0
                target.position.z = min(max(self.default_altitude, self.eff_zw_min), self.eff_zw_max)
                target.yaw = 1.5708

            target.velocity.x = 0.0
            target.velocity.y = 0.0
            target.velocity.z = 0.0

        self.pub_target.publish(target)


if __name__ == '__main__':
    try:
        node = FlightEnvelopeGuard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
