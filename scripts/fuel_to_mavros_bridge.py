#!/usr/bin/env python3
"""
FUEL to MAVROS Offboard Control Bridge with Velocity Capping
============================================================
Translates B-spline trajectory commands from FUEL's exploration planner
into MAVROS PositionTarget setpoints for PX4 offboard flight.
Enforces a maximum Euclidean velocity cap of 1.0 m/s for warehouse exploration safety.
"""

import rospy
import math
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool

try:
    from quadrotor_msgs.msg import PositionCommand
except ImportError:
    rospy.logwarn("quadrotor_msgs not in PYTHONPATH; FUEL message import will be re-attempted at runtime.")
    PositionCommand = None


class FuelMavrosBridge:
    def __init__(self):
        rospy.init_node("fuel_to_mavros_bridge", anonymous=True)
        
        # Operational limits & safety constraints
        self.max_velocity = 1.0  # m/s velocity cap as specified in Phase 4 plan
        self.rate_hz = 30.0      # 30Hz offboard heartbeat
        
        # State tracking
        self.current_state = State()
        self.latest_odom = None
        self.latest_cmd = None
        self.cmd_received = False
        
        # ROS Subscribers
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.odom_sub = rospy.Subscriber("/Fast_LIO/odometry", Odometry, self.odom_cb)
        
        if PositionCommand is not None:
            self.cmd_sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_cb)
        else:
            # Fallback dynamic subscription if module was loaded post-init
            from quadrotor_msgs.msg import PositionCommand as PosCmd
            self.cmd_sub = rospy.Subscriber("/planning/pos_cmd", PosCmd, self.cmd_cb)
            
        # ROS Publishers & Services
        self.target_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        
        rospy.loginfo("[FUEL Bridge] Initialized with v_max = %.1f m/s and heartbeat = %.1f Hz", 
                      self.max_velocity, self.rate_hz)
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_setpoint)

    def state_cb(self, msg):
        self.current_state = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def cmd_cb(self, msg):
        self.latest_cmd = msg
        self.cmd_received = True

    def publish_setpoint(self, event):
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        if not self.cmd_received or self.latest_cmd is None:
            # If no FUEL command yet, maintain hover at current position if known
            if self.latest_odom is not None:
                target.position = self.latest_odom.pose.pose.position
                target.yaw = 0.0
                # Ignore velocity, acceleration, and yaw rate (control position only)
                target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                                    PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                                    PositionTarget.IGNORE_YAW_RATE)
            else:
                return
        else:
            # We have an active trajectory setpoint from FUEL
            cmd = self.latest_cmd
            target.position = cmd.position
            target.yaw = cmd.yaw
            
            # Extract velocities
            vx = cmd.velocity.x
            vy = cmd.velocity.y
            vz = cmd.velocity.z
            
            # Enforce 1.0 m/s velocity cap
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)
            if speed > self.max_velocity and speed > 0:
                scale = self.max_velocity / speed
                vx *= scale
                vy *= scale
                vz *= scale
                rospy.logdebug_throttle(2.0, "[FUEL Bridge] Velocity %.2f m/s capped to %.2f m/s", speed, self.max_velocity)
                
            target.velocity = Vector3(vx, vy, vz)
            
            # Type mask: Ignore acceleration and yaw rate; feed position, velocity, and yaw
            target.type_mask = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                                PositionTarget.IGNORE_YAW_RATE)

        self.target_pub.publish(target)


if __name__ == "__main__":
    try:
        bridge = FuelMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
