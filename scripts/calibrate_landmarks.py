#!/usr/bin/env python3
"""
Landmark Calibration & Frame Audit Script
===========================================
Measures actual coordinates published across all 4 frames:
1. Gazebo World / MAVROS local_position
2. FAST-LIO2 odometry (/Fast_LIO/odometry)
3. FUEL planner (/planning/pos_cmd)
4. Flight Envelope Guard status (/flight_envelope_guard/status)
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

latest_mavros = None
latest_fastlio = None
latest_fuel = None
latest_guard = None

def mavros_cb(msg):
    global latest_mavros
    latest_mavros = msg

def fastlio_cb(msg):
    global latest_fastlio
    latest_fastlio = msg

def fuel_cb(msg):
    global latest_fuel
    latest_fuel = msg

def guard_cb(msg):
    global latest_guard
    latest_guard = msg

def main():
    rospy.init_node('landmark_calibration_node', anonymous=True)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavros_cb)
    rospy.Subscriber('/Fast_LIO/odometry', Odometry, fastlio_cb)
    rospy.Subscriber('/planning/pos_cmd', PositionCommand, fuel_cb)
    rospy.Subscriber('/flight_envelope_guard/status', String, guard_cb)

    rate = rospy.Rate(1)
    print("=" * 70)
    print("LANDMARK COORDINATE CALIBRATION MONITOR ACTIVE")
    print("=" * 70)

    for i in range(30):
        if rospy.is_shutdown():
            break

        m_str = f"({latest_mavros.pose.position.x:.2f}, {latest_mavros.pose.position.y:.2f}, {latest_mavros.pose.position.z:.2f})" if latest_mavros else "N/A"
        f_str = f"({latest_fastlio.pose.pose.position.x:.2f}, {latest_fastlio.pose.pose.position.y:.2f}, {latest_fastlio.pose.pose.position.z:.2f})" if latest_fastlio else "N/A"
        p_str = f"({latest_fuel.position.x:.2f}, {latest_fuel.position.y:.2f}, {latest_fuel.position.z:.2f})" if latest_fuel else "N/A"
        g_str = latest_guard.data if latest_guard else "N/A"

        print(f"[t+{i*2}s] MAVROS:{m_str:22s} | FAST-LIO:{f_str:22s} | FUEL:{p_str:22s} | Guard:{g_str}")
        rate.sleep()

if __name__ == '__main__':
    main()
