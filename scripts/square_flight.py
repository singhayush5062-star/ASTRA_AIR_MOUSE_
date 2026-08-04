#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
import time

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('square_flight_node', anonymous=True)

    rospy.Subscriber('/mavros/state', State, state_cb)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    # Rate of publishing setpoints
    rate = rospy.Rate(20)

    # Wait for FCU connection
    rospy.loginfo("Waiting for MAVROS FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("MAVROS FCU Connected!")

    # Define Square Pattern Waypoints (X, Y, Z)
    waypoints = [
        (0.0, 0.0, 1.5),  # Takeoff / Home
        (3.0, 0.0, 1.5),  # Corner 1 (3m East)
        (3.0, 3.0, 1.5),  # Corner 2 (3m North)
        (0.0, 3.0, 1.5),  # Corner 3 (3m West)
        (0.0, 0.0, 1.5),  # Return to Home
    ]

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = waypoints[0][0]
    pose.pose.position.y = waypoints[0][1]
    pose.pose.position.z = waypoints[0][2]

    # Send a few setpoints before starting offboard
    rospy.loginfo("Pre-streaming setpoints...")
    for _ in range(100):
        if rospy.is_shutdown():
            break
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    # Request OFFBOARD mode
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    # Request ARM
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    rospy.loginfo("Arming and switching to OFFBOARD mode...")
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD Mode enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed successfully!")
                last_req = rospy.Time.now()

        if current_state.armed and current_state.mode == "OFFBOARD":
            break

        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Beginning Square Waypoint Flight Sequence...")
    
    for idx, wp in enumerate(waypoints):
        rospy.loginfo(f"Executing Waypoint {idx + 1}/{len(waypoints)} -> Target: X={wp[0]}m, Y={wp[1]}m, Z={wp[2]}m")
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = wp[0]
        target_pose.pose.position.y = wp[1]
        target_pose.pose.position.z = wp[2]
        target_pose.pose.orientation.w = 1.0

        # Fly towards waypoint for 10 seconds (200 cycles @ 20Hz)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time < rospy.Duration(10.0)):
            target_pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(target_pose)
            rate.sleep()

    rospy.loginfo("Square pattern complete! Landing drone...")
    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'AUTO.LAND'
    set_mode_client.call(land_set_mode)
    rospy.loginfo("Land command sent successfully.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
