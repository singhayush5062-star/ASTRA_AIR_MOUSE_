#!/usr/bin/env python3
# pyrefly: ignore [missing-import]
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest

current_state = State()
current_pose = PoseStamped()
pose_received = False

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose, pose_received
    current_pose = msg
    pose_received = True

def get_distance(p1, p2):
    dx = p1.pose.position.x - p2[0]
    dy = p1.pose.position.y - p2[1]
    dz = p1.pose.position.z - p2[2]
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def main():
    rospy.init_node('robust_square_flight_node', anonymous=True)

    rospy.Subscriber('/mavros/state', State, state_cb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_cb)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.wait_for_service('/mavros/cmd/arming')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    rospy.wait_for_service('/mavros/set_mode')
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    rate = rospy.Rate(20)

    rospy.loginfo("1. Waiting for MAVROS FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("MAVROS FCU Connected!")

    rospy.loginfo("2. Waiting for Local Position / EKF2 Lock...")
    while not rospy.is_shutdown() and not pose_received:
        rate.sleep()
    rospy.loginfo("Local Position Lock Confirmed!")

    home_x = current_pose.pose.position.x
    home_y = current_pose.pose.position.y
    target_z = 1.6

    # Clear, collision-free square trajectory inside warehouse open aisle
    waypoints = [
        (home_x + 0.0, home_y + 0.0, target_z),
        (home_x + 0.0, home_y + 2.2, target_z),
        (home_x - 2.0, home_y + 2.2, target_z),
        (home_x - 2.0, home_y + 0.0, target_z),
        (home_x + 0.0, home_y + 0.0, target_z),
    ]

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = waypoints[0][0]
    pose.pose.position.y = waypoints[0][1]
    pose.pose.position.z = waypoints[0][2]
    pose.pose.orientation.w = 1.0

    rospy.loginfo("3. Pre-streaming setpoints at 20Hz (required by PX4 for OFFBOARD)...")
    for _ in range(50):
        if rospy.is_shutdown():
            break
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_arm_req = rospy.Time.now()
    last_offb_req = rospy.Time.now()

    rospy.loginfo("4. Requesting Arming & OFFBOARD Mode...")
    while not rospy.is_shutdown():
        now = rospy.Time.now()

        if not current_state.armed and (now - last_arm_req > rospy.Duration(1.5)):
            res = arming_client.call(arm_cmd)
            if res.success:
                rospy.loginfo("Vehicle ARMED successfully!")
            last_arm_req = now

        if current_state.mode != "OFFBOARD" and (now - last_offb_req > rospy.Duration(1.5)):
            res = set_mode_client.call(offb_set_mode)
            if res.mode_sent:
                rospy.loginfo("OFFBOARD Mode requested.")
            last_offb_req = now

        if current_state.armed and current_state.mode == "OFFBOARD":
            rospy.loginfo(">>> Vehicle ARMED and in OFFBOARD mode! Starting flight! <<<")
            break

        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("5. Executing Collision-Free Open Aisle Square Trajectory...")
    for idx, wp in enumerate(waypoints):
        rospy.loginfo(f"--> Heading to Waypoint {idx+1}/{len(waypoints)}: Target=(X={wp[0]:.2f}m, Y={wp[1]:.2f}m, Z={wp[2]:.2f}m)")
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = wp[0]
        target_pose.pose.position.y = wp[1]
        target_pose.pose.position.z = wp[2]
        target_pose.pose.orientation.w = 1.0

        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            target_pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(target_pose)

            dist = get_distance(current_pose, wp)
            cp = current_pose.pose.position
            
            if dist < 0.45:
                rospy.loginfo(f"    [REACHED WP {idx+1}] Current Pos: (X={cp.x:.2f}, Y={cp.y:.2f}, Z={cp.z:.2f}) | Error: {dist:.2f}m")
                break

            if rospy.Time.now() - start_time > rospy.Duration(20.0):
                rospy.logwarn(f"    [TIMEOUT WP {idx+1}] Current Pos: (X={cp.x:.2f}, Y={cp.y:.2f}, Z={cp.z:.2f}) | Dist: {dist:.2f}m")
                break

            rate.sleep()

        hover_end = rospy.Time.now() + rospy.Duration(1.5)
        while not rospy.is_shutdown() and rospy.Time.now() < hover_end:
            target_pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(target_pose)
            rate.sleep()

    rospy.loginfo("6. Square Trajectory Complete! Landing Vehicle...")
    land_set_mode = SetModeRequest()
    land_set_mode.custom_mode = 'AUTO.LAND'
    set_mode_client.call(land_set_mode)
    rospy.loginfo("AUTO.LAND Mode Sent. Mission Completed Successfully!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
