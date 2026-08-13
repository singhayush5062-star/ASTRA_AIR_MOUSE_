#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, Int32
from tf.transformations import euler_from_quaternion, quaternion_from_euler

is_armed = False
origin_initialized = False
origin_x = 0.0
origin_y = 0.0
origin_z = 0.0
last_pub_x = None
last_pub_y = None
last_pub_z = None
last_pub_time = None
rejected_count = 0
healthy_streak = 0

def state_callback(msg):
    global is_armed
    is_armed = msg.armed

def odometry_callback(msg):
    global origin_initialized, origin_x, origin_y, origin_z
    global last_pub_x, last_pub_y, last_pub_z, last_pub_time
    global rejected_count, healthy_streak

    pose_msg = PoseStamped()
    # Update stamp to current ROS time to eliminate latency rejection in PX4 EKF2
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    pose_msg.pose = msg.pose.pose

    # Anchor external vision to a local origin to avoid large absolute frame offsets.
    if use_relative_origin and not origin_initialized:
        origin_x = msg.pose.pose.position.x
        origin_y = msg.pose.pose.position.y
        origin_z = msg.pose.pose.position.z
        origin_initialized = True
        rospy.loginfo("[relay_odometry] Vision origin initialized at x=%.2f y=%.2f z=%.2f", origin_x, origin_y, origin_z)

    if use_relative_origin and origin_initialized:
        pose_msg.pose.position.x = msg.pose.pose.position.x - origin_x
        pose_msg.pose.position.y = msg.pose.pose.position.y - origin_y
        pose_msg.pose.position.z = msg.pose.pose.position.z - origin_z

    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y

    # Clamp vision-z fed to EKF to prevent sudden estimator altitude jumps.
    z = pose_msg.pose.position.z
    if z < vision_z_min:
        z = vision_z_min
    elif z > vision_z_max:
        z = vision_z_max

    if last_pub_z is not None and abs(z - last_pub_z) > vision_z_jump_warn:
        rospy.logwarn_throttle(1.0, "[relay_odometry] Large vision-z jump detected: %.2f -> %.2f", last_pub_z, z)

    pose_msg.pose.position.z = z

    now = rospy.Time.now().to_sec()
    if last_pub_time is not None:
        dt = max(1e-3, now - last_pub_time)
        dx = x - last_pub_x
        dy = y - last_pub_y
        dz = z - last_pub_z
        planar_jump = (dx * dx + dy * dy) ** 0.5
        speed = planar_jump / dt

        if planar_jump > vision_xy_jump_reject:
            rospy.logwarn_throttle(1.0, "[relay_odometry] Rejecting vision sample: planar jump %.2fm exceeds %.2fm", planar_jump, vision_xy_jump_reject)
            rejected_count += 1
            healthy_streak = 0
            last_pub_x = x
            last_pub_y = y
            last_pub_z = z
            last_pub_time = now
            pub_rejected.publish(Int32(data=rejected_count))
            pub_health.publish(Bool(data=(rejected_count < rejection_threshold)))
            return

        if speed > vision_speed_reject:
            rospy.logwarn_throttle(1.0, "[relay_odometry] Rejecting vision sample: speed %.2fm/s exceeds %.2fm/s", speed, vision_speed_reject)
            rejected_count += 1
            healthy_streak = 0
            last_pub_x = x
            last_pub_y = y
            last_pub_z = z
            last_pub_time = now
            pub_rejected.publish(Int32(data=rejected_count))
            pub_health.publish(Bool(data=(rejected_count < rejection_threshold)))
            return

        if abs(dz) > vision_z_step_reject:
            rospy.logwarn_throttle(1.0, "[relay_odometry] Rejecting vision sample: z step %.2fm exceeds %.2fm", abs(dz), vision_z_step_reject)
            rejected_count += 1
            healthy_streak = 0
            last_pub_x = x
            last_pub_y = y
            last_pub_z = z
            last_pub_time = now
            pub_rejected.publish(Int32(data=rejected_count))
            pub_health.publish(Bool(data=(rejected_count < rejection_threshold)))
            return

    last_pub_x = x
    last_pub_y = y
    last_pub_z = z
    last_pub_time = now

    healthy_streak += 1
    if healthy_streak >= 5:
        rejected_count = 0
    elif rejected_count > 0:
        rejected_count -= 1

    pub_rejected.publish(Int32(data=rejected_count))
    pub_health.publish(Bool(data=(rejected_count < rejection_threshold)))

    # Level roll/pitch when on ground to eliminate Preflight Fail: Attitude failure (roll)
    if not is_armed:
        q_raw = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q_raw)
        q_clean = quaternion_from_euler(0.0, 0.0, yaw)
        pose_msg.pose.orientation.x = q_clean[0]
        pose_msg.pose.orientation.y = q_clean[1]
        pose_msg.pose.orientation.z = q_clean[2]
        pose_msg.pose.orientation.w = q_clean[3]

    pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('odometry_relay_node', anonymous=True)
    use_relative_origin = rospy.get_param('~use_relative_origin', False)
    vision_z_min = rospy.get_param('~vision_z_min', -5.0)
    vision_z_max = rospy.get_param('~vision_z_max', 10.0)
    vision_z_jump_warn = rospy.get_param('~vision_z_jump_warn', 0.6)
    vision_xy_jump_reject = rospy.get_param('~vision_xy_jump_reject', 2.5)
    vision_speed_reject = rospy.get_param('~vision_speed_reject', 6.0)
    vision_z_step_reject = rospy.get_param('~vision_z_step_reject', 1.0)
    rejection_threshold = rospy.get_param('~rejection_threshold', 15)
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
    pub_health = rospy.Publisher('/localization/healthy', Bool, queue_size=5)
    pub_rejected = rospy.Publisher('/localization/rejected_count', Int32, queue_size=5)
    rospy.Subscriber('/mavros/state', State, state_callback, queue_size=1)
    rospy.Subscriber('/Fast_LIO/odometry', Odometry, odometry_callback, queue_size=10)
    rospy.loginfo("Relaying /Fast_LIO/odometry to /mavros/vision_pose/pose with EKF2 stability enhancements (relative_origin=%s, z_clamp=[%.2f, %.2f], xy_jump<=%.2f, speed<=%.2f, z_step<=%.2f, reject_threshold=%d)",
                  str(use_relative_origin), vision_z_min, vision_z_max, vision_xy_jump_reject, vision_speed_reject, vision_z_step_reject, rejection_threshold)
    rospy.spin()
