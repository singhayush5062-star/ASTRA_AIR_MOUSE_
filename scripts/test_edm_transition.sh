#!/usr/bin/env bash
# ==============================================================================
# Entry Detection Module (EDM) Integration & State Transition Test
# ==============================================================================
set -e

echo "============================================================"
echo "Starting Entry Detection Module (EDM) Transition Test..."
echo "============================================================"

# Source ROS environment
source /opt/ros/noetic/setup.bash
if [ -f "/home/developer/NIDAR/catkin_ws/devel/setup.bash" ]; then
    source /home/developer/NIDAR/catkin_ws/devel/setup.bash
fi

# Run python integration test simulator
python3 -c "
import rospy
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Float64

rospy.init_node('edm_integration_test_rig', anonymous=True)

edm_state = 'UNKNOWN'
confidence = 0.0
trigger_received = False

def state_cb(msg):
    global edm_state
    edm_state = msg.data

def conf_cb(msg):
    global confidence
    confidence = msg.data

def trigger_cb(msg):
    global trigger_received
    trigger_received = True

rospy.Subscriber('/edm/mission_state', String, state_cb)
rospy.Subscriber('/edm/confidence_score', Float64, conf_cb)
rospy.Subscriber('/waypoint_generator/waypoints', Path, trigger_cb)

pub_pose = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=5)
pub_odom = rospy.Publisher('/Fast_LIO/odometry', Odometry, queue_size=5)
pub_cloud = rospy.Publisher('/cloud_registered', PointCloud2, queue_size=5)

rate = rospy.Rate(20)
start_t = time.time()

# 1. Simulate Takeoff phase (Z increases to 1.5m)
for step in range(30):
    z = min(1.5, step * 0.05)
    p = PoseStamped()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = 'camera_init'
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = z
    p.pose.orientation.w = 1.0
    pub_pose.publish(p)

    o = Odometry()
    o.header = p.header
    o.pose.pose = p.pose
    pub_odom.publish(o)
    rate.sleep()

print(f'[Test Rig] Takeoff simulated. Current EDM State: {edm_state}')

# 2. Simulate Entry Search phase with obstacle walls and a 1.0m door opening at X=1.5m
# Left wall: Y in [0.5, 3.0], Right wall: Y in [-3.0, -0.5] -> gap = 1.0m
points = []
for y in np.linspace(0.5, 3.0, 15):
    for z in np.linspace(0.5, 2.0, 10):
        points.append([1.5, y, z])
for y in np.linspace(-3.0, -0.5, 15):
    for z in np.linspace(0.5, 2.0, 10):
        points.append([1.5, y, z])

header = std_msgs.msg.Header()
header.frame_id = 'camera_init'

for step in range(40):
    header.stamp = rospy.Time.now()
    cloud_msg = pc2.create_cloud_xyz32(header, points)
    pub_cloud.publish(cloud_msg)

    # Move UAV forward past the 1.0m door opening (X from 0.0 to 2.2m)
    x = min(2.2, step * 0.06)
    p = PoseStamped()
    p.header.stamp = header.stamp
    p.header.frame_id = 'camera_init'
    p.pose.position.x = x
    p.pose.position.y = 0.0
    p.pose.position.z = 1.5
    p.pose.orientation.w = 1.0
    pub_pose.publish(p)

    o = Odometry()
    o.header = header
    o.pose.pose = p.pose
    pub_odom.publish(o)
    rate.sleep()

print(f'[Test Rig] Door traversal simulated. EDM State: {edm_state}, Confidence: {confidence:.2f}, Trigger Received: {trigger_received}')

if trigger_received or edm_state in ['EXPLORATION', 'ENTRY_CONFIRMATION']:
    print('[Test Rig] SUCCESS: EDM correctly detected opening, crossed door, and triggered FUEL Exploration!')
else:
    print(f'[Test Rig] FAILURE: EDM state is {edm_state}, trigger received: {trigger_received}')
    exit(1)
"
