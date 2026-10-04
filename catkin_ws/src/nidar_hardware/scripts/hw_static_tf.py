#!/usr/bin/env python3
"""Static TF of the real drone, from the sensor mounts in hardware.yaml (loaded as /hardware/...).

    body -> base_link          FAST-LIO tracks `body` = the IMU it integrates. With the Mid-360's
                               own IMU (lidar.imu_source: livox) that IMU sits at
                               mount + its fixed offset inside the LiDAR; with the FC's IMU it is
                               base_link itself. (The simulation gets this link from its URDF.)
    base_link -> livox_frame   the LiDAR (raw scans, for RViz)
    base_link -> tfmini_link   the rangefinder, x along the beam (straight down)
    base_link -> camera_link   the camera, x along the optical axis (survivor_detector.py)

With world -> map -> camera_init (FUEL launch) and camera_init -> body (FAST-LIO) this completes
world -> ... -> camera_link, which the detector needs to place survivors on the arena grid.
"""
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler

MID360_IMU_IN_LIDAR = (0.011, 0.02329, -0.04412)   # same constant as apply_hardware_config.py


def mount(section):
    m = rospy.get_param('/hardware/%s/mount' % section, {}) or {}
    return {k: float(m.get(k, 0.0)) for k in ('x', 'y', 'z', 'roll', 'pitch', 'yaw')}


def tf(parent, child, xyz, rpy_deg):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    q = quaternion_from_euler(*[math.radians(a) for a in rpy_deg])
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t


def main():
    rospy.init_node('hw_static_tf')
    lm, rm, cm = mount('lidar'), mount('rangefinder'), mount('camera')
    imu_source = rospy.get_param('/hardware/lidar/imu_source', 'livox')
    out = [
        tf('base_link', 'livox_frame', (lm['x'], lm['y'], lm['z']), (lm['roll'], lm['pitch'], lm['yaw'])),
        # Range convention: x along the beam. Pitch +90 deg (ROS) turns +x to point down.
        tf('base_link', 'tfmini_link', (rm['x'], rm['y'], rm['z']), (0.0, 90.0, 0.0)),
        tf('base_link', 'camera_link', (cm['x'], cm['y'], cm['z']), (cm['roll'], cm['pitch'], cm['yaw'])),
    ]
    if imu_source == 'livox':
        if any(abs(lm[k]) > 1e-6 for k in ('roll', 'pitch', 'yaw')):
            rospy.logfatal('[hw_static_tf] imu_source livox needs lidar.mount roll/pitch/yaw = 0 '
                           '(see apply_hardware_config.py); refusing to publish a wrong body frame')
            return
        imu = [lm[k] + c for k, c in zip(('x', 'y', 'z'), MID360_IMU_IN_LIDAR)]
        out.append(tf('body', 'base_link', [-v for v in imu], (0.0, 0.0, 0.0)))
    else:
        out.append(tf('body', 'base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
    tf2_ros.StaticTransformBroadcaster().sendTransform(out)
    rospy.loginfo('[hw_static_tf] published %s', ', '.join('%s->%s' % (t.header.frame_id, t.child_frame_id)
                                                          for t in out))
    rospy.spin()


if __name__ == '__main__':
    main()
