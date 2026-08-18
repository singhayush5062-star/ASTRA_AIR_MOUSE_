#!/usr/bin/env python3
"""
Mission Telemetry Logger
=========================
Project: NIDAR 2026 AirMouse

Replaces the per-iteration bash polling loop in test_takeoff.sh (which spawned ~7 throwaway
rostopic/python processes every 5s and only ever looked at drone pose + arm state). This is a
single persistent subscriber node that, every 5 seconds, prints AND appends to CSV one
correlated snapshot of both sides of the system:
  - what FUEL/traj_server is actually commanding (/planning/pos_cmd, camera_init frame)
  - what the guard last decided (/flight_envelope_guard/status)
  - what PX4's fused EKF2 estimate says (/mavros/local_position/pose) — camera_init frame,
    since relay_odometry.py relays FAST-LIO's odometry to /mavros/vision_pose/pose unmodified
    (use_relative_origin=False by default), so under healthy operation this should closely
    track raw FAST-LIO output.
  - what FAST-LIO's raw LiDAR-only odometry says (/Fast_LIO/odometry) — camera_init frame,
    NOT dependent on EKF2 fusion or MAVROS at all. This is the closest thing to ground truth
    available and is logged specifically so it can be compared against the EKF2 estimate above:
    if they diverge, PX4's position estimate has desynced from physical reality (e.g. because
    relay_odometry.py has been dropping vision samples), which is invisible if you only ever
    look at /mavros/local_position/pose.
  - arm state / flight mode (/mavros/state)

Without this, "the drone is stuck/descending" and "what FUEL asked for" are two disjoint,
unaligned streams of console output, and there was previously no way to tell a stale/diverged
EKF2 estimate apart from an actual physical hold. NOTE: pose/target columns below are in
different, unrelated frames (PX4 local EKF frame vs. FUEL's camera_init frame vs. the guard's
internal world frame) — don't compare raw values across those groups.
"""

import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import String

LANDED_STATE_NAMES = {
    ExtendedState.LANDED_STATE_UNDEFINED: 'UNDEFINED',
    ExtendedState.LANDED_STATE_ON_GROUND: 'ON_GROUND',
    ExtendedState.LANDED_STATE_IN_AIR: 'IN_AIR',
    ExtendedState.LANDED_STATE_TAKEOFF: 'TAKEOFF',
    ExtendedState.LANDED_STATE_LANDING: 'LANDING',
}


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionTelemetryLogger:
    def __init__(self):
        rospy.init_node('mission_telemetry_logger', anonymous=False)

        self.max_duration = rospy.get_param('~max_duration', 1500.0)
        self.interval = rospy.get_param('~interval', 5.0)

        log_dir = rospy.get_param('~log_dir', '/home/developer/NIDAR/logs')
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, 'mission_telemetry_%d.csv' % int(time.time()))
        self.csv_file = open(log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'wall_time', 't_elapsed_s', 'mode', 'armed', 'landed_state',
            'ekf_x', 'ekf_y', 'ekf_z',
            'vel_x', 'vel_y', 'vel_z',
            'ekf_yaw', 'fastlio_x', 'fastlio_y', 'fastlio_z',
            'ekf_fastlio_divergence_m',
            'fuel_target_xc', 'fuel_target_yc', 'fuel_target_zc', 'fuel_target_yaw',
            'fuel_target_age_s', 'fuel_target_changed',
            'guard_last_status',
        ])
        rospy.loginfo(f"[MissionTelemetryLogger] Logging to: {log_path}")

        self.state = None
        self.extended_state = None
        self.pose = None
        self.vel = None
        self.fastlio_odom = None
        self.last_fuel_cmd = None
        self.last_fuel_cmd_time = None
        self.prev_fuel_target = None
        self.last_guard_status = ''

        rospy.Subscriber('/mavros/state', State, self._state_cb, queue_size=1)
        rospy.Subscriber('/mavros/extended_state', ExtendedState, self._extended_state_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._vel_cb, queue_size=1)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self._fastlio_cb, queue_size=1)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self._fuel_cb, queue_size=10)
        rospy.Subscriber('/flight_envelope_guard/status', String, self._guard_status_cb, queue_size=10)

    def _state_cb(self, msg):
        self.state = msg

    def _extended_state_cb(self, msg):
        self.extended_state = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _vel_cb(self, msg):
        self.vel = msg

    def _fastlio_cb(self, msg):
        self.fastlio_odom = msg

    def _fuel_cb(self, msg):
        self.last_fuel_cmd = msg
        self.last_fuel_cmd_time = rospy.Time.now()

    def _guard_status_cb(self, msg):
        self.last_guard_status = msg.data

    def run(self):
        start = rospy.Time.now()
        rate = rospy.Rate(1.0 / self.interval)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()

            mode = self.state.mode if self.state else 'UNKNOWN'
            armed = self.state.armed if self.state else False
            landed_state = (LANDED_STATE_NAMES.get(self.extended_state.landed_state, 'UNKNOWN')
                            if self.extended_state else 'UNKNOWN')

            if self.pose:
                px, py, pz = (self.pose.pose.position.x, self.pose.pose.position.y,
                              self.pose.pose.position.z)
                ekf_yaw = yaw_from_quaternion(self.pose.pose.orientation)
            else:
                px = py = pz = ekf_yaw = float('nan')

            if self.vel:
                vx, vy, vz = (self.vel.twist.linear.x, self.vel.twist.linear.y,
                              self.vel.twist.linear.z)
            else:
                vx = vy = vz = float('nan')

            if self.fastlio_odom:
                lx, ly, lz = (self.fastlio_odom.pose.pose.position.x,
                              self.fastlio_odom.pose.pose.position.y,
                              self.fastlio_odom.pose.pose.position.z)
                divergence = ((px - lx) ** 2 + (py - ly) ** 2 + (pz - lz) ** 2) ** 0.5
            else:
                lx = ly = lz = float('nan')
                divergence = float('nan')

            if self.last_fuel_cmd:
                fx, fy, fz = (self.last_fuel_cmd.position.x, self.last_fuel_cmd.position.y,
                              self.last_fuel_cmd.position.z)
                fyaw = self.last_fuel_cmd.yaw
                fuel_age = (rospy.Time.now() - self.last_fuel_cmd_time).to_sec()
                cur_target = (round(fx, 2), round(fy, 2), round(fz, 2))
                target_changed = (self.prev_fuel_target is None or cur_target != self.prev_fuel_target)
                self.prev_fuel_target = cur_target
            else:
                fx = fy = fz = fyaw = float('nan')
                fuel_age = float('nan')
                target_changed = False

            diverge_flag = ' [EKF/FASTLIO DIVERGED!]' if (divergence == divergence and divergence > 0.5) else ''
            print(
                f"[Telemetry t+{elapsed:.0f}s] Landed={landed_state} "
                f"EKF(px4,camera_init)=({px:.2f},{py:.2f},{pz:.2f}) yaw={ekf_yaw:.2f} "
                f"FastLIO(raw,camera_init)=({lx:.2f},{ly:.2f},{lz:.2f}) diverge={divergence:.2f}m{diverge_flag} "
                f"Vel=({vx:.2f},{vy:.2f},{vz:.2f}) Mode={mode} Armed={armed} | "
                f"FUEL target(camera_init)=({fx:.2f},{fy:.2f},{fz:.2f}) yaw={fyaw:.2f} "
                f"age={fuel_age:.1f}s {'[CHANGED]' if target_changed else '[STATIC]'} | "
                f"Guard: {self.last_guard_status or 'n/a'}",
                flush=True,
            )

            self.csv_writer.writerow([
                rospy.Time.now().to_sec(), elapsed, mode, armed, landed_state, px, py, pz, vx, vy, vz,
                ekf_yaw, lx, ly, lz, divergence,
                fx, fy, fz, fyaw, fuel_age, target_changed, self.last_guard_status,
            ])
            self.csv_file.flush()

            if not armed and elapsed > 20.0:
                print("Drone disarmed. Telemetry logging complete.")
                break
            if elapsed > self.max_duration:
                print("Max telemetry duration reached. Stopping.")
                break

            rate.sleep()

        self.csv_file.close()


if __name__ == '__main__':
    try:
        MissionTelemetryLogger().run()
    except rospy.ROSInterruptException:
        pass
