#!/usr/bin/env python3
"""
Entry Detection Module (EDM) for NIDAR UAV RescueSwarm
------------------------------------------------------
Perception-driven module operating before FUEL exploration to detect when the
UAV has successfully entered an unknown indoor arena through an opening (nominal 1m door).

Mission States:
- TAKEOFF
- ENTRY_SEARCH
- ENTRY_CONFIRMATION
- EXPLORATION (FUEL)
- RETURN
- LAND
"""

import math
import json
import time
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, Point, Vector3
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Float64, Bool
from nav_msgs.msg import Path, Odometry
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MissionState:
    TAKEOFF = "TAKEOFF"
    ENTRY_SEARCH = "ENTRY_SEARCH"
    ENTRY_CONFIRMATION = "ENTRY_CONFIRMATION"
    EXPLORATION = "EXPLORATION"
    RETURN = "RETURN"
    LAND = "LAND"


class MultiCueEntryDetector:
    """
    Evaluates 5 perception-driven cues for indoor entry:
    - Cue 1: Opening Crossed (Highest priority / mandatory, door width 0.5m-1.2m)
    - Cue 2: Increased Free Space
    - Cue 3: Stable Localization (Mandatory)
    - Cue 4: Adequate Obstacle Clearance (Mandatory)
    - Cue 5: Increased Obstacle Density
    """

    def __init__(self):
        # Configurable parameters
        self.door_min_width = rospy.get_param('~door_min_width', 0.50)
        self.door_max_width = rospy.get_param('~door_max_width', 1.20)
        self.target_door_width = rospy.get_param('~target_door_width', 1.00)
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.70)
        self.stability_required_cycles = rospy.get_param('~stability_required_cycles', 10)
        self.min_clearance = rospy.get_param('~min_clearance', 0.45)
        self.target_clearance = rospy.get_param('~target_clearance', 0.60)

        # Multi-cue weights
        self.w1 = 0.35  # Opening Crossed
        self.w2 = 0.20  # Increased Free Space
        self.w3 = 0.15  # Stable Localization
        self.w4 = 0.15  # Obstacle Clearance
        self.w5 = 0.15  # Obstacle Density

        # Internal state metrics
        self.opening_detected = False
        self.opening_crossed = False
        self.opening_behind = False
        self.detected_opening_pos = None  # (x, y, z) in world frame
        self.detected_opening_width = 0.0

        self.baseline_free_space = None
        self.current_free_space = 0.0
        self.free_space_ratio = 1.0

        self.baseline_obstacle_density = None
        self.current_obstacle_density = 0.0
        self.obstacle_density_ratio = 1.0

        self.last_odom_time = None
        self.odom_rate_hz = 0.0
        self.last_pose = None
        self.pose_jump_max = 0.0
        self.localization_healthy = True

        self.min_obstacle_dist = 2.0  # meters

        self.cue_scores = {
            'cue1_opening_crossed': 0.0,
            'cue2_free_space': 0.0,
            'cue3_stable_localization': 1.0,
            'cue4_obstacle_clearance': 1.0,
            'cue5_obstacle_density': 0.0
        }

        self.confidence_score = 0.0
        self.consecutive_stable_cycles = 0

    def update_odometry(self, odom_msg):
        now = odom_msg.header.stamp.to_sec()
        if self.last_odom_time is not None and now > self.last_odom_time:
            dt = now - self.last_odom_time
            inst_rate = 1.0 / dt if dt > 0 else 0.0
            self.odom_rate_hz = 0.8 * self.odom_rate_hz + 0.2 * inst_rate if self.odom_rate_hz > 0 else inst_rate

        self.last_odom_time = now

        curr_p = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])

        if self.last_pose is not None:
            step = np.linalg.norm(curr_p - self.last_pose)
            self.pose_jump_max = max(self.pose_jump_max * 0.95, step)

        self.last_pose = curr_p

    def process_point_cloud(self, cloud_msg, uav_pos, uav_yaw):
        """
        Processes LiDAR point cloud in body frame to evaluate:
        1. Opening/Door detection (gap of ~1.0m width)
        2. Traversal/Crossing verification
        3. Free space & obstacle density metrics
        4. ESDF / Obstacle clearance metric
        """
        if uav_pos is None or uav_yaw is None:
            return

        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        if not points:
            return

        pts = np.array(points)

        # Transform to body frame relative to UAV pose
        dx = pts[:, 0] - uav_pos[0]
        dy = pts[:, 1] - uav_pos[1]
        dz = pts[:, 2] - uav_pos[2]

        cos_y = math.cos(-uav_yaw)
        sin_y = math.sin(-uav_yaw)

        x_body = dx * cos_y - dy * sin_y
        y_body = dx * sin_y + dy * cos_y
        z_body = dz

        # Filter height band around drone cruising altitude (z_body in [-0.8, 0.8])
        band_mask = (np.abs(z_body) <= 0.8)
        x_band = x_body[band_mask]
        y_band = y_body[band_mask]
        dist_3d = np.linalg.norm(pts - uav_pos, axis=1)

        # --- Cue 4: Obstacle Clearance ---
        if len(dist_3d) > 0:
            self.min_obstacle_dist = float(np.min(dist_3d))
        else:
            self.min_obstacle_dist = 2.0

        if self.min_obstacle_dist >= self.target_clearance:
            self.cue_scores['cue4_obstacle_clearance'] = 1.0
        elif self.min_obstacle_dist <= self.min_clearance:
            self.cue_scores['cue4_obstacle_clearance'] = 0.0
        else:
            self.cue_scores['cue4_obstacle_clearance'] = (self.min_obstacle_dist - self.min_clearance) / (self.target_clearance - self.min_clearance)

        # --- Cue 1: Opening Detection & Traversal ---
        # Look ahead in body frame: x in [0.3, 4.0]m, y in [-2.5, 2.5]m
        fwd_mask = (x_band >= 0.3) & (x_band <= 4.0) & (np.abs(y_band) <= 2.5)
        x_fwd = x_band[fwd_mask]
        y_fwd = y_band[fwd_mask]

        if len(y_fwd) > 10:
            # Separate into left (y > 0.15) and right (y < -0.15) obstacle boundaries
            left_pts = y_fwd[y_fwd > 0.15]
            right_pts = y_fwd[y_fwd < -0.15]

            if len(left_pts) > 3 and len(right_pts) > 3:
                left_edge = float(np.min(left_pts))
                right_edge = float(np.max(right_pts))
                gap_width = left_edge - right_edge

                if self.door_min_width <= gap_width <= self.door_max_width:
                    self.opening_detected = True
                    self.detected_opening_width = gap_width
                    gap_center_y_body = (left_edge + right_edge) / 2.0
                    gap_dist_x_body = float(np.mean(x_fwd))

                    # Calculate world coordinates of detected opening
                    cos_w = math.cos(uav_yaw)
                    sin_w = math.sin(uav_yaw)
                    gate_x_world = uav_pos[0] + gap_dist_x_body * cos_w - gap_center_y_body * sin_w
                    gate_y_world = uav_pos[1] + gap_dist_x_body * sin_w + gap_center_y_body * cos_w
                    self.detected_opening_pos = (gate_x_world, gate_y_world, uav_pos[2])

        # Track opening traversal / crossing vector if opening detected
        if self.detected_opening_pos is not None:
            gx, gy, gz = self.detected_opening_pos
            vec_to_gate = np.array([gx - uav_pos[0], gy - uav_pos[1]])
            fwd_vec = np.array([math.cos(uav_yaw), math.sin(uav_yaw)])

            dot_prod = np.dot(vec_to_gate, fwd_vec)

            # If dot product changes sign or gate distance behind drone (x_body < -0.2), gate was crossed!
            if dot_prod < -0.10 or (self.opening_detected and np.linalg.norm(vec_to_gate) < 1.2 and dot_prod < 0.2):
                self.opening_crossed = True
                self.opening_behind = True

        self.cue_scores['cue1_opening_crossed'] = 1.0 if self.opening_crossed else (0.4 if self.opening_detected else 0.0)

        # --- Cue 2: Free Space Expansion ---
        # Measure volume of un-obstructed area in forward semi-cylinder (R=4m)
        free_pts_count = int(np.sum((x_band >= 0.5) & (x_band <= 4.0) & (np.abs(y_band) <= 2.5)))
        self.current_free_space = float(free_pts_count)

        if self.baseline_free_space is None:
            self.baseline_free_space = max(1.0, self.current_free_space)

        self.free_space_ratio = self.current_free_space / max(1.0, self.baseline_free_space)
        self.cue_scores['cue2_free_space'] = float(np.clip(self.free_space_ratio / 1.3, 0.0, 1.0))

        # --- Cue 5: Obstacle Density Increase ---
        # Measure obstacle point density in surrounding 360 annulus R in [1.5, 4.0]m
        ring_mask = (dist_3d >= 1.5) & (dist_3d <= 4.0) & (np.abs(dz) <= 1.0)
        self.current_obstacle_density = float(np.sum(ring_mask))

        if self.baseline_obstacle_density is None:
            self.baseline_obstacle_density = max(1.0, self.current_obstacle_density)

        self.obstacle_density_ratio = self.current_obstacle_density / max(1.0, self.baseline_obstacle_density)
        self.cue_scores['cue5_obstacle_density'] = float(np.clip(self.obstacle_density_ratio / 1.3, 0.0, 1.0))

    def evaluate_confidence(self, loc_healthy=True):
        """
        Computes weighted confidence score and applies mandatory cue hard-gating & hysteresis.
        """
        # --- Cue 3: Localization Health ---
        rate_ok = (self.odom_rate_hz >= 12.0) if self.odom_rate_hz > 0.0 else True
        jump_ok = (self.pose_jump_max <= 0.25)
        is_loc_stable = loc_healthy and rate_ok and jump_ok
        self.cue_scores['cue3_stable_localization'] = 1.0 if is_loc_stable else 0.0

        # Raw composite confidence score
        raw_score = (
            self.w1 * self.cue_scores['cue1_opening_crossed'] +
            self.w2 * self.cue_scores['cue2_free_space'] +
            self.w3 * self.cue_scores['cue3_stable_localization'] +
            self.w4 * self.cue_scores['cue4_obstacle_clearance'] +
            self.w5 * self.cue_scores['cue5_obstacle_density']
        )

        # Mandatory Cues Hard-Gating:
        # Cue 1 (Opening Crossed), Cue 3 (Stable Loc), and Cue 4 (Clearance) MUST be satisfied
        mandatory_satisfied = (
            self.opening_crossed and
            (self.cue_scores['cue3_stable_localization'] == 1.0) and
            (self.cue_scores['cue4_obstacle_clearance'] >= 0.5)
        )

        if mandatory_satisfied:
            self.confidence_score = float(raw_score)
        else:
            # Cap confidence below threshold if mandatory cues fail
            self.confidence_score = float(min(raw_score, 0.45))

        # Temporal Hysteresis Filter
        if self.confidence_score >= self.confidence_threshold and mandatory_satisfied:
            self.consecutive_stable_cycles += 1
        else:
            self.consecutive_stable_cycles = 0

        return self.confidence_score, mandatory_satisfied, (self.consecutive_stable_cycles >= self.stability_required_cycles)


class EntryDetectionModuleNode:
    """
    Main ROS node for Entry Detection Module (EDM)
    Operates ONCE at mission start before handing over 100% control to FUEL.
    """

    def __init__(self):
        rospy.init_node('entry_detection_module', anonymous=False)

        self.state = MissionState.TAKEOFF
        self.detector = MultiCueEntryDetector()

        self.takeoff_height = rospy.get_param('~takeoff_height', 1.50)
        self.search_forward_speed = rospy.get_param('~search_forward_speed', 0.35)
        self.max_search_duration = rospy.get_param('~max_search_duration', 45.0)

        self.uav_pose = None
        self.uav_yaw = None
        self.home_x = None
        self.home_y = None
        self.home_z = self.takeoff_height

        self.is_armed = False
        self.current_mode = ""
        self.state_start_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb)
        rospy.Subscriber('/exploration_completed', Bool, self.completed_cb)

        # Publishers
        self.pub_mission_state = rospy.Publisher('/edm/mission_state', String, queue_size=5)
        self.pub_confidence = rospy.Publisher('/edm/confidence_score', Float64, queue_size=5)
        self.pub_diagnostics = rospy.Publisher('/edm/diagnostics', String, queue_size=5)
        self.pub_markers = rospy.Publisher('/edm/markers', MarkerArray, queue_size=5)
        self.pub_fuel_trigger = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
        self.pub_setpoint_raw = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

        # 20Hz Main Control Loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("[EDM] Entry Detection Module Node initialized successfully.")

    def state_cb(self, msg):
        self.is_armed = msg.armed
        self.current_mode = msg.mode

    def pose_cb(self, msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.uav_pose = np.array([px, py, pz])
        self.uav_yaw = yaw

        if self.home_x is None:
            self.home_x = px
            self.home_y = py
            rospy.loginfo(f"[EDM] Cached Home Coordinates: X={self.home_x:.2f}, Y={self.home_y:.2f}")

    def odom_cb(self, msg):
        self.detector.update_odometry(msg)

    def cloud_cb(self, msg):
        if self.uav_pose is not None and self.uav_yaw is not None:
            self.detector.process_point_cloud(msg, self.uav_pose, self.uav_yaw)

    def completed_cb(self, msg):
        if msg.data and self.state == MissionState.EXPLORATION:
            rospy.loginfo("[EDM] Exploration completed signal received! Transitioning to RETURN...")
            self.transition_to(MissionState.RETURN)

    def transition_to(self, new_state):
        rospy.loginfo(f"[EDM] State Transition: {self.state} ---> {new_state}")
        self.state = new_state
        self.state_start_time = rospy.Time.now()

    def publish_diagnostics(self, confidence, mandatory_ok, confirmed):
        self.pub_mission_state.publish(String(data=self.state))
        self.pub_confidence.publish(Float64(data=confidence))

        diag_data = {
            'timestamp': rospy.Time.now().to_sec(),
            'mission_state': self.state,
            'entry_confidence_score': round(confidence, 4),
            'opening_detected': self.detector.opening_detected,
            'opening_crossed': self.detector.opening_crossed,
            'opening_width_m': round(self.detector.detected_opening_width, 2),
            'free_space_ratio': round(self.detector.free_space_ratio, 2),
            'localization_health': self.detector.cue_scores['cue3_stable_localization'],
            'obstacle_clearance_m': round(self.detector.min_obstacle_dist, 2),
            'obstacle_density_ratio': round(self.detector.obstacle_density_ratio, 2),
            'mandatory_cues_satisfied': mandatory_ok,
            'entry_confirmed': confirmed,
            'stable_cycles': self.detector.consecutive_stable_cycles
        }
        self.pub_diagnostics.publish(String(data=json.dumps(diag_data)))

        # Publish RViz Visual Markers
        markers = MarkerArray()

        # Floating Text Score Marker
        if self.uav_pose is not None:
            txt_m = Marker()
            txt_m.header.frame_id = "camera_init"
            txt_m.header.stamp = rospy.Time.now()
            txt_m.id = 1
            txt_m.type = Marker.TEXT_VIEW_FACING
            txt_m.action = Marker.ADD
            txt_m.pose.position.x = self.uav_pose[0]
            txt_m.pose.position.y = self.uav_pose[1]
            txt_m.pose.position.z = self.uav_pose[2] + 0.6
            txt_m.scale.z = 0.25
            txt_m.color.r = 0.1
            txt_m.color.g = 0.9
            txt_m.color.b = 0.2
            txt_m.color.a = 1.0
            txt_m.text = f"EDM: {self.state}\nConf: {confidence:.2f} (Cycles: {self.detector.consecutive_stable_cycles})"
            markers.markers.append(txt_m)

        # Door opening marker
        if self.detector.detected_opening_pos is not None:
            gx, gy, gz = self.detector.detected_opening_pos
            door_m = Marker()
            door_m.header.frame_id = "camera_init"
            door_m.header.stamp = rospy.Time.now()
            door_m.id = 2
            door_m.type = Marker.CUBE
            door_m.action = Marker.ADD
            door_m.pose.position.x = gx
            door_m.pose.position.y = gy
            door_m.pose.position.z = gz
            door_m.scale.x = 0.10
            door_m.scale.y = max(0.5, self.detector.detected_opening_width)
            door_m.scale.z = 1.60
            door_m.color.r = 0.9 if not self.detector.opening_crossed else 0.1
            door_m.color.g = 0.2 if not self.detector.opening_crossed else 0.9
            door_m.color.b = 0.8
            door_m.color.a = 0.6
            markers.markers.append(door_m)

        self.pub_markers.publish(markers)

    def create_search_motion_cmd(self, x_vel, y_vel, target_z, yaw_rate):
        """
        Sends perception-guided search motion setpoints (forward preference, lateral gap centering)
        during ENTRY_SEARCH state.
        """
        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "camera_init"
        target.coordinate_frame = PositionTarget.FRAME_BODY_NED

        # Mask: ignore pos x/y, use velocity x/y, use target z position, use yaw_rate
        target.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_AX | PositionTarget.IGNORE_AY | PositionTarget.IGNORE_AZ |
            PositionTarget.IGNORE_YAW
        )

        # Enforce non-negative forward speed preference (v_x >= 0)
        target.velocity.x = max(0.0, x_vel)
        target.velocity.y = y_vel
        target.position.z = target_z
        target.yaw_rate = yaw_rate

        return target

    def trigger_fuel_exploration(self):
        """
        Publishes waypoint trigger to /waypoint_generator/waypoints to launch FUEL 360 exploration.
        """
        rospy.loginfo("[EDM] ENTRY CONFIRMED! Publishing waypoint path trigger to enable FUEL Exploration...")
        p = Path()
        p.header.frame_id = 'camera_init'
        p.header.stamp = rospy.Time.now()

        ps = PoseStamped()
        ps.header = p.header
        if self.uav_pose is not None:
            ps.pose.position.x = self.uav_pose[0]
            ps.pose.position.y = self.uav_pose[1]
            ps.pose.position.z = self.takeoff_height
        else:
            ps.pose.position.x = 0.0
            ps.pose.position.y = 0.0
            ps.pose.position.z = 1.5

        ps.pose.orientation.w = 1.0
        p.poses.append(ps)

        for _ in range(15):
            self.pub_fuel_trigger.publish(p)
            rospy.sleep(0.05)

        rospy.loginfo("[EDM] FUEL Trigger Published Successfully! EDM deactivating motion control.")

    def control_loop(self, event):
        if self.uav_pose is None or self.uav_yaw is None:
            return

        conf, mandatory_ok, confirmed = self.detector.evaluate_confidence()
        self.publish_diagnostics(conf, mandatory_ok, confirmed)

        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()

        if self.state == MissionState.TAKEOFF:
            # Wait for UAV to arm & reach cruising altitude
            if self.uav_pose[2] >= (self.takeoff_height - 0.20):
                rospy.loginfo("[EDM] Takeoff height reached. Transitioning to ENTRY_SEARCH.")
                self.transition_to(MissionState.ENTRY_SEARCH)

        elif self.state == MissionState.ENTRY_SEARCH:
            # Perceptual search motion logic:
            # Forward motion preference (0.35 m/s) with obstacle gap alignment
            v_fwd = self.search_forward_speed
            v_lat = 0.0

            # If opening detected ahead, steer laterally toward gap center
            if self.detector.opening_detected and self.detector.detected_opening_pos is not None:
                gx, gy, gz = self.detector.detected_opening_pos
                dx = gx - self.uav_pose[0]
                dy = gy - self.uav_pose[1]

                # Transform to body frame lateral error
                cos_y = math.cos(-self.uav_yaw)
                sin_y = math.sin(-self.uav_yaw)
                lat_err = dx * sin_y + dy * cos_y

                v_lat = float(np.clip(0.8 * lat_err, -0.4, 0.4))

            # Send search motion setpoint
            sp = self.create_search_motion_cmd(v_fwd, v_lat, self.takeoff_height, 0.0)
            self.pub_setpoint_raw.publish(sp)

            # Check transition condition: Entry confirmed!
            if confirmed:
                rospy.loginfo(f"[EDM] Entry confirmed after {elapsed:.1f}s in ENTRY_SEARCH (Conf: {conf:.2f}).")
                self.transition_to(MissionState.ENTRY_CONFIRMATION)
            elif elapsed > self.max_search_duration:
                rospy.logwarn(f"[EDM] Max search duration ({self.max_search_duration}s) reached. Forcing transition to ENTRY_CONFIRMATION.")
                self.transition_to(MissionState.ENTRY_CONFIRMATION)

        elif self.state == MissionState.ENTRY_CONFIRMATION:
            # Hold zero velocity for 0.5s to stabilize before handover
            sp = self.create_search_motion_cmd(0.0, 0.0, self.takeoff_height, 0.0)
            self.pub_setpoint_raw.publish(sp)

            if elapsed >= 0.5:
                # Trigger FUEL Exploration and hand over 100% control
                self.trigger_fuel_exploration()
                self.transition_to(MissionState.EXPLORATION)

        elif self.state == MissionState.EXPLORATION:
            # EDM is now PASSIVE / DISABLED. Control is 100% handled by FUEL.
            rospy.loginfo_throttle(10.0, "[EDM] Mission State: EXPLORATION (FUEL active, EDM passive).")

        elif self.state == MissionState.RETURN:
            rospy.loginfo_throttle(5.0, "[EDM] Mission State: RETURN TO HOME.")

        elif self.state == MissionState.LAND:
            rospy.loginfo_throttle(5.0, "[EDM] Mission State: LANDING.")


if __name__ == '__main__':
    try:
        node = EntryDetectionModuleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
