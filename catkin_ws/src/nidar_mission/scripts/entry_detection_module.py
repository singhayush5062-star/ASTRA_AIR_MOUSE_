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
from collections import deque
import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped, Point, Vector3
from sensor_msgs.msg import PointCloud2, Imu
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String, Float64, Bool, Float64MultiArray
from nav_msgs.msg import Path, Odometry
from mavros_msgs.msg import State
# The EDM commands motion on /planning/pos_cmd (validated by flight_envelope_guard.py),
# the same interface FUEL's traj_server uses, so it speaks PositionCommand rather than the
# PositionTarget it previously wrote straight to MAVROS.
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MissionState:
    TAKEOFF = "TAKEOFF"
    ENTRY_SEARCH = "ENTRY_SEARCH"
    ENTRY_CONFIRMATION = "ENTRY_CONFIRMATION"
    EXPLORATION = "EXPLORATION"
    RETURN = "RETURN"
    # Explicit final approach to the launch pad. Runs BEFORE handing to PX4
    # AUTO.LAND. Ensures the touchdown is on the pad rather than wherever
    # AUTO.LAND happens to be issued -- run 20260911_101520 saw PX4 fire its
    # own "Failsafe: blind land" at t=527 because coverage plateaued below the
    # 97% floor and EDM never reached RETURN, leaving the vehicle to be landed
    # mid-arena by PX4's failsafe rather than on the pad by us.
    DESCEND = "DESCEND"
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
        # Defaults come from /nidar/entry in nidar_mission/config/mission_config.yaml; a
        # private (~) param still wins for one-off experiments. The fallbacks below are sized
        # for the ARINA_NIDAR arena's measured 1.90 m south door.
        def _p(name, default):
            return rospy.get_param('~' + name, rospy.get_param('/nidar/entry/' + name, default))
        self.door_min_width = _p('door_min_width', 0.80)
        self.door_max_width = _p('door_max_width', 3.00)
        self.target_door_width = _p('target_door_width', 1.90)
        self.confidence_threshold = _p('confidence_threshold', 0.70)
        self.stability_required_cycles = _p('stability_required_cycles', 10)
        self.min_clearance = _p('min_clearance', 0.42)
        self.target_clearance = _p('target_clearance', 0.60)

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

        def _cfg(name, default):
            """~param, else /nidar/entry/<name>, else default."""
            return rospy.get_param('~' + name,
                                   rospy.get_param('/nidar/entry/' + name, default))

        # Mission-level values live in nidar_mission/config/mission_config.yaml under
        # /nidar/entry and /nidar/vehicle; private (~) params still override for one-off tests.
        self.takeoff_height = rospy.get_param(
            '~takeoff_height', rospy.get_param('/nidar/vehicle/cruise_altitude_world', 1.50))
        self.search_forward_speed = rospy.get_param(
            '~search_forward_speed', rospy.get_param('/nidar/entry/search_forward_speed', 0.40))
        self.max_search_duration = rospy.get_param(
            '~max_search_duration', rospy.get_param('/nidar/entry/max_search_duration', 30.0))
        # How far past the door line counts as "inside".
        self.entry_confirm_depth = rospy.get_param(
            '~entry_confirm_depth', rospy.get_param('/nidar/entry/entry_confirm_depth', 1.0))
        # Cruise altitude expressed in camera_init, which is what /planning/pos_cmd carries.
        # traj_server pins its own output to the same z_cruise, so this must match it exactly
        # or the handover to FUEL steps the altitude.
        #
        # DERIVED, not a literal. camera_init's origin is base_link at FAST-LIO init, i.e. the
        # spawn height, so camera_init z = world z - (pad thickness + belly clearance). The old
        # hardcoded 1.40 was correct only for the iris, which spawned at 0.10; on the X500
        # (spawn 0.26) it meant world 1.66, above the guard's 1.55 ceiling, so the EDM handed
        # over at an altitude the guard was actively clamping.
        # NOTE: not via _cfg(), which prefixes /nidar/entry/ -- these live elsewhere in the
        # config tree, and _cfg would silently fall through to the defaults.
        _cruise_w = rospy.get_param('/nidar/vehicle/cruise_altitude_world', 1.5)
        _spawn_z = (rospy.get_param('/nidar/launch_pad/thickness', 0.03)
                    + rospy.get_param('/nidar/vehicle/belly_clearance', 0.23))
        self.cruise_z_camera_init = rospy.get_param(
            '~cruise_z_camera_init', round(_cruise_w - _spawn_z, 3))
        rospy.loginfo("[EDM] cruise altitude: world %.2f, camera_init %.3f (spawn z %.3f)",
                      _cruise_w, self.cruise_z_camera_init, _spawn_z)

        # ---- RETURN leg -------------------------------------------------------------
        # Breadcrumb trail of where the vehicle has actually BEEN, camera_init xy, recorded
        # throughout exploration and flown backwards to get home.
        #
        # Why retrace rather than plan a route home: the arena has interior structure, and a
        # straight line from wherever exploration ended back to the door goes through walls.
        # FUEL could plan it, but FUEL is in FINISH by then and has no goal interface, and
        # writing a second planner into the mission layer would be a second thing to get wrong.
        # Every crumb is a point the vehicle physically occupied, so the reversed trail is
        # collision-free by construction - no map, no planner, no assumptions.
        self.breadcrumbs = []
        self.crumb_spacing = 0.5          # metres between crumbs
        # Loop closure: when a new crumb lands within this of an older one, everything between
        # them is dropped. The vehicle occupied both endpoints, and they are closer together
        # than the 0.4 m planning inflation, so the shortcut is inside swept free space. This
        # is what stops a 15 minute exploration producing a 15 minute return.
        self.crumb_loop_radius = 0.3
        self.return_path = []
        self.return_wp = None             # index into return_path
        self.return_accept = 0.35         # metres, waypoint acceptance radius
        self.return_deadline = None
        self.max_return_duration = rospy.get_param(
            '~max_return_duration', rospy.get_param('/nidar/return/max_duration', 240.0))
        self.landed_requested = False

        # ---- DESCEND leg ------------------------------------------------------------
        # DESCEND owns the last approach to the pad and the handoff to AUTO.LAND. It exists
        # because RETURN commanding AUTO.LAND directly landed the vehicle at whatever pose
        # RETURN happened to finish at (or, when RETURN timed out, mid-arena) -- so a run
        # that never satisfied the coverage completion condition ended with a PX4 blind land
        # rather than a controlled touchdown on the pad.
        #
        # State machine:
        #   1. If the vehicle is > descend_accept from the pad centre, command pos_cmd at
        #      the pad centre (camera_init origin) and wait for it to close in.
        #   2. Once within descend_accept OR after descend_timeout, hand off to AUTO.LAND.
        #      The guard's z window bottoms at 1.45 m so we cannot descend on pos_cmd; the
        #      last metre is PX4's job either way. What DESCEND adds is that PX4 gets asked
        #      only when the vehicle is where it belongs, not from a stall inside the maze.
        # Pad centre in camera_init is (0, 0) by construction (spawn plants camera_init).
        self.descend_accept = rospy.get_param(
            '~descend_accept', rospy.get_param('/nidar/descend/accept_radius', 0.40))
        self.descend_timeout = rospy.get_param(
            '~descend_timeout', rospy.get_param('/nidar/descend/timeout', 15.0))
        self.descend_start_time = None

        # Integrated position setpoint. See create_position_cmd() for why the EDM commands
        # position rather than the velocity setpoints it used to publish.
        self.setpoint_xy = None
        # Heading latched when ENTRY_SEARCH begins, held all the way through the door.
        self.entry_yaw = None
        # camera_init x at which the vehicle is considered through the opening. Derived in
        # ENTRY_SEARCH from the door line in mission_config.yaml: with the standard spawn
        # (facing +Y world, camera_init x == world y - spawn_y) the door at world y = line
        # sits at camera_init x = line - spawn_y.
        self.entry_line_camera_x = None

        # --- Approach corridor -------------------------------------------------------
        # The 2026-09-05 entry run crashed because nothing held the vehicle on the door
        # centreline: it drifted to world x=-1.04 while the opening only spans +-0.95 m, so
        # with a 0.384 m collision radius its west edge sat 0.47 m inside the wall and it
        # clipped the jamb (ulog: pitch 0 -> -39.5 deg over t+38..48, thrust saturated at
        # 1.0, vehicle on the floor from t+52). Perception alone cannot be the only thing
        # keeping the vehicle centred -- the gap detector never fired on that run -- so the
        # KNOWN door position from mission_config.yaml is now the primary lateral reference
        # and the lidar detector only refines it.
        self.door_camera_x = None   # door line, camera_init forward axis
        self.door_camera_y = None   # door centre, camera_init lateral axis
        self.align_tolerance = _cfg('align_tolerance', 0.20)
        # How far ahead of the vehicle the forward setpoint may sit. This, not a
        # velocity command, is what sets the approach speed; PX4's position
        # controller converts it into a properly damped motion.
        self.approach_lead = _cfg('approach_lead', 0.6)
        # Once advancing, keep advancing until the error exceeds this (hysteresis).
        self.align_release = _cfg('align_release', 0.40)
        # Max distance the lidar gap detector may shift the configured door centre.
        self.detector_refine_limit = _cfg('detector_refine_limit', 0.10)
        # Cap on how far past the confirm line the forward setpoint may reach.
        self.approach_stop_margin = _cfg('approach_stop_margin', 0.25)
        self.approach_fwd = None
        self.approach_advancing = False
        # Hard clamp on how far the integrated setpoint may sit off the centreline. Sized
        # from the real geometry: half-door (0.95) - collision radius (0.384) = 0.566 m of
        # true clearance; 0.45 keeps a margin inside that.
        self.corridor_half_width = _cfg('corridor_half_width', 0.45)
        # Radius around the pad within which the vehicle counts as "still on the pad".
        self.pad_radius = _cfg('pad_radius', 1.0)
        # How long to hold inside the arena waiting for FUEL to subscribe.
        self.fuel_trigger_timeout = _cfg('fuel_trigger_timeout', 60.0)
        # EKF2 occasionally snaps its horizontal position (a 1.87 m reset on 2026-09-05 threw
        # the vehicle past the 60 deg tilt limit and PX4 terminated the flight). A reset is a
        # step in the ESTIMATE, not vehicle motion, so the correct response is to ride it out:
        # freeze the approach, re-seed the setpoint on the new estimate, and resume once the
        # pose has been quiet for settle_time.
        self.pose_jump_threshold = _cfg('pose_jump_threshold', 0.30)
        self.settle_time = _cfg('settle_time', 3.0)
        # Latched once the vehicle has actually been observed on the pad, on the ground.
        # Without this the TAKEOFF gate is satisfied by ANY pose above the height threshold,
        # including one sampled mid-flight -- which is exactly what happened on 2026-09-05:
        # the EDM node started 9 s after the vehicle was already airborne in OFFBOARD, saw
        # 1.5 m, and transitioned 0.18 s after init while 1 m off the door centreline.
        self.saw_on_pad = False
        self.pad_z = None
        self.last_pose_xy = None
        self.last_jump_time = None
        self.last_jump_size = 0.0
        self.approach_ready_logged = False

        self.uav_pose = None
        self.uav_yaw = None
        self.home_x = None
        self.home_y = None
        self.home_z = self.takeoff_height

        self.is_armed = False
        self.current_mode = ""
        self.state_start_time = rospy.Time.now()

        # --- coverage-plateau completion -------------------------------------------------
        # A SECOND, independent completion condition: mapped free area has stopped growing
        # while coverage is already high. FUEL's own /exploration_completed stays the primary
        # path and is untouched; this only matters when FUEL has not yet declared finish.
        #
        # Measured 2026-09-09 by replaying the recorded coverage streams of the three
        # ground-truth-verified successful runs (20260908_141709, 20260908_143514,
        # 20260909_040942) through this exact callback: at the defaults below the condition
        # first holds at 98.09 / 98.48 / 99.19 % coverage, saving 70 / 117 / 74 s once armed.
        #
        # min_coverage_pct is a safety gate, not decoration. Run 20260908_160711 published
        # exploration_completed = True at 95.3 % after the retirement machinery emptied the
        # frontier list; a floor of 97 stops this trigger doing the same thing sooner.
        self.coverage_hist = deque()          # (t, free_area_m2, pct) inside the window
        self.plateau_fired = False            # one-shot latch
        self.plateau_stop_sent_time = None    # when we asked FUEL to stop
        # If FUEL never answers the stop request we must still come home rather than orbit the
        # arena forever, so give up on the handshake after this long and transition anyway.
        self.plateau_stop_timeout_s = rospy.get_param(
            '/nidar/exploration_completion/stop_handshake_timeout_s', 15.0)

        # --- absolute mission clock -------------------------------------------------------
        # mission.clock_limit_s was documented as the 30-minute rule cap but NOTHING read it:
        # grepped 2026-09-09, the only file mentioning clock_limit was the config defining it.
        # The sole enforced timeout was return.max_duration, which applies only AFTER the
        # mission has reached RETURN -- so a vehicle stuck in EXPLORATION was never landed.
        # Run 20260909_062048 sat wedged at 49.5 % coverage for 14 minutes and was still
        # flying when stopped by hand; it would have run until the battery.
        self.mission_start_time = None
        self.clock_limit_s = rospy.get_param('/nidar/mission/clock_limit_s', 1800.0)
        # Leave enough of the budget to actually fly home rather than cutting at the very end.
        self.clock_return_margin_s = rospy.get_param(
            '/nidar/mission/clock_return_margin_s', 300.0)
        self.clock_forced_return = False
        self.plateau_window_s = rospy.get_param(
            '/nidar/exploration_completion/plateau_window_s', 45.0)
        self.plateau_growth_m2 = rospy.get_param(
            '/nidar/exploration_completion/plateau_growth_m2', 0.5)
        self.min_coverage_pct = rospy.get_param(
            '/nidar/exploration_completion/min_coverage_pct', 97.0)
        self.plateau_force = rospy.get_param(
            '/nidar/exploration_completion/force_enabled', False)
        rospy.loginfo("[EDM] coverage-plateau completion: window=%.0fs growth<%.2fm2 "
                      "coverage>=%.1f%% force=%s",
                      self.plateau_window_s, self.plateau_growth_m2,
                      self.min_coverage_pct, self.plateau_force)

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/Fast_LIO/odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/cloud_registered', PointCloud2, self.cloud_cb)
        rospy.Subscriber('/exploration_completed', Bool, self.completed_cb)
        rospy.Subscriber('/sdf_map/coverage', Float64MultiArray, self.coverage_cb)

        # Publishers
        self.pub_mission_state = rospy.Publisher('/edm/mission_state', String, queue_size=5)
        self.pub_confidence = rospy.Publisher('/edm/confidence_score', Float64, queue_size=5)
        self.pub_diagnostics = rospy.Publisher('/edm/diagnostics', String, queue_size=5)
        self.pub_markers = rospy.Publisher('/edm/markers', MarkerArray, queue_size=5)
        self.pub_fuel_trigger = rospy.Publisher('/waypoint_generator/waypoints', Path, queue_size=1)
        # SAFETY: this used to publish PositionTarget straight onto /mavros/setpoint_raw/local.
        # That is the topic flight_envelope_guard.py itself PUBLISHES on, so the EDM was not
        # merely bypassing the envelope check -- it was a second publisher racing the guard, and
        # PX4 received an interleaved mix of validated and unvalidated setpoints. Every other
        # motion source in this stack goes through /planning/pos_cmd and is validated before it
        # reaches MAVROS; the EDM now does the same.
        self.pub_pos_cmd = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
        # Handshake out of exploration. FUEL transits to FINISH on this and then publishes the
        # latched /exploration_completed we already subscribe to; see coverage_cb.
        self.pub_stop_exploration = rospy.Publisher('/mission/stop_exploration', Bool,
                                                    queue_size=1, latch=True)

        # 20Hz Main Control Loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("[EDM] Entry Detection Module Node initialized successfully.")

    def state_cb(self, msg):
        # Anchor the mission clock on the first armed report. Everything else in this node
        # measures time from the last STATE TRANSITION, which by construction can never bound
        # the total flight -- see the clock check in control_loop().
        if msg.armed and self.mission_start_time is None:
            self.mission_start_time = rospy.Time.now()
            rospy.loginfo("[EDM] mission clock started (limit %.0f s).", self.clock_limit_s)
        self.is_armed = msg.armed
        self.current_mode = msg.mode

    def pose_cb(self, msg):
        px = msg.pose.position.x
        py = msg.pose.position.y
        pz = msg.pose.position.z

        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.last_pose_xy is not None:
            step = math.hypot(px - self.last_pose_xy[0], py - self.last_pose_xy[1])
            if step > self.pose_jump_threshold:
                # /mavros/local_position/pose runs at ~30 Hz, so a step this large is an EKF2
                # reset, not motion (it would imply >9 m/s).
                self.last_jump_time = rospy.Time.now()
                self.last_jump_size = step
                rospy.logwarn("[EDM] Estimator position reset of %.2f m detected; freezing "
                              "the approach for %.1fs and re-seeding the setpoint.",
                              step, self.settle_time)
                self.setpoint_xy = [px, py]
        self.last_pose_xy = (px, py)

        self.uav_pose = np.array([px, py, pz])
        self.uav_yaw = yaw

        if self.home_x is None:
            self.home_x = px
            self.home_y = py
            rospy.loginfo(f"[EDM] Cached Home Coordinates: X={self.home_x:.2f}, Y={self.home_y:.2f}")

    def odom_cb(self, msg):
        self.detector.update_odometry(msg)

    def _record_breadcrumb(self):
        """Append the current position to the trail, with loop-closure pruning."""
        if self.uav_pose is None:
            return
        p = (self.uav_pose[0], self.uav_pose[1])
        if self.breadcrumbs:
            last = self.breadcrumbs[-1]
            if math.hypot(p[0] - last[0], p[1] - last[1]) < self.crumb_spacing:
                return
        self.breadcrumbs.append(p)
        # Prune any loop we just closed: if this crumb is near an older one, everything
        # between them was a detour we do not need to re-fly.
        for i in range(len(self.breadcrumbs) - 3, -1, -1):
            q = self.breadcrumbs[i]
            if math.hypot(p[0] - q[0], p[1] - q[1]) < self.crumb_loop_radius:
                dropped = len(self.breadcrumbs) - 2 - i
                if dropped > 0:
                    del self.breadcrumbs[i + 1:-1]
                    rospy.logdebug("[EDM] return trail: closed a loop, dropped %d crumbs",
                                   dropped)
                break

    def cloud_cb(self, msg):
        if self.uav_pose is not None and self.uav_yaw is not None:
            self.detector.process_point_cloud(msg, self.uav_pose, self.uav_yaw)

    def completed_cb(self, msg):
        if msg.data and self.state == MissionState.EXPLORATION:
            rospy.loginfo("[EDM] Exploration completed signal received! Transitioning to RETURN...")
            self.transition_to(MissionState.RETURN)

    def coverage_cb(self, msg):
        """Coverage-plateau completion trigger. See the block in __init__ for the rationale.

        /sdf_map/coverage is published by MapROS::coverageCallback every map_ros/coverage_interval
        (2.0 s). Layout, from map_ros.cpp:
            [free_n, occ_n, unk_n, free_area, known_area, unknown_area,
             denom, pct, left, layers, elapsed_s]
        so index 3 is free area in m2 and index 7 is the observable-coverage percentage.

        This never publishes on /exploration_completed. FUEL's FSM owns that topic and latches
        it; a second latched publisher would leave subscribers with whichever wrote last. EDM
        already owns the RETURN transition, so it transitions directly.
        """
        if len(msg.data) < 8:
            return

        # History is only meaningful inside one exploration episode. Clearing it outside
        # EXPLORATION stops a window built during TAKEOFF/ENTRY -- when free area is flat
        # because FUEL is not mapping yet -- from satisfying the plateau test on the first
        # exploration sample.
        if self.state != MissionState.EXPLORATION:
            self.coverage_hist.clear()
            return

        now = rospy.Time.now().to_sec()
        free = msg.data[3]
        pct = msg.data[7]

        self.coverage_hist.append((now, free, pct))
        # Keep the OLDEST sample that is still at least a full window back, so the span
        # measured below is always >= plateau_window_s rather than just under it.
        #
        # Trimming to `now - hist[0] <= window` instead -- the obvious way to write this --
        # is silently broken: samples arrive every coverage_interval (2.0 s), so the span can
        # only take discrete values, and a strict "span >= window" test then passes only when
        # the two happen to line up. Measured 2026-09-09 by replaying the recorded coverage
        # streams of the three successful runs through this callback: window=45 never fired on
        # ANY of them (largest 2 s multiple below 45 is 44), and window=60 fired on
        # 20260908_143514 and 20260909_040942 but not on 20260908_141709 -- pure floating-point
        # luck on whether the span evaluated to 60.0 or 59.999...
        while (len(self.coverage_hist) >= 2 and
               now - self.coverage_hist[1][0] >= self.plateau_window_s):
            self.coverage_hist.popleft()

        if self.plateau_fired or len(self.coverage_hist) < 2:
            return
        # Judge only on a FULL window, or the test passes trivially at episode start.
        if now - self.coverage_hist[0][0] < self.plateau_window_s:
            return

        growth = free - self.coverage_hist[0][1]
        would = (pct >= self.min_coverage_pct) and (growth < self.plateau_growth_m2)

        # Logged whether or not it is armed: this line is the calibration dataset.
        rospy.loginfo_throttle(
            10.0, "[EDM] plateau: pct=%.2f growth=%.2f m2 / %.0f s would_trigger=%s force=%s",
            pct, growth, self.plateau_window_s, would, self.plateau_force)

        if would and self.plateau_force:
            self.plateau_fired = True
            self.request_end_of_exploration(
                "coverage plateau (%.2f%%, +%.2f m2 / %.0f s)"
                % (pct, growth, self.plateau_window_s))

    def request_end_of_exploration(self, reason):
        """Ask FUEL to stop, instead of transitioning to RETURN underneath it.

        Both endings the mission layer decides on its own -- the coverage plateau and the
        mission clock -- used to call transition_to(RETURN) directly. That leaves FUEL in
        EXEC_TRAJ, still publishing /planning/pos_cmd, while run_return() publishes the same
        topic. Run 20260909_101738 is what that costs: the plateau fired at t=505.4 s but FUEL
        did not finish until t=594.5 s, so for 89 s both drove the topic, coverage sat frozen at
        99.24%, FAST-LIO then diverged at t=600 and PX4 failsafed the vehicle down 9.1 m from
        the pad. Run 20260909_082915 failed the same way for 140 s.

        FUEL answers on the latched /exploration_completed, which completed_cb already handles;
        the watchdog in control_loop transitions anyway if that answer never comes.
        """
        if self.state != MissionState.EXPLORATION:
            return
        self.plateau_stop_sent_time = rospy.Time.now()
        rospy.logwarn("[EDM] %s -- asking FUEL to stop exploring (waiting up to %.0f s for the "
                      "handover).", reason, self.plateau_stop_timeout_s)
        self.pub_stop_exploration.publish(Bool(data=True))

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

    def _pose_settled(self):
        """True when no estimator reset has been seen for settle_time."""
        if self.last_jump_time is None:
            return True
        return (rospy.Time.now() - self.last_jump_time).to_sec() >= self.settle_time

    def _resolve_door_geometry(self):
        """Express the configured door in camera_init, the frame /planning/pos_cmd uses.

        camera_init is planted at the spawn pose with its +x along the spawn heading. For the
        standard pad-facing-north setup that gives
            camera_init x = world y - spawn_y      (forward, toward the arena)
            camera_init y = spawn_x - world x      (lateral)
        which is the same mapping flight_envelope_guard.camera_to_world() inverts.
        """
        arena = rospy.get_param('/nidar/arena/active', 'arina_nidar')
        base = '/nidar/arenas/%s/entry' % arena
        line = rospy.get_param(base + '/line', None)
        cx = rospy.get_param(base + '/center/x', None)
        spawn_x = rospy.get_param('/nidar/launch_pad/center/x', None)
        spawn_y = rospy.get_param('/nidar/launch_pad/center/y', None)
        if line is None or spawn_y is None:
            rospy.logerr("[EDM] Door line missing from mission_config.yaml (%s/line); "
                         "geometric entry test and lateral centring are DISABLED.", base)
            return
        self.entry_line_camera_x = line - spawn_y
        self.door_camera_x = self.entry_line_camera_x
        if cx is not None and spawn_x is not None:
            self.door_camera_y = spawn_x - cx
        else:
            self.door_camera_y = 0.0
            rospy.logwarn("[EDM] Door centre x missing; assuming it is straight ahead "
                          "of the pad (camera_init y = 0).")
        rospy.loginfo("[EDM] Door: world y=%.2f centre x=%s -> camera_init (x=%.2f, y=%.2f); "
                      "corridor +-%.2f m, align tol %.2f m, confirm %.2f m past the line.",
                      line, cx, self.door_camera_x, self.door_camera_y,
                      self.corridor_half_width, self.align_tolerance, self.entry_confirm_depth)

    def _approach_errors(self):
        """Body-frame (forward, lateral) error from the vehicle to the door centre."""
        if self.door_camera_x is None or self.door_camera_y is None:
            return None, None
        ex = self.door_camera_x - self.uav_pose[0]
        ey = self.door_camera_y - self.uav_pose[1]
        yaw = self.entry_yaw if self.entry_yaw is not None else self.uav_yaw
        c, sn = math.cos(yaw), math.sin(yaw)
        return ex * c + ey * sn, -ex * sn + ey * c

    def create_position_cmd(self, v_fwd, v_lat, dt):
        """Advance an integrated position setpoint and return it as a PositionCommand.

        The EDM's search behaviour is naturally expressed as a velocity (creep forward,
        slide sideways onto the gap centre), but the guard validates POSITION commands on
        /planning/pos_cmd -- it has to, since an envelope is a position constraint and a
        velocity setpoint alone cannot be checked against one.

        So the requested body-frame velocity is integrated here into a world-frame position
        setpoint that leads the vehicle. The setpoint is seeded from the vehicle's own pose
        and re-anchored whenever it drifts more than lead_limit ahead, which keeps it from
        running away if the vehicle is held up (by the guard clamping, or by contact) while
        this loop keeps integrating.
        """
        px, py = self.uav_pose[0], self.uav_pose[1]
        if self.setpoint_xy is None:
            self.setpoint_xy = [px, py]

        # Body -> camera_init. uav_yaw is the vehicle heading in the same frame the pose is in.
        cos_y, sin_y = math.cos(self.uav_yaw), math.sin(self.uav_yaw)
        vx = v_fwd * cos_y - v_lat * sin_y
        vy = v_fwd * sin_y + v_lat * cos_y

        self.setpoint_xy[0] += vx * dt
        self.setpoint_xy[1] += vy * dt

        # Anti-windup: never let the setpoint lead the vehicle by more than this.
        lead_limit = 1.0
        ex, ey = self.setpoint_xy[0] - px, self.setpoint_xy[1] - py
        lead = math.hypot(ex, ey)
        if lead > lead_limit:
            scale = lead_limit / lead
            self.setpoint_xy[0] = px + ex * scale
            self.setpoint_xy[1] = py + ey * scale

        # Corridor clamp: the integrated setpoint may never sit further off the door
        # centreline than the airframe can actually fit through. This is a hard geometric
        # limit, independent of whatever the detector or the integrator produce.
        if self.door_camera_y is not None and self.entry_yaw is not None:
            c, sn = math.cos(self.entry_yaw), math.sin(self.entry_yaw)
            rx = self.setpoint_xy[0] - (self.door_camera_x if self.door_camera_x is not None
                                        else self.setpoint_xy[0])
            ry = self.setpoint_xy[1] - self.door_camera_y
            lat = -rx * sn + ry * c
            if abs(lat) > self.corridor_half_width:
                corr = (abs(lat) - self.corridor_half_width) * (1.0 if lat > 0 else -1.0)
                self.setpoint_xy[0] += corr * sn
                self.setpoint_xy[1] -= corr * c

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "camera_init"
        cmd.position.x = self.setpoint_xy[0]
        cmd.position.y = self.setpoint_xy[1]
        cmd.position.z = self.cruise_z_camera_init
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = 0.0
        # Hold the spawn heading through the door. Yawing while threading a 1.90 m opening in a
        # 0.767 m airframe spends clearance for no benefit; FUEL takes over yaw after handover.
        cmd.yaw = self.uav_yaw if self.entry_yaw is None else self.entry_yaw
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = 1
        return cmd

    def create_approach_cmd(self, advance, dt):
        """Build the approach setpoint directly in the door frame.

        The lateral coordinate is commanded ABSOLUTELY, as the door centreline -- it is NOT
        servoed. An earlier version ran a proportional controller on lateral error and
        integrated its output into the position setpoint. That is an outer position loop
        wrapped around PX4's own position loop, and the two fought: on 2026-09-05 the vehicle
        limit-cycled at 0.17 Hz with 2.01 m peak-to-peak lateral travel (std 0.54 m) while the
        commanded setpoint moved only +-0.4 m and sat in ANTIPHASE with the vehicle -- the
        signature of a loop oscillating, not tracking. It ended in a -22 deg pitch excursion,
        thrust saturation and flight termination.

        The door position is known, so no servo is needed: command the centreline and let the
        vehicle's own (properly damped) position controller converge to it. Only the forward
        coordinate is advanced, and only when already aligned.
        """
        px, py = self.uav_pose[0], self.uav_pose[1]
        yaw = self.entry_yaw if self.entry_yaw is not None else self.uav_yaw
        c, sn = math.cos(yaw), math.sin(yaw)
        # Door frame: origin at the door centre, +f along the approach heading.
        dx, dy = px - self.door_camera_x, py - self.door_camera_y
        fwd_now = dx * c + dy * sn
        if advance:
            base = fwd_now if self.approach_fwd is None else max(self.approach_fwd, fwd_now)
            # Never lead the vehicle by more than approach_lead along the corridor: the lead
            # IS the speed command, and an unbounded one would demand max acceleration.
            self.approach_fwd = min(base + self.search_forward_speed * dt,
                                    fwd_now + self.approach_lead)
            # Never aim past the stop line. The corridor narrows sharply ~0.9 m inside the
            # door (0.34 m clearance at world y=-6.45, against a 0.384 m vehicle radius), so a
            # setpoint that reaches beyond the confirm line drives the vehicle into a wall --
            # which is exactly how both hard crashes on 2026-09-05 ended.
            self.approach_fwd = min(self.approach_fwd,
                                    self.entry_confirm_depth + self.approach_stop_margin)
        else:
            self.approach_fwd = fwd_now

        sx = self.door_camera_x + self.approach_fwd * c
        sy = self.door_camera_y + self.approach_fwd * sn
        self.setpoint_xy = [sx, sy]

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "camera_init"
        cmd.position.x = sx
        cmd.position.y = sy
        cmd.position.z = self.cruise_z_camera_init
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        cmd.yaw = yaw
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = 1
        return cmd

    def trigger_fuel_exploration(self):
        """
        Publishes waypoint trigger to /waypoint_generator/waypoints to launch FUEL 360 exploration.

        Caller must ensure a subscriber exists first -- see ENTRY_CONFIRMATION. A latched-style
        blind publish is not enough: test_takeoff.sh starts FUEL only after the climb, so on
        2026-09-05 the EDM crossed the door and fired this trigger ~10 s later, before
        exploration_node had finished registering its subscriber. The trigger was dropped and
        the vehicle hovered inside the arena for the rest of the run with a stale setpoint.
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
            # camera_init frame (set on p.header above), so the camera_init cruise altitude
            # belongs here -- takeoff_height is world AGL and would ask FUEL to explore 0.26 m
            # higher than the configured cruise height.
            ps.pose.position.z = self.cruise_z_camera_init
        else:
            ps.pose.position.x = 0.0
            ps.pose.position.y = 0.0
            ps.pose.position.z = self.cruise_z_camera_init

        ps.pose.orientation.w = 1.0
        p.poses.append(ps)

        for _ in range(15):
            self.pub_fuel_trigger.publish(p)
            rospy.sleep(0.05)

        rospy.loginfo("[EDM] FUEL Trigger Published Successfully! EDM deactivating motion control.")

    def _home_waypoints(self):
        """The trail home: reversed breadcrumbs, then out through the door to the pad.

        camera_init is planted at the vehicle's spawn pose, so the launch pad is the
        camera_init ORIGIN and needs no coordinate maths: (0, 0) is home. The door waypoints
        are the same ones the entry leg flew, expressed the same way, so the last leg retraces
        a path already proven flyable rather than assuming the doorway is where it should be.
        """
        wps = list(reversed(self.breadcrumbs))
        # Door line and pad, in camera_init. world -> camera_init here is
        #   cx = world_y - spawn_world_y,  cy = -(world_x - spawn_world_x)
        # which for the configured pad at (0, -9.5) makes the door line (world y = -7.5) sit
        # at cx = 2.0 and the pad at the origin.
        pad_x = rospy.get_param('/nidar/launch_pad/center/x', 0.0)
        pad_y = rospy.get_param('/nidar/launch_pad/center/y', -9.5)
        # Follow arena.active rather than naming an arena: switching arenas is a one-line
        # config edit everywhere else in this stack, and a hardcoded name here would silently
        # send the vehicle at the previous arena's doorway.
        active = rospy.get_param('/nidar/arena/active', 'arina_nidar')
        door = rospy.get_param('/nidar/arenas/%s/entry/center' % active, {'x': 0.0, 'y': -7.5})
        dcx = door.get('y', -7.5) - pad_y
        dcy = -(door.get('x', 0.0) - pad_x)
        # Just inside the door, on the door line, then just outside, then the pad. The
        # intermediate points matter: a single leg from the last crumb to the pad would cut
        # the corner through the arena's south wall.
        wps.append((dcx + 0.5, dcy))
        wps.append((dcx, dcy))
        wps.append((dcx - 1.0, dcy))
        wps.append((0.0, 0.0))
        return wps

    def run_return(self, dt):
        """Fly the recorded trail backwards to the launch pad, one waypoint at a time."""
        if self.uav_pose is None:
            return
        if self.return_wp is None:
            self.return_path = self._home_waypoints()
            self.return_wp = 0
            self.return_deadline = rospy.Time.now() + rospy.Duration(self.max_return_duration)
            self.setpoint_xy = [self.uav_pose[0], self.uav_pose[1]]
            rospy.logwarn("[EDM] RETURN: %d waypoints home (%d from the flown trail).",
                          len(self.return_path), len(self.breadcrumbs))

        # A return that overruns is more dangerous than one that stops: the battery model is
        # not simulated, but a vehicle wandering the arena on a stale trail is a crash waiting
        # to happen. Land where we are instead.
        if rospy.Time.now() > self.return_deadline:
            # A RETURN timeout means the trail was not walked home. Do NOT jump straight to
            # AUTO.LAND -- that would descend at whatever mid-arena pose the vehicle happens
            # to be in, which is exactly the "PX4 blind-land somewhere in the maze" failure
            # mode seen in run 20260911_101520. DESCEND owns the last leg: it steers the
            # vehicle to the pad centre (or, if the pad is genuinely unreachable, holds and
            # logs before ceding to AUTO.LAND with a 15 s safety cap).
            rospy.logerr("[EDM] RETURN timed out after %.0f s at waypoint %d/%d - handing to DESCEND.",
                         self.max_return_duration, self.return_wp, len(self.return_path))
            self.transition_to(MissionState.DESCEND)
            return

        # Advance through every waypoint already satisfied, not just the next one: after a
        # loop-closure prune several consecutive crumbs can sit inside the acceptance radius.
        while self.return_wp < len(self.return_path):
            tx, ty = self.return_path[self.return_wp]
            if math.hypot(tx - self.uav_pose[0], ty - self.uav_pose[1]) < self.return_accept:
                self.return_wp += 1
            else:
                break

        if self.return_wp >= len(self.return_path):
            rospy.logwarn("[EDM] RETURN complete: over the pad. Handing to DESCEND.")
            self.transition_to(MissionState.DESCEND)
            return

        tx, ty = self.return_path[self.return_wp]
        # Lead-limited, exactly like the entry leg: command a point at most lead_limit ahead of
        # the vehicle along the bearing to the waypoint, so the guard sees a reachable setpoint
        # and PX4's own position loop does the damping. Commanding the far waypoint directly
        # would ask for a step of several metres.
        lead_limit = 1.0
        ex, ey = tx - self.uav_pose[0], ty - self.uav_pose[1]
        d = math.hypot(ex, ey)
        if d > lead_limit:
            ex, ey = ex * lead_limit / d, ey * lead_limit / d
        self.setpoint_xy = [self.uav_pose[0] + ex, self.uav_pose[1] + ey]

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "camera_init"
        cmd.position.x = self.setpoint_xy[0]
        cmd.position.y = self.setpoint_xy[1]
        cmd.position.z = self.cruise_z_camera_init
        # Hold heading. The lidar is omnidirectional so there is nothing to gain by turning to
        # face the way home, and yaw rate is the one input that has actually broken FAST-LIO in
        # this stack (158 deg/s cost a flight on 2026-09-05).
        cmd.yaw = self.entry_yaw if self.entry_yaw is not None else self.uav_yaw
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = 3
        self.pub_pos_cmd.publish(cmd)
        rospy.loginfo_throttle(
            5.0, "[EDM] RETURN: waypoint %d/%d, %.2f m to go, %.0f s left",
            self.return_wp, len(self.return_path), d,
            (self.return_deadline - rospy.Time.now()).to_sec())

    def run_descend(self, dt):
        """Fly the final approach to the pad centre before handing off to AUTO.LAND.

        This exists specifically to fix the "touched down mid-arena" failure: a RETURN
        that overshoots or times out used to jump straight to AUTO.LAND, so PX4 descended
        vertically from whatever pose the vehicle was stopped at. DESCEND commands one more
        pos_cmd at the pad centre (camera_init origin, by construction), waits for the
        vehicle to close in, and only then issues AUTO.LAND. If the pad turns out to be
        unreachable within descend_timeout the hand-off still fires -- a controlled
        AUTO.LAND is better than orbiting until PX4 blind-lands us.
        """
        if self.uav_pose is None:
            return
        if self.descend_start_time is None:
            self.descend_start_time = rospy.Time.now()
            rospy.logwarn("[EDM] DESCEND: pad approach begun; commanding (0, 0) camera_init.")

        # Distance to the pad in camera_init (pad == origin by construction).
        dist = math.hypot(self.uav_pose[0], self.uav_pose[1])
        elapsed = (rospy.Time.now() - self.descend_start_time).to_sec()

        if dist <= self.descend_accept:
            rospy.logwarn("[EDM] DESCEND: over the pad (%.2f m from centre). Requesting AUTO.LAND.",
                          dist)
            self.transition_to(MissionState.LAND)
            return
        if elapsed >= self.descend_timeout:
            # Timing out is a genuine failure -- the vehicle could not get to the pad. Land
            # anyway rather than orbit indefinitely, but log loudly enough that the operator
            # (and any post-run analysis) can see the touchdown was off-pad.
            rospy.logerr("[EDM] DESCEND: %.1f s elapsed and still %.2f m from the pad; "
                         "AUTO.LAND-ing here.", elapsed, dist)
            self.transition_to(MissionState.LAND)
            return

        # Lead-limited hop toward the pad, same pattern as RETURN. Prevents commanding a
        # large step the guard would clamp anyway.
        lead = 0.6
        ex, ey = -self.uav_pose[0], -self.uav_pose[1]
        d = math.hypot(ex, ey)
        if d > lead:
            ex, ey = ex * lead / d, ey * lead / d
        self.setpoint_xy = [self.uav_pose[0] + ex, self.uav_pose[1] + ey]

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "camera_init"
        cmd.position.x = self.setpoint_xy[0]
        cmd.position.y = self.setpoint_xy[1]
        cmd.position.z = self.cruise_z_camera_init
        cmd.yaw = self.entry_yaw if self.entry_yaw is not None else self.uav_yaw
        cmd.yaw_dot = 0.0
        cmd.trajectory_id = 4
        self.pub_pos_cmd.publish(cmd)
        rospy.loginfo_throttle(
            2.0, "[EDM] DESCEND: %.2f m to pad centre, %.1f s left before forced LAND.",
            dist, self.descend_timeout - elapsed)

    def run_land(self):
        """Hand the descent to PX4 rather than flying it down on position setpoints.

        AUTO.LAND runs PX4's own land detector and disarms on touchdown. Descending on
        /planning/pos_cmd instead would fight the guard, whose altitude band bottoms out at
        1.45 m - it would clamp the vehicle in mid-air and it would never touch down.
        """
        if self.landed_requested:
            rospy.loginfo_throttle(5.0, "[EDM] Mission State: LANDING (PX4 AUTO.LAND).")
            return
        try:
            rospy.wait_for_service('/mavros/set_mode', timeout=5.0)
            set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            resp = set_mode(custom_mode='AUTO.LAND')
            self.landed_requested = bool(resp.mode_sent)
            if self.landed_requested:
                rospy.logwarn("[EDM] LAND: AUTO.LAND accepted. Mission complete.")
            else:
                rospy.logerr("[EDM] LAND: PX4 rejected AUTO.LAND; will retry.")
        except Exception as exc:
            rospy.logerr("[EDM] LAND: set_mode failed (%s); will retry.", exc)

    def control_loop(self, event):
        if self.uav_pose is None or self.uav_yaw is None:
            return

        conf, mandatory_ok, confirmed = self.detector.evaluate_confidence()
        self.publish_diagnostics(conf, mandatory_ok, confirmed)

        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()

        dt = 0.05  # control_loop timer period

        # --- absolute mission clock (BUG-3) ------------------------------------------------
        # Bounds the WHOLE flight, independent of which state it is stuck in. Two stages so a
        # timeout still ends on the pad rather than wherever it happened to be: force RETURN
        # with the margin left to fly home, then force LAND at the hard limit.
        if self.mission_start_time is not None:
            mission_s = (rospy.Time.now() - self.mission_start_time).to_sec()
            if (not self.clock_forced_return and
                    mission_s >= self.clock_limit_s - self.clock_return_margin_s and
                    self.state in (MissionState.ENTRY_SEARCH,
                                   MissionState.ENTRY_CONFIRMATION,
                                   MissionState.EXPLORATION)):
                self.clock_forced_return = True
                if self.state == MissionState.EXPLORATION:
                    # Same handshake as the plateau: FUEL owns /planning/pos_cmd right now.
                    self.request_end_of_exploration(
                        "mission clock %.0f s of %.0f s, %.0f s reserved to fly home"
                        % (mission_s, self.clock_limit_s, self.clock_return_margin_s))
                else:
                    # ENTRY_SEARCH / ENTRY_CONFIRMATION: FUEL has not been triggered yet, so
                    # there is no second publisher to hand over from.
                    rospy.logwarn("[EDM] mission clock %.0f s of %.0f s; %.0f s reserved to fly "
                                  "home. Returning.",
                                  mission_s, self.clock_limit_s, self.clock_return_margin_s)
                    self.transition_to(MissionState.RETURN)
            elif mission_s >= self.clock_limit_s and self.state != MissionState.LAND:
                rospy.logerr("[EDM] mission clock hard limit %.0f s reached in state %s; "
                             "landing here.", self.clock_limit_s, self.state)
                self.transition_to(MissionState.LAND)

        # --- stop-request handshake watchdog ------------------------------------------------
        # completed_cb normally makes this transition when FUEL answers. If it never does, we
        # must not orbit the arena indefinitely -- but we also must not take over pos_cmd early,
        # so this waits the full timeout before forcing it.
        if (self.plateau_stop_sent_time is not None and
                self.state == MissionState.EXPLORATION and
                (rospy.Time.now() - self.plateau_stop_sent_time).to_sec()
                >= self.plateau_stop_timeout_s):
            rospy.logerr("[EDM] FUEL did not acknowledge the stop request within %.0f s; "
                         "returning anyway.", self.plateau_stop_timeout_s)
            self.plateau_stop_sent_time = None
            self.transition_to(MissionState.RETURN)

        if self.state == MissionState.TAKEOFF:
            # The vehicle must be seen ON THE PAD, on the ground, BEFORE any takeoff is
            # accepted. camera_init is planted at the spawn pose, so the pad is the
            # camera_init origin by construction.
            if not self.saw_on_pad:
                dist_pad = math.hypot(self.uav_pose[0], self.uav_pose[1])
                if dist_pad < self.pad_radius and self.uav_pose[2] < 0.5:
                    self.saw_on_pad = True
                    self.pad_z = float(self.uav_pose[2])
                    rospy.loginfo("[EDM] Vehicle confirmed on the launch pad "
                                  "(camera_init %.2f, %.2f, %.2f). Awaiting takeoff.",
                                  *self.uav_pose)
                else:
                    # Refusing to command anything is the safe outcome: this node started
                    # too late to own the takeoff, so something else is already flying.
                    rospy.logerr_throttle(
                        5.0,
                        "[EDM] Started with the vehicle already %.2f m from the pad at "
                        "%.2f m altitude. The EDM must be running BEFORE takeoff to own the "
                        "approach; it will NOT command motion now. Start nidar_mission.launch "
                        "before arming." % (dist_pad, self.uav_pose[2]))
                    return

            # Own the vehicle through the climb. Previously the EDM published nothing until
            # ENTRY_SEARCH, so during takeoff the only setpoint source was the guard's
            # fallback hover-hold, which derives its target from /mavros/local_position/pose.
            # EKF2 is still finishing its external-vision alignment then -- the 2026-09-05
            # ulog shows a 90.8 deg heading reset and large delta_xy resets -- so the hold
            # point moved with every reset and the vehicle chased it 1.6 m sideways before
            # any mission code was in control. Commanding an explicit hold over the pad
            # (camera_init origin, by construction) removes that free-running window.
            if self.is_armed:
                self.setpoint_xy = [0.0, 0.0]
                self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))

            # Height gained since the pad, not absolute height: immune to wherever EKF2
            # happens to plant its height origin.
            climb = self.uav_pose[2] - (self.pad_z if self.pad_z is not None else 0.0)
            # Compare against the CAMERA_INIT cruise altitude, not takeoff_height. uav_pose
            # comes from /mavros/local_position/pose, which this stack feeds from FAST-LIO and
            # is therefore camera_init, whose origin sits at the spawn pose -- 0.26 m above the
            # world floor. takeoff_height is a WORLD AGL figure (1.50), so comparing the two
            # demanded a camera_init climb of 1.30 m from a vehicle whose commanded ceiling is
            # camera_init 1.240. The gate could then only open on a >0.06 m overshoot, which is
            # why entry worked on some runs and not others; once the EDM began holding the pad
            # through the climb the overshoot largely vanished and it stopped opening at all.
            # Measured 2026-09-07: held at camera_init z=1.24 for 7+ minutes, FUEL parked in
            # WAIT_TRIGGER the whole time, 5.7% coverage from the pad.
            if climb >= (self.cruise_z_camera_init - 0.20) and not self._pose_settled():
                # EKF2's external-vision alignment resets cluster in the seconds after takeoff.
                # Leaving the pad while they are still happening is what threw the vehicle
                # 2.2 m off centre on 2026-09-05.
                rospy.logwarn_throttle(2.0, "[EDM] At altitude but the estimator is still "
                                            "settling (last reset %.2f m); holding over the pad.",
                                       self.last_jump_size)
            elif climb >= (self.cruise_z_camera_init - 0.20):
                rospy.loginfo("[EDM] Takeoff height reached (%.2f m above the pad). "
                              "Transitioning to ENTRY_SEARCH.", climb)
                self.entry_yaw = self.uav_yaw
                self.setpoint_xy = [self.uav_pose[0], self.uav_pose[1]]
                self._resolve_door_geometry()
                self.approach_fwd = None
                self.transition_to(MissionState.ENTRY_SEARCH)

        elif self.state == MissionState.ENTRY_SEARCH:
            # Approach guidance. The KNOWN door centre from mission_config.yaml is the
            # primary lateral reference; the lidar gap detector only refines it when it has
            # actually locked onto an opening. On 2026-09-05 the detector never fired and
            # there was no geometric fallback, so nothing corrected a 1 m lateral drift and
            # the vehicle flew into the door jamb.
            if not self._pose_settled():
                rospy.logwarn_throttle(
                    2.0, "[EDM] Holding: estimator reset %.2f m, waiting %.1fs for the pose "
                         "to settle before resuming the approach.",
                    self.last_jump_size, self.settle_time)
                self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))
                return

            fwd_err, lat_err = self._approach_errors()

            if (self.detector.opening_detected and
                    self.detector.detected_opening_pos is not None):
                gx, gy, _gz = self.detector.detected_opening_pos
                dx = gx - self.uav_pose[0]
                dy = gy - self.uav_pose[1]
                yaw = self.entry_yaw if self.entry_yaw is not None else self.uav_yaw
                c, sn = math.cos(yaw), math.sin(yaw)
                perceived_lat = -dx * sn + dy * c
                if lat_err is None:
                    lat_err = perceived_lat
                else:
                    # The detector REFINES the configured door position; it does not replace
                    # it. Letting it replace the prior deadlocked the approach on 2026-09-05:
                    # the vehicle sat at camera_init y=0.01 -- 1 cm off the centreline -- while
                    # the detector insisted the gap was 0.25 m away, which is outside
                    # align_tolerance, so forward motion was held indefinitely and the vehicle
                    # stalled in the doorway. The arena geometry is known exactly and the
                    # vehicle localises to 0.05-0.10 m, so the prior is the better estimate;
                    # perception can nudge it, bounded, in case the real door is offset.
                    corr = perceived_lat - lat_err
                    lim = self.detector_refine_limit
                    if abs(corr) > lim:
                        rospy.logwarn_throttle(
                            5.0, "[EDM] Gap detection %.2f m disagrees with the configured "
                                 "door centre by %.2f m; clamping the refinement to %.2f m.",
                            perceived_lat, corr, lim)
                        corr = math.copysign(lim, corr)
                    lat_err = lat_err + corr

            if lat_err is None:
                # No door prior and no detection: creeping forward blind is what put the
                # vehicle into the wall. Hold instead.
                rospy.logerr_throttle(5.0, "[EDM] No door reference available; holding.")
                self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))
            else:
                # ALIGN BEFORE ADVANCING. The opening leaves a 0.767 m airframe only about
                # +-0.6 m of true lateral clearance, so advancing while misaligned spends
                # clearance the vehicle does not have. The lateral coordinate itself is not
                # servoed here -- create_approach_cmd() commands the centreline absolutely.
                # Hysteresis on the advance gate: without it the vehicle chatters between
                # advancing and holding as it settles across the threshold.
                thresh = (self.align_release if self.approach_advancing
                          else self.align_tolerance)
                advance = abs(lat_err) <= thresh
                self.approach_advancing = advance
                if not advance:
                    rospy.logwarn_throttle(
                        3.0, "[EDM] Aligning: lateral error %.2f m > %.2f m tolerance; holding forward motion.",
                        lat_err, self.align_tolerance)
                elif not self.approach_ready_logged:
                    self.approach_ready_logged = True
                    rospy.loginfo("[EDM] Aligned on the door centreline (lateral error "
                                  "%.2f m). Advancing at %.2f m/s.",
                                  lat_err, self.search_forward_speed)
                self.pub_pos_cmd.publish(self.create_approach_cmd(advance, dt))

            # Geometric crossing test is the AUTHORITATIVE entry criterion: the vehicle is
            # inside once it is entry_confirm_depth past the door line. The perception
            # confidence score is kept as a secondary trigger, but it was tuned against a
            # 0.5-1.2 m doorway that no longer matches this arena, so it is not trusted alone.
            crossed = (self.entry_line_camera_x is not None and
                       self.uav_pose[0] > self.entry_line_camera_x + self.entry_confirm_depth)

            if crossed:
                rospy.loginfo("[EDM] Door crossed: camera_init x=%.2f > %.2f. Entering arena.",
                              self.uav_pose[0], self.entry_line_camera_x + self.entry_confirm_depth)
                self.transition_to(MissionState.ENTRY_CONFIRMATION)
            elif confirmed:
                rospy.loginfo(f"[EDM] Entry confirmed after {elapsed:.1f}s in ENTRY_SEARCH (Conf: {conf:.2f}).")
                self.transition_to(MissionState.ENTRY_CONFIRMATION)
            elif elapsed > self.max_search_duration:
                # Previously this forced the transition to ENTRY_CONFIRMATION, which handed
                # control to FUEL while the vehicle was very possibly still OUTSIDE the arena
                # and nowhere near a door. Hold position instead and keep saying so: a timeout
                # means the search failed, and failing loudly in place is the safe outcome.
                rospy.logerr_throttle(
                    5.0,
                    "[EDM] ENTRY SEARCH TIMEOUT after %.1fs with no door crossing "
                    "(camera_init x=%.2f, target >%.2f). Holding position; NOT handing over to "
                    "FUEL. Check the door line in mission_config.yaml and the spawn pose."
                    % (elapsed, self.uav_pose[0],
                       (self.entry_line_camera_x + self.entry_confirm_depth)
                       if self.entry_line_camera_x is not None else float('nan')))
                self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))

        elif self.state == MissionState.ENTRY_CONFIRMATION:
            # Hold position while waiting for FUEL to be ready to receive the handover.
            self.pub_pos_cmd.publish(self.create_position_cmd(0.0, 0.0, dt))

            if elapsed < 0.5:
                return

            # Do not hand over into the void. exploration_node may still be starting up.
            n_sub = self.pub_fuel_trigger.get_num_connections()
            if n_sub < 1:
                rospy.logwarn_throttle(
                    5.0, "[EDM] Entry complete; holding for FUEL to come up "
                         "(no subscriber on /waypoint_generator/waypoints after %.1fs).", elapsed)
                if elapsed > self.fuel_trigger_timeout:
                    rospy.logerr_throttle(
                        10.0, "[EDM] FUEL has not subscribed after %.0fs. Still holding inside "
                              "the arena; exploration will NOT start. Check that "
                              "nidar_fuel_upstream.launch came up.", self.fuel_trigger_timeout)
                return

            self.trigger_fuel_exploration()
            self.transition_to(MissionState.EXPLORATION)

        elif self.state == MissionState.EXPLORATION:
            # EDM does not COMMAND during exploration - control is 100% FUEL's - but it does
            # watch, so that when FUEL is done there is a trail to follow home.
            self._record_breadcrumb()
            rospy.loginfo_throttle(
                10.0, "[EDM] Mission State: EXPLORATION (FUEL active, EDM passive). "
                      "return trail: %d crumbs", len(self.breadcrumbs))

        elif self.state == MissionState.RETURN:
            self.run_return(dt)

        elif self.state == MissionState.DESCEND:
            self.run_descend(dt)

        elif self.state == MissionState.LAND:
            self.run_land()


if __name__ == '__main__':
    try:
        node = EntryDetectionModuleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
