#!/usr/bin/env python3
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from bspline.msg import Bspline
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, SetModeRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool, Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class FuelToMavrosBridge:
    def __init__(self):
        rospy.init_node('fuel_to_mavros_bridge', anonymous=False)

        self.completed = False
        self.current_pose = None
        self.latest_imu = None
        self.latest_pos_cmd = None
        self.latest_pos_cmd_time = rospy.Time(0)
        self.latest_bspline_target = None
        self.is_armed = False
        self.prev_armed = False
        self.current_mode = ""
        self.last_mode_request_time = rospy.Time(0)
        self.bridge_start_time = rospy.Time.now()
        self.home_x = None
        self.home_y = None
        self.yaw_ref_locked = False
        self.yaw_ref = 0.0
        self.forced_heading_yaw = rospy.get_param('~forced_heading_yaw', 999.0)
        self.bootstrap_started = False
        self.bootstrap_start_time = rospy.Time(0)
        self.arena_min_x = rospy.get_param('~arena_min_x', -14.5)
        self.arena_max_x = rospy.get_param('~arena_max_x', 14.5)
        self.arena_min_y = rospy.get_param('~arena_min_y', -7.0)
        self.arena_max_y = rospy.get_param('~arena_max_y', 14.5)
        self.min_forward_distance = rospy.get_param('~min_forward_distance', 0.2)
        self.max_lateral_ratio = rospy.get_param('~max_lateral_ratio', 0.75)
        self.forward_corridor_width = rospy.get_param('~forward_corridor_width', 0.8)
        self.max_forward_step = rospy.get_param('~max_forward_step', 2.5)
        self.yaw_align_threshold = rospy.get_param('~yaw_align_threshold', 0.25)
        self.enable_yaw_align_gate = rospy.get_param('~enable_yaw_align_gate', False)
        self.bootstrap_duration = rospy.get_param('~bootstrap_duration', 20.0)
        self.bootstrap_forward_step = rospy.get_param('~bootstrap_forward_step', 0.8)
        self.bootstrap_corridor_half_width = rospy.get_param('~bootstrap_corridor_half_width', 1.2)
        self.enable_bootstrap_nudge = rospy.get_param('~enable_bootstrap_nudge', False)
        self.deactivate_on_completion = rospy.get_param('~deactivate_on_completion', False)
        self.min_completion_time = rospy.get_param('~min_completion_time', 180.0)
        self.min_completion_distance = rospy.get_param('~min_completion_distance', 4.0)
        self.enforce_forward_only = rospy.get_param('~enforce_forward_only', False)
        self.forward_min_step = rospy.get_param('~forward_min_step', 0.30)
        self.forward_replan_step = rospy.get_param('~forward_replan_step', 1.00)
        self.backward_tolerance = rospy.get_param('~backward_tolerance', 0.02)
        self.max_lateral_deviation = rospy.get_param('~max_lateral_deviation', 2.0)
        self.replan_lateral_limit = rospy.get_param('~replan_lateral_limit', 3.5)
        self.backward_imu_accel_threshold = rospy.get_param('~backward_imu_accel_threshold', -0.8)
        self.gate_entry_forward_distance = rospy.get_param('~gate_entry_forward_distance', 0.5)
        self.gate_entry_max_step = rospy.get_param('~gate_entry_max_step', 0.45)
        self.gate_entry_min_z = rospy.get_param('~gate_entry_min_z', 1.3)
        self.safety_grid_resolution = rospy.get_param('~safety_grid_resolution', 0.2)
        self.safety_grid_origin_x = rospy.get_param('~safety_grid_origin_x', -16.0)
        self.safety_grid_origin_y = rospy.get_param('~safety_grid_origin_y', -16.0)
        self.safety_grid_size_x = rospy.get_param('~safety_grid_size_x', 32.0)
        self.safety_grid_size_y = rospy.get_param('~safety_grid_size_y', 32.0)
        self.safety_inflation_cells = rospy.get_param('~safety_inflation_cells', 2)
        self.localization_reject_threshold = rospy.get_param('~localization_reject_threshold', 8)
        self.enable_line_check = rospy.get_param('~enable_line_check', False)

        self.origin_locked = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_z = 0.0
        self.max_forward_progress = 0.0
        self.localization_healthy = True
        self.localization_rejected_count = 0

        self.grid_width = int(self.safety_grid_size_x / self.safety_grid_resolution)
        self.grid_height = int(self.safety_grid_size_y / self.safety_grid_resolution)
        self.obstacles = set()
        self.inflated_obstacles = set()

        if abs(self.forced_heading_yaw) <= math.pi + 0.2:
            self.yaw_ref = self.forced_heading_yaw
            self.yaw_ref_locked = True
            rospy.loginfo(f"[Bridge] Using forced heading yaw {self.yaw_ref:.3f} rad")

        if self.enable_yaw_align_gate:
            rospy.loginfo("[Bridge] Yaw-align gate is ENABLED")
        else:
            rospy.logwarn("[Bridge] Yaw-align gate is DISABLED (direct tracking mode)")

        self.sub_state = rospy.Subscriber(
            '/mavros/state',
            State,
            self.state_callback,
            queue_size=1
        )

        self.sub_completed = rospy.Subscriber(
            '/exploration_completed',
            Bool,
            self.completed_callback,
            queue_size=1
        )

        self.sub_cmd = rospy.Subscriber(
            '/planning/pos_cmd', 
            PositionCommand, 
            self.pos_cmd_callback, 
            queue_size=10
        )

        self.sub_pose = rospy.Subscriber(
            '/mavros/local_position/pose',
            PoseStamped,
            self.pose_callback,
            queue_size=1
        )

        self.sub_imu = rospy.Subscriber(
            '/mavros/imu/data',
            Imu,
            self.imu_callback,
            queue_size=5
        )

        self.sub_bspline = rospy.Subscriber(
            '/planning/bspline',
            Bspline,
            self.bspline_callback,
            queue_size=5
        )

        self.sub_occ = rospy.Subscriber(
            '/sdf_map/occupancy_all',
            PointCloud2,
            self.occupancy_callback,
            queue_size=1
        )

        self.sub_loc_health = rospy.Subscriber(
            '/localization/healthy',
            Bool,
            self.localization_health_callback,
            queue_size=5
        )

        self.sub_loc_rejected = rospy.Subscriber(
            '/localization/rejected_count',
            Int32,
            self.localization_rejected_callback,
            queue_size=5
        )

        self.pub_setpoint_raw = rospy.Publisher(
            '/mavros/setpoint_raw/local', 
            PositionTarget, 
            queue_size=10
        )
        
        self.pub_setpoint_pose = rospy.Publisher(
            '/mavros/setpoint_position/local', 
            PoseStamped, 
            queue_size=10
        )

        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Continuous setpoint timer running at 20Hz for rock-solid OFFBOARD stability
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        rospy.loginfo("FUEL to MAVROS Setpoint Bridge initialized with continuous 20Hz setpoint streaming.")

    def state_callback(self, msg):
        self.prev_armed = self.is_armed
        self.is_armed = msg.armed
        self.current_mode = msg.mode

        # Arm-time lock for mission origin and forward heading reference.
        if (not self.prev_armed) and self.is_armed and self.current_pose is not None:
            self.lock_origin_and_heading("arm")

        # Prepare for a fresh mission origin when disarmed.
        if self.prev_armed and (not self.is_armed):
            self.origin_locked = False
            self.max_forward_progress = 0.0

    def imu_callback(self, msg):
        self.latest_imu = msg

    def localization_health_callback(self, msg):
        self.localization_healthy = bool(msg.data)

    def localization_rejected_callback(self, msg):
        self.localization_rejected_count = int(msg.data)

    def metric_to_grid(self, x, y):
        gx = int(round((x - self.safety_grid_origin_x) / self.safety_grid_resolution))
        gy = int(round((y - self.safety_grid_origin_y) / self.safety_grid_resolution))
        gx = max(0, min(self.grid_width - 1, gx))
        gy = max(0, min(self.grid_height - 1, gy))
        return gx, gy

    def inflate_obstacles(self):
        inflated = set()
        r = self.safety_inflation_cells
        for gx, gy in self.obstacles:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx * dx + dy * dy > r * r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                        inflated.add((nx, ny))
        self.inflated_obstacles = inflated

    def occupancy_callback(self, msg):
        new_obs = set()
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if 0.2 <= p[2] <= 2.5:
                gx, gy = self.metric_to_grid(p[0], p[1])
                new_obs.add((gx, gy))
        self.obstacles = new_obs
        self.inflate_obstacles()

        self.enable_line_check = rospy.get_param('~enable_line_check', False)

    def line_blocked(self, x0, y0, x1, y1):
        if not self.enable_line_check or not self.inflated_obstacles:
            return False

        gx0, gy0 = self.metric_to_grid(x0, y0)
        gx1, gy1 = self.metric_to_grid(x1, y1)

        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy
        x = gx0
        y = gy0

        while True:
            if (x, y) in self.inflated_obstacles:
                return True
            if x == gx1 and y == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return False

    def lock_origin_and_heading(self, reason):
        if self.current_pose is None:
            return

        self.origin_x = self.current_pose.pose.position.x
        self.origin_y = self.current_pose.pose.position.y
        self.origin_z = self.current_pose.pose.position.z
        self.origin_locked = True

        # Unless explicitly forced, forward reference is arm-time heading.
        if abs(self.forced_heading_yaw) > math.pi + 0.2:
            self.yaw_ref = self.get_current_yaw()
            self.yaw_ref_locked = True

        self.max_forward_progress = 0.0
        rospy.loginfo(
            "[Bridge] Origin locked (%s): x=%.2f y=%.2f z=%.2f, forward_yaw=%.3f",
            reason,
            self.origin_x,
            self.origin_y,
            self.origin_z,
            self.yaw_ref,
        )

    def get_forward_lateral(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        fwd = math.cos(self.yaw_ref) * dx + math.sin(self.yaw_ref) * dy
        lat = -math.sin(self.yaw_ref) * dx + math.cos(self.yaw_ref) * dy
        return fwd, lat

    def get_forward_velocity(self, vx, vy):
        return math.cos(self.yaw_ref) * vx + math.sin(self.yaw_ref) * vy

    def build_forward_replan_target(self, candidate_x, candidate_y, candidate_z):
        if self.current_pose is None:
            return None

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_fwd, _ = self.get_forward_lateral(curr_x, curr_y)
        target_fwd = max(curr_fwd + self.forward_replan_step, self.max_forward_progress + self.forward_min_step)

        # Preserve planner lateral intent while enforcing non-negative forward progress.
        _, candidate_lat = self.get_forward_lateral(candidate_x, candidate_y)
        target_lat = max(-self.replan_lateral_limit, min(self.replan_lateral_limit, candidate_lat))

        target_x = self.origin_x + target_fwd * math.cos(self.yaw_ref) - target_lat * math.sin(self.yaw_ref)
        target_y = self.origin_y + target_fwd * math.sin(self.yaw_ref) + target_lat * math.cos(self.yaw_ref)
        target_z = min(1.5, max(1.0, candidate_z))
        return target_x, target_y, target_z

    def clamp_to_arena_bounds(self, candidate_x, candidate_y, candidate_z):
        if not self.origin_locked or self.current_pose is None:
            return candidate_x, candidate_y, candidate_z

        fwd, lat = self.get_forward_lateral(candidate_x, candidate_y)
        curr_fwd, _ = self.get_forward_lateral(self.current_pose.pose.position.x, self.current_pose.pose.position.y)

        # Strict Arena Boundary Clamps in Forward/Lateral Body Frame:
        # fwd >= -0.3m (never move backward out the entrance door)
        # fwd <= 13.5m (never move past far warehouse wall)
        # lat in [-6.2m, 6.2m] (never hit left/right warehouse walls)
        fwd_clamped = max(-0.3, min(13.5, fwd))

        # Multi-stage arena geometry safety clamp matching physical warehouse structure:
        # Stage 1: Entrance Doorway (fwd < 0.8m) -> restrict lateral to door width [-0.6m, 0.6m]
        # Stage 2: Entrance Alcove (0.8m <= fwd < 2.5m) -> restrict lateral to alcove width [-1.8m, 1.8m]
        # Stage 3: Main Warehouse Arena (fwd >= 2.5m) -> allow full warehouse width [-6.0m, 6.0m]
        effective_fwd = min(curr_fwd, fwd_clamped)
        if effective_fwd < 0.8:
            lat_clamped = max(-0.6, min(0.6, lat))
        elif effective_fwd < 2.5:
            lat_clamped = max(-1.8, min(1.8, lat))
        else:
            lat_clamped = max(-6.0, min(6.0, lat))

        # Convert clamped fwd/lat back to map coordinates
        clamped_x = self.origin_x + fwd_clamped * math.cos(self.yaw_ref) - lat_clamped * math.sin(self.yaw_ref)
        clamped_y = self.origin_y + fwd_clamped * math.sin(self.yaw_ref) + lat_clamped * math.cos(self.yaw_ref)

        # Hard Arena Boundary Gate Clamp (protecting gate at Y = -7.0m)
        if hasattr(self, 'arena_min_y') and self.arena_min_y is not None:
            clamped_y = max(self.arena_min_y, clamped_y)
        else:
            clamped_y = max(-7.0, clamped_y)

        if hasattr(self, 'arena_max_y') and self.arena_max_y is not None:
            clamped_y = min(self.arena_max_y, clamped_y)
        if hasattr(self, 'arena_min_x') and self.arena_min_x is not None:
            clamped_x = max(self.arena_min_x, clamped_x)
        if hasattr(self, 'arena_max_x') and self.arena_max_x is not None:
            clamped_x = min(self.arena_max_x, clamped_x)

        # Safe Cruising Altitude Clamp: Z in [1.0m, 1.8m] (clears all 0.5m-0.7m warehouse obstacles)
        clamped_z = max(1.0, min(1.8, candidate_z))

        return clamped_x, clamped_y, clamped_z


    def enforce_forward_policy(self, candidate_x, candidate_y, candidate_z, cmd_vx, cmd_vy):
        if not self.enforce_forward_only or self.current_pose is None:
            return candidate_x, candidate_y, candidate_z, False

        if not self.origin_locked:
            self.lock_origin_and_heading("first-command")
        if not self.origin_locked:
            return candidate_x, candidate_y, candidate_z, False

        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y
        curr_fwd, _ = self.get_forward_lateral(curr_x, curr_y)
        tgt_fwd, tgt_lat = self.get_forward_lateral(candidate_x, candidate_y)
        fwd_vel = self.get_forward_velocity(cmd_vx, cmd_vy)

        if curr_fwd > self.max_forward_progress:
            self.max_forward_progress = curr_fwd

        backward_by_target = tgt_fwd < (curr_fwd - self.backward_tolerance)
        backward_vs_progress = tgt_fwd < (self.max_forward_progress - self.backward_tolerance)
        backward_by_velocity = fwd_vel < (-self.backward_tolerance)
        if backward_by_target or backward_vs_progress or backward_by_velocity:
            replanned = self.build_forward_replan_target(candidate_x, candidate_y, candidate_z)
            if replanned is None:
                return candidate_x, candidate_y, candidate_z, False
            rx, ry, rz = replanned
            rospy.logwarn_throttle(
                1.0,
                "[Bridge] Forward-only filter rejected cmd (curr_f=%.2f max_f=%.2f tgt_f=%.2f vf=%.2f lat=%.2f). Replanned -> (%.2f, %.2f, %.2f)",
                curr_fwd,
                self.max_forward_progress,
                tgt_fwd,
                fwd_vel,
                tgt_lat,
                rx,
                ry,
                rz,
            )
            return rx, ry, rz, True

        return candidate_x, candidate_y, candidate_z, False

    def completed_callback(self, msg):
        if not msg.data or self.completed:
            return

        if not self.deactivate_on_completion:
            rospy.logwarn_throttle(2.0, "[Bridge] Ignoring exploration_completed to avoid false bridge shutdown")
            return

        elapsed = (rospy.Time.now() - self.bridge_start_time).to_sec() if self.bridge_start_time else 0.0
        if elapsed < self.min_completion_time:
            rospy.logwarn_throttle(2.0, f"[Bridge] Ignored early completion signal (elapsed {elapsed:.1f}s < {self.min_completion_time:.1f}s)")
            return

        if self.current_pose is not None and self.home_x is not None and self.home_y is not None:
            traveled = math.hypot(self.current_pose.pose.position.x - self.home_x,
                                  self.current_pose.pose.position.y - self.home_y)
            if traveled < self.min_completion_distance:
                rospy.logwarn_throttle(2.0, f"[Bridge] Ignored completion: traveled only {traveled:.2f}m")
                return

        rospy.loginfo("[Bridge] Exploration completion validated. Deactivating bridge relay.")
        self.completed = True

    def command_is_safe(self, target_x, target_y):
        if self.current_pose is None:
            return True

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        dx = target_x - current_x
        dy = target_y - current_y

        step = math.hypot(dx, dy)
        if step > (self.max_forward_step * 3.0):
            rospy.logwarn_throttle(1.0, f"[Bridge] Blocking overly long candidate step: {step:.2f}m")
            return False

        if not (self.arena_min_x <= target_x <= self.arena_max_x and
                self.arena_min_y <= target_y <= self.arena_max_y):
            rospy.logwarn_throttle(1.0, f"[Bridge] Blocking out-of-bounds command: ({target_x:.2f}, {target_y:.2f})")
            return False

        if self.line_blocked(current_x, current_y, target_x, target_y):
            rospy.logwarn_throttle(1.0, f"[Bridge] Blocking collision path to ({target_x:.2f}, {target_y:.2f})")
            return False

        return True

    def build_hover_target(self):
        if self.current_pose is None:
            return None, None

        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.position.x = self.current_pose.pose.position.x
        target.position.y = self.current_pose.pose.position.y
        target.position.z = max(1.0, min(1.5, self.current_pose.pose.position.z))
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = 0.0
        target.yaw = self.get_current_yaw()
        target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

        pose = PoseStamped()
        pose.header = target.header
        pose.pose.position = target.position
        q = quaternion_from_euler(0, 0, target.yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return target, pose

    def get_current_yaw(self):
        if self.current_pose is None:
            return 0.0

        q = [
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def build_heading_first_target(self, candidate_x, candidate_y, candidate_z):
        if self.current_pose is None:
            return None

        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_yaw = self.get_current_yaw()

        desired_yaw = math.atan2(candidate_y - current_y, candidate_x - current_x)
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)

        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        pose = PoseStamped()
        pose.header = target.header

        # First rotate to face the direction of motion, then translate forward.
        if abs(yaw_error) > self.yaw_align_threshold:
            target.position.x = current_x
            target.position.y = current_y
            target.position.z = min(1.5, max(1.0, candidate_z))
            target.velocity.x = 0.0
            target.velocity.y = 0.0
            target.velocity.z = 0.0
            target.yaw = desired_yaw
            target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

            q = quaternion_from_euler(0, 0, desired_yaw)
            pose.pose.position.x = target.position.x
            pose.pose.position.y = target.position.y
            pose.pose.position.z = target.position.z
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            return target, pose, False, desired_yaw, yaw_error

        forward_step = min(self.max_forward_step, math.hypot(candidate_x - current_x, candidate_y - current_y))
        if forward_step < self.min_forward_distance:
            forward_step = self.min_forward_distance

        target.position.x = current_x + forward_step * math.cos(desired_yaw)
        target.position.y = current_y + forward_step * math.sin(desired_yaw)
        target.position.z = min(1.5, max(1.0, candidate_z))
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = 0.0
        target.yaw = desired_yaw
        target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

        q = quaternion_from_euler(0, 0, desired_yaw)
        pose.pose.position.x = target.position.x
        pose.pose.position.y = target.position.y
        pose.pose.position.z = target.position.z
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return target, pose, True, desired_yaw, yaw_error

    def pose_callback(self, msg):
        self.current_pose = msg
        if self.home_x is None:
            self.home_x = msg.pose.position.x
            self.home_y = msg.pose.position.y
        if not self.yaw_ref_locked:
            q = [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            _, _, self.yaw_ref = euler_from_quaternion(q)
            self.yaw_ref_locked = True
            rospy.loginfo(f"[Bridge] Locked forward reference yaw at {self.yaw_ref:.3f} rad")

        if self.is_armed and not self.origin_locked:
            self.lock_origin_and_heading("pose-update")

    def pos_cmd_callback(self, msg):
        if not self.completed:
            self.latest_pos_cmd = msg
            self.latest_pos_cmd_time = rospy.Time.now()
            if not self.bootstrap_started:
                self.bootstrap_started = True
                self.bootstrap_start_time = rospy.Time.now()

    def bspline_callback(self, msg):
        if self.completed or not msg.pos_pts:
            return

        # Prefer the first control point that is clearly ahead of the vehicle.
        # This keeps the fallback moving forward even when the pos_cmd stream is stale.
        candidate = None
        if self.current_pose is not None:
            yaw = self.get_current_yaw()

            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y
            for point in msg.pos_pts:
                dx = point.x - current_x
                dy = point.y - current_y
                forward = math.cos(yaw) * dx + math.sin(yaw) * dy
                lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
                if forward > self.min_forward_distance and abs(lateral) <= self.forward_corridor_width:
                    candidate = point
                    break

        if candidate is None:
            idx = max(0, min(len(msg.pos_pts) - 1, len(msg.pos_pts) // 2))
            candidate = msg.pos_pts[idx]

        self.latest_bspline_target = candidate

    def timer_callback(self, event):
        if self.completed:
            return

        # Keep OFFBOARD mode latched during autonomous run.
        # Request periodically to recover from AUTO.LOITER or callback desync.
        if self.current_pose is not None and not self.completed:
            now = rospy.Time.now()
            if self.last_mode_request_time == rospy.Time(0) or (now - self.last_mode_request_time).to_sec() > 1.0:
                req = SetModeRequest()
                req.custom_mode = 'OFFBOARD'
                try:
                    resp = self.set_mode_client.call(req)
                    if resp.mode_sent:
                        rospy.loginfo_throttle(2.0, "[Bridge] Requested OFFBOARD mode")
                    else:
                        rospy.logwarn_throttle(2.0, "[Bridge] OFFBOARD mode request rejected")
                except rospy.ServiceException as exc:
                    rospy.logwarn_throttle(2.0, f"[Bridge] OFFBOARD request failed: {exc}")
                self.last_mode_request_time = now

        target = PositionTarget()
        target.header.stamp = rospy.Time.now()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        pose = PoseStamped()
        pose.header = target.header

        pos_cmd_fresh = (
            self.latest_pos_cmd is not None and
            self.latest_pos_cmd_time != rospy.Time(0) and
            (rospy.Time.now() - self.latest_pos_cmd_time).to_sec() < 0.8
        )

        using_bspline_fallback = False
        if pos_cmd_fresh:
            # Position from latest FUEL command
            candidate_x = self.latest_pos_cmd.position.x
            candidate_y = self.latest_pos_cmd.position.y
            candidate_z = self.latest_pos_cmd.position.z
            cmd_vx = self.latest_pos_cmd.velocity.x
            cmd_vy = self.latest_pos_cmd.velocity.y
            cmd_vz = self.latest_pos_cmd.velocity.z
            cmd_yaw = self.latest_pos_cmd.yaw
        elif self.latest_bspline_target is not None and self.current_pose is not None:
            # Fallback path: follow bspline when traj_server does not emit pos_cmd.
            using_bspline_fallback = True
            candidate_x = self.latest_bspline_target.x
            candidate_y = self.latest_bspline_target.y
            candidate_z = self.latest_bspline_target.z
            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_vz = 0.0

            q = [
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ]
            _, _, cmd_yaw = euler_from_quaternion(q)
        else:
            # Initial hold target: pull forward into arena at Y=0.50m and altitude Z=1.30m
            candidate_x = 0.0
            candidate_y = 0.50
            candidate_z = 1.30
            cmd_vx = cmd_vy = cmd_vz = 0.0
            cmd_yaw = self.yaw_ref

        if candidate_x is not None:
            if using_bspline_fallback:
                rospy.logwarn_throttle(2.0, "[Bridge] Using bspline fallback (pos_cmd stream stale)")

            if self.current_pose is None:
                return

            # IMU-based backward acceleration warning for runtime verification.
            if self.latest_imu is not None and self.latest_imu.linear_acceleration.x < self.backward_imu_accel_threshold:
                rospy.logwarn_throttle(
                    1.0,
                    "[Bridge] IMU indicates backward acceleration ax=%.2f m/s^2",
                    self.latest_imu.linear_acceleration.x,
                )

            candidate_x, candidate_y, candidate_z, replanned = self.enforce_forward_policy(
                candidate_x,
                candidate_y,
                candidate_z,
                cmd_vx,
                cmd_vy,
            )

            # Enforce arena physical 3D wall bounds (fwd in [-0.3m, 13.5m], lat in [-6.2m, 6.2m], Z in [0.4m, 1.8m])
            candidate_x, candidate_y, candidate_z = self.clamp_to_arena_bounds(candidate_x, candidate_y, candidate_z)

            if self.current_pose is not None and self.origin_locked:
                curr_fwd, _ = self.get_forward_lateral(self.current_pose.pose.position.x, self.current_pose.pose.position.y)
                if curr_fwd < self.gate_entry_forward_distance:
                    dx = candidate_x - self.current_pose.pose.position.x
                    dy = candidate_y - self.current_pose.pose.position.y
                    dist = math.hypot(dx, dy)
                    if dist > self.gate_entry_max_step and dist > 1e-3:
                        scale = self.gate_entry_max_step / dist
                        candidate_x = self.current_pose.pose.position.x + dx * scale
                        candidate_y = self.current_pose.pose.position.y + dy * scale
                    candidate_z = max(self.gate_entry_min_z, min(1.8, candidate_z))
                    candidate_x, candidate_y, candidate_z = self.clamp_to_arena_bounds(candidate_x, candidate_y, candidate_z)

            if (not self.localization_healthy) or (self.localization_rejected_count >= self.localization_reject_threshold):
                rospy.logwarn_throttle(
                    1.0,
                    "[Bridge] Localization unhealthy (healthy=%s rejected=%d). Holding position.",
                    str(self.localization_healthy),
                    self.localization_rejected_count,
                )
                target, pose = self.build_hover_target()
                if target is None:
                    return
                pose.pose.position = target.position
                self.pub_setpoint_pose.publish(pose)
                return

            if self.command_is_safe(candidate_x, candidate_y):
                if self.enable_yaw_align_gate:
                    built = self.build_heading_first_target(candidate_x, candidate_y, candidate_z)
                    if built is None:
                        return

                    target, pose, moving_forward, desired_yaw, yaw_error = built

                    if not moving_forward:
                        rospy.logwarn_throttle(1.0, f"[Bridge] Yaw-align first: error={yaw_error:.2f}rad desired={desired_yaw:.2f}rad")
                    else:
                        rospy.loginfo_throttle(2.0, f"[Bridge] Heading-first step: forward target ({target.position.x:.2f}, {target.position.y:.2f})")
                else:
                    # Direct tracking mode: do not block translation on yaw alignment.
                    target.position.x = candidate_x
                    target.position.y = candidate_y
                    target.position.z = min(1.5, max(1.0, candidate_z))
                    target.velocity.x = 0.0
                    target.velocity.y = 0.0
                    target.velocity.z = 0.0
                    target.yaw = cmd_yaw
                    target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

                    q = quaternion_from_euler(0, 0, cmd_yaw)
                    pose.pose.position.x = target.position.x
                    pose.pose.position.y = target.position.y
                    pose.pose.position.z = target.position.z
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    rospy.loginfo_throttle(2.0, f"[Bridge] Direct-tracking target ({target.position.x:.2f}, {target.position.y:.2f}, {target.position.z:.2f})")
            else:
                # During initial bootstrap, bias motion forward into the door corridor.
                in_bootstrap = self.bootstrap_started and (rospy.Time.now() - self.bootstrap_start_time).to_sec() < self.bootstrap_duration
                if self.enable_bootstrap_nudge and in_bootstrap and self.current_pose is not None and self.yaw_ref_locked:
                    forward_x = self.current_pose.pose.position.x + self.bootstrap_forward_step * math.cos(self.yaw_ref)
                    forward_y = self.current_pose.pose.position.y + self.bootstrap_forward_step * math.sin(self.yaw_ref)
                    built = self.build_heading_first_target(forward_x, forward_y, candidate_z)
                    if built is None:
                        return
                    target, pose, moving_forward, desired_yaw, yaw_error = built
                    rospy.logwarn_throttle(1.0, "[Bridge] Bootstrap heading-first nudge applied")
                else:
                    # Hold current position and yaw if candidate is unsafe.
                    target = PositionTarget()
                    target.header.stamp = rospy.Time.now()
                    target.header.frame_id = "map"
                    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                    target.position.x = self.current_pose.pose.position.x
                    target.position.y = self.current_pose.pose.position.y
                    target.position.z = self.current_pose.pose.position.z
                    target.velocity.x = 0.0
                    target.velocity.y = 0.0
                    target.velocity.z = 0.0
                    hold_yaw = self.get_current_yaw()
                    target.yaw = hold_yaw
                    target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048

                    pose = PoseStamped()
                    pose.header = target.header
                    pose.pose.position = target.position
                    q = quaternion_from_euler(0, 0, hold_yaw)
                    pose.pose.orientation.x = q[0]
                    pose.pose.orientation.y = q[1]
                    pose.pose.orientation.z = q[2]
                    pose.pose.orientation.w = q[3]
                    rospy.logwarn_throttle(1.0, "[Bridge] Unsafe candidate blocked: holding position")
        else:
            if self.current_pose is None:
                return

            # Standby setpoint: if armed, climb to 1.5m hover; if disarmed, hold current pose
            target.position.x = self.current_pose.pose.position.x
            target.position.y = self.current_pose.pose.position.y
            target.position.z = 1.5 if self.is_armed else self.current_pose.pose.position.z

            q = [
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ]
            _, _, yaw = euler_from_quaternion(q)
            target.yaw = yaw

            # Type mask: Position + Yaw
            target.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048
            pose.pose.orientation = self.current_pose.pose.orientation

        pose.pose.position = target.position

        # Publish setpoint_position/local for rock-solid PX4 OFFBOARD position control
        self.pub_setpoint_pose.publish(pose)

if __name__ == '__main__':
    try:
        bridge = FuelToMavrosBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
