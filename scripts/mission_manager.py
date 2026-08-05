#!/usr/bin/env python3
import rospy
import math
import heapq
from collections import deque
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest

class GridMap:
    def __init__(self, resolution=0.2, origin_x=-12.0, origin_y=-12.0, size_x=24.0, size_y=24.0):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = int(size_x / resolution)
        self.height = int(size_y / resolution)
        self.obstacles = set()
        self.inflated_obstacles = set()
        self.inflation_radius_cells = 2  # 2 cells * 0.2m = 0.4m safety inflation

    def update_from_pc(self, pc_msg):
        new_obstacles = set()
        for p in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
            # Focus on obstacles within height limits where the drone or its body might collide
            if 0.2 <= p[2] <= 2.5:
                gx = int(round((p[0] - self.origin_x) / self.resolution))
                gy = int(round((p[1] - self.origin_y) / self.resolution))
                if 0 <= gx < self.width and 0 <= gy < self.height:
                    new_obstacles.add((gx, gy))
        self.obstacles = new_obstacles
        self.inflate()

    def inflate(self):
        inflated = set()
        r = self.inflation_radius_cells
        for (gx, gy) in self.obstacles:
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if dx*dx + dy*dy <= r*r:
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.width and 0 <= ny < self.height:
                            inflated.add((nx, ny))
        self.inflated_obstacles = inflated

    def is_occupied(self, gx, gy):
        return (gx, gy) in self.inflated_obstacles

    def metric_to_grid(self, x, y):
        gx = int(round((x - self.origin_x) / self.resolution))
        gy = int(round((y - self.origin_y) / self.resolution))
        # Clamp to bounds
        gx = max(0, min(self.width - 1, gx))
        gy = max(0, min(self.height - 1, gy))
        return (gx, gy)

    def grid_to_metric(self, gx, gy):
        x = gx * self.resolution + self.origin_x
        y = gy * self.resolution + self.origin_y
        return (x, y)


def find_nearest_unoccupied(grid_map, cell):
    if not grid_map.is_occupied(cell[0], cell[1]):
        return cell
    queue = deque([cell])
    visited = {cell}
    while queue:
        curr = queue.popleft()
        if not grid_map.is_occupied(curr[0], curr[1]):
            return curr
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (curr[0] + dx, curr[1] + dy)
                if 0 <= neighbor[0] < grid_map.width and 0 <= neighbor[1] < grid_map.height:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append(neighbor)
    return cell


def astar(grid_map, start, goal):
    start = find_nearest_unoccupied(grid_map, start)
    goal = find_nearest_unoccupied(grid_map, goal)

    def heuristic(p1, p2):
        dx = abs(p1[0] - p2[0])
        dy = abs(p1[1] - p2[1])
        return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), start))
    came_from = {}
    g_score = {start: 0.0}
    closed_set = set()

    motions = [
        (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
    ]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        if current in closed_set:
            continue
        closed_set.add(current)

        for dx, dy, cost in motions:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < grid_map.width and 0 <= neighbor[1] < grid_map.height):
                continue
            if grid_map.is_occupied(neighbor[0], neighbor[1]):
                continue

            tentative_g_score = g_score[current] + cost
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_set, (f_score, neighbor))
    return None


def line_of_sight(grid_map, p1, p2):
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if grid_map.is_occupied(x0, y0):
            return False
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return True


def prune_path(grid_map, path):
    if not path or len(path) <= 2:
        return path
    pruned = [path[0]]
    curr = path[0]
    i = 1
    while i < len(path) - 1:
        if line_of_sight(grid_map, curr, path[i + 1]):
            i += 1
        else:
            pruned.append(path[i])
            curr = path[i]
            i += 1
    pruned.append(path[-1])
    return pruned


class NidarMissionManager:
    def __init__(self):
        rospy.init_node('nidar_mission_manager', anonymous=True)

        self.state = "MONITOR"
        self.home_x = None
        self.home_y = None
        self.home_z = 1.5  # Fixed return and landing starting height

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.pose_received = False
        self.completed = False

        self.grid_map = GridMap()
        self.waypoints = []
        self.current_wp_idx = 0
        self.start_time = None

        # ROS subscribers and publishers
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/exploration_completed', Bool, self.completed_callback)
        rospy.Subscriber('/sdf_map/occupancy_all', PointCloud2, self.map_callback)

        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.completed_pub = rospy.Publisher('/exploration_completed', Bool, queue_size=1)

        # MAVROS mode service
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)
        rospy.loginfo("[Mission Manager] Initialized successfully.")

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        self.pose_received = True

        # Cache takeoff coordinates on first lock
        if self.home_x is None:
            self.home_x = self.current_x
            self.home_y = self.current_y
            rospy.loginfo(f"[Mission Manager] Home coordinate cached: X={self.home_x:.2f}, Y={self.home_y:.2f}")

    def completed_callback(self, msg):
        if msg.data and not self.completed:
            rospy.loginfo("[Mission Manager] Exploration completed signal received!")
            self.completed = True

    def map_callback(self, msg):
        self.grid_map.update_from_pc(msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.state == "MONITOR":
                if self.pose_received and self.start_time is None:
                    now = rospy.Time.now()
                    if now.to_sec() > 1.0:
                        self.start_time = now
                        rospy.loginfo(f"[Mission Manager] Exploration timer started at ROS time: {now.to_sec():.2f}s")

                max_duration = rospy.get_param('~max_exploration_time', 90.0)
                elapsed = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0.0

                if self.completed or (self.start_time and elapsed >= max_duration):
                    if not self.completed:
                        rospy.loginfo(f"[Mission Manager] Max exploration duration ({max_duration:.1f}s) reached!")
                        self.completed = True
                        self.completed_pub.publish(Bool(data=True))

                    rospy.loginfo("[Mission Manager] Transitioning to path planning for Return-to-Home (RTH)...")
                    self.state = "PLAN"
            
            elif self.state == "PLAN":
                # Compute A* path from current position to home position
                start_grid = self.grid_map.metric_to_grid(self.current_x, self.current_y)
                goal_grid = self.grid_map.metric_to_grid(self.home_x, self.home_y)
                
                rospy.loginfo(f"[Mission Manager] Planning A* path from {start_grid} to {goal_grid}...")
                raw_path = astar(self.grid_map, start_grid, goal_grid)
                
                if raw_path:
                    pruned = prune_path(self.grid_map, raw_path)
                    self.waypoints = [self.grid_map.grid_to_metric(gx, gy) for gx, gy in pruned]
                    rospy.loginfo(f"[Mission Manager] Shortest distance path found with {len(self.waypoints)} waypoints!")
                else:
                    rospy.logwarn("[Mission Manager] A* failed. Falling back to straight-line return.")
                    self.waypoints = [(self.home_x, self.home_y)]

                self.current_wp_idx = 0
                self.state = "RTH"

            elif self.state == "RTH":
                # Send setpoint commands to return to home
                if self.current_wp_idx < len(self.waypoints):
                    target_x, target_y = self.waypoints[self.current_wp_idx]
                    
                    # Construct and publish target pose maintaining 1.5m altitude
                    target_pose = PoseStamped()
                    target_pose.header.stamp = rospy.Time.now()
                    target_pose.header.frame_id = "map"
                    target_pose.pose.position.x = target_x
                    target_pose.pose.position.y = target_y
                    target_pose.pose.position.z = 1.5  # Maintain height in all cases
                    target_pose.pose.orientation.w = 1.0
                    self.setpoint_pub.publish(target_pose)

                    # Check distance to waypoint
                    dx = self.current_x - target_x
                    dy = self.current_y - target_y
                    dz = self.current_z - 1.5
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                    rospy.loginfo_throttle(2.0, f"[Mission Manager] Returning: Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} Target=({target_x:.2f}, {target_y:.2f}), Distance error: {dist:.2f}m")

                    if dist < 0.25:
                        rospy.loginfo(f"[Mission Manager] Reached waypoint {self.current_wp_idx + 1}")
                        self.current_wp_idx += 1
                else:
                    rospy.loginfo("[Mission Manager] Reached Home location coordinates. Transitioning to precision landing...")
                    self.land_target_z = 1.5
                    self.state = "PRECISION_LAND"

            elif self.state == "PRECISION_LAND":
                # Descent phase: slowly decrease altitude while locking (home_x, home_y)
                self.land_target_z -= 0.15 / 20.0  # Decrease Z at 0.15 m/s (running at 20Hz)
                if self.land_target_z < 0.2:
                    self.land_target_z = 0.2

                target_pose = PoseStamped()
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = "map"
                target_pose.pose.position.x = self.home_x
                target_pose.pose.position.y = self.home_y
                target_pose.pose.position.z = self.land_target_z
                target_pose.pose.orientation.w = 1.0
                self.setpoint_pub.publish(target_pose)

                rospy.loginfo_throttle(1.0, f"[Mission Manager] Precision Landing: Target Z={self.land_target_z:.2f}m, Current Z={self.current_z:.2f}m")

                # If reached minimum descent altitude of 0.2m, trigger AUTO.LAND to land and disarm
                if self.current_z <= 0.25:
                    rospy.loginfo("[Mission Manager] Low altitude reached. Initiating AUTO.LAND...")
                    land_set_mode = SetModeRequest()
                    land_set_mode.custom_mode = 'AUTO.LAND'
                    try:
                        self.set_mode_client.call(land_set_mode)
                        rospy.loginfo("[Mission Manager] AUTO.LAND command sent successfully. Exiting.")
                        break
                    except rospy.ServiceException as e:
                        rospy.logerr(f"[Mission Manager] Service call failed: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        manager = NidarMissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
