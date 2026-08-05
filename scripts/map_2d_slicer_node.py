#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class Map2DSlicerNode:
    def __init__(self):
        rospy.init_node('map_2d_slicer_node', anonymous=True)

        self.resolution = rospy.get_param('~resolution', 0.1)  # 10cm grid
        self.width_m = rospy.get_param('~width_m', 30.0)      # 30m x 30m grid
        self.height_m = rospy.get_param('~height_m', 30.0)
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0
        self.z_min = rospy.get_param('~z_min', 0.3)
        self.z_max = rospy.get_param('~z_max', 1.8)

        self.grid_width = int(self.width_m / self.resolution)
        self.grid_height = int(self.height_m / self.resolution)

        self.sub_cloud = rospy.Subscriber(
            '/cloud_registered',
            PointCloud2,
            self.cloud_callback,
            queue_size=1
        )

        self.pub_map = rospy.Publisher(
            '/map_2d',
            OccupancyGrid,
            queue_size=1
        )

        rospy.loginfo(f"[2D Map Slicer] Initialized: Slicing Z in [{self.z_min}m, {self.z_max}m], Grid={self.grid_width}x{self.grid_height} @ {self.resolution}m res.")

    def cloud_callback(self, cloud_msg):
        grid_data = np.full((self.grid_height, self.grid_width), -1, dtype=np.int8)

        # Free space initializer inside boundary
        grid_data[:, :] = 0

        # Read 3D points from FAST-LIO registered cloud
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            if self.z_min <= p[2] <= self.z_max:
                gx = int((p[0] - self.origin_x) / self.resolution)
                gy = int((p[1] - self.origin_y) / self.resolution)

                if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                    grid_data[gy, gx] = 100  # Occupied cell

        # Create OccupancyGrid ROS Message
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "camera_init"

        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.origin.position.x = self.origin_x
        grid_msg.info.origin.position.y = self.origin_y
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0

        grid_msg.data = grid_data.flatten().tolist()
        self.pub_map.publish(grid_msg)

if __name__ == '__main__':
    try:
        node = Map2DSlicerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
