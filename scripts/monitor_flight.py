#!/usr/bin/env python3
import rospy
import csv
import sys
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

class FlightMonitor:
    def __init__(self):
        rospy.init_node('flight_monitor', anonymous=True)
        
        self.gazebo_pos = [0.0, 0.0, 0.0]
        self.fast_lio_pos = [0.0, 0.0, 0.0]
        self.mavros_pos = [0.0, 0.0, 0.0]
        self.vision_pos = [0.0, 0.0, 0.0]
        self.armed = False
        self.mode = "UNKNOWN"
        
        self.sub_gazebo = rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_cb)
        self.sub_fast_lio = rospy.Subscriber('/Fast_LIO/odometry', Odometry, self.fast_lio_cb)
        self.sub_mavros = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_cb)
        self.sub_vision = rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, self.vision_cb)
        self.sub_state = rospy.Subscriber('/mavros/state', State, self.state_cb)
        
        self.csv_file = open('/tmp/flight_telemetry.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'armed', 'mode', 
            'gt_x', 'gt_y', 'gt_z', 
            'lio_x', 'lio_y', 'lio_z', 
            'mavros_x', 'mavros_y', 'mavros_z', 
            'vision_x', 'vision_y', 'vision_z'
        ])
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.log_telemetry)
        rospy.loginfo("Flight Monitor Initialized. Logging to /tmp/flight_telemetry.csv")

    def gazebo_cb(self, msg):
        try:
            idx = msg.name.index('iris_vlp16')
            pos = msg.pose[idx].position
            self.gazebo_pos = [pos.x, pos.y, pos.z]
        except ValueError:
            pass

    def fast_lio_cb(self, msg):
        pos = msg.pose.pose.position
        self.fast_lio_pos = [pos.x, pos.y, pos.z]

    def mavros_cb(self, msg):
        pos = msg.pose.position
        self.mavros_pos = [pos.x, pos.y, pos.z]

    def vision_cb(self, msg):
        pos = msg.pose.position
        self.vision_pos = [pos.x, pos.y, pos.z]

    def state_cb(self, msg):
        self.armed = msg.armed
        self.mode = msg.mode

    def log_telemetry(self, event):
        t = rospy.get_time()
        self.csv_writer.writerow([
            t, self.armed, self.mode,
            self.gazebo_pos[0], self.gazebo_pos[1], self.gazebo_pos[2],
            self.fast_lio_pos[0], self.fast_lio_pos[1], self.fast_lio_pos[2],
            self.mavros_pos[0], self.mavros_pos[1], self.mavros_pos[2],
            self.vision_pos[0], self.vision_pos[1], self.vision_pos[2]
        ])
        self.csv_file.flush()
        
        # Also print to stdout for easy reading
        print(f"[Mon t={t:.1f}] Armed={self.armed} Mode={self.mode} | "
              f"GT: ({self.gazebo_pos[0]:.2f}, {self.gazebo_pos[1]:.2f}, {self.gazebo_pos[2]:.2f}) | "
              f"LIO: ({self.fast_lio_pos[0]:.2f}, {self.fast_lio_pos[1]:.2f}, {self.fast_lio_pos[2]:.2f}) | "
              f"PX4: ({self.mavros_pos[0]:.2f}, {self.mavros_pos[1]:.2f}, {self.mavros_pos[2]:.2f})", 
              flush=True)

    def close(self):
        self.csv_file.close()

if __name__ == '__main__':
    monitor = FlightMonitor()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        monitor.close()
