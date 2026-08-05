#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class SurvivorDetectorNode:
    def __init__(self):
        rospy.init_node('survivor_detector_node', anonymous=True)

        self.pub_markers = rospy.Publisher('/survivor_markers', MarkerArray, queue_size=10)

        # Simulated survivor locations inside competition arena rooms
        self.survivors = [
            {"id": 1, "x": -1.5, "y": -6.0, "name": "Survivor_Room1"},
            {"id": 2, "x": -1.5, "y": 1.0,  "name": "Survivor_Room2"},
            {"id": 3, "x": 3.5,  "y": 1.0,  "name": "Survivor_Room3"},
            {"id": 4, "x": 3.5,  "y": 6.0,  "name": "Survivor_GoalRoom"},
            {"id": 5, "x": -3.1, "y": -2.5, "name": "Survivor_TrapRoom5"},
            {"id": 6, "x": 6.3,  "y": 0.0,  "name": "Survivor_PillarGauntlet"}
        ]

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_markers)
        rospy.loginfo("[Survivor Detector] Initialized: 6 competition target locations configured.")

    def publish_markers(self, event):
        marker_array = MarkerArray()

        for s in self.survivors:
            # Sphere Marker
            marker = Marker()
            marker.header.frame_id = "camera_init"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "survivors"
            marker.id = s["id"]
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = s["x"]
            marker.pose.position.y = s["y"]
            marker.pose.position.z = 0.5  # Ground height indicator

            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4

            # Green color indicator for target survivors
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 0.9

            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)

            # Text Label Marker
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "survivor_labels"
            text_marker.id = s["id"] + 100
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = s["x"]
            text_marker.pose.position.y = s["y"]
            text_marker.pose.position.z = 0.9

            text_marker.scale.z = 0.35  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"ID#{s['id']}: {s['name']}"
            marker_array.markers.append(text_marker)

        self.pub_markers.publish(marker_array)

if __name__ == '__main__':
    try:
        node = SurvivorDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
