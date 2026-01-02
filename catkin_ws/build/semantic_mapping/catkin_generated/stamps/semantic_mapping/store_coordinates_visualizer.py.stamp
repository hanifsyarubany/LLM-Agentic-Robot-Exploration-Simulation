#!/usr/bin/env python3
import rospy
import json
import os
import rospkg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class StoreCoordinateVisualizer:
    def __init__(self):
        rospy.init_node("store_coordinate_visualizer")

        # Locate semantic_mapping/maps folder
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("semantic_mapping")
        self.json_path = os.path.join(pkg_path, "maps", "store_coordinates.json")

        # Publisher
        self.pub = rospy.Publisher("/store_coordinates_markers", MarkerArray, queue_size=10)

        # Timer for 1 Hz refresh
        rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        rospy.loginfo("[store_visualizer] Running, reading JSON at 1 Hz...")
        rospy.spin()

    def timer_callback(self, event):
        if not os.path.exists(self.json_path):
            rospy.logwarn("JSON file not found: %s", self.json_path)
            return

        # Load JSON file
        try:
            with open(self.json_path, "r") as f:
                data = json.load(f)
            points = data.get("points", [])
        except Exception as e:
            rospy.logerr("Failed to read JSON: %s", e)
            return

        # Build marker array
        m_array = MarkerArray()
        now = rospy.Time.now()

        # Marker for points (green circles)
        points_marker = Marker()
        points_marker.header.stamp = now
        points_marker.header.frame_id = "world"

        points_marker.ns = "store_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD

        points_marker.scale.x = 0.2  # circle diameter
        points_marker.scale.y = 0.2

        # Green color
        points_marker.color.r = 0.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0
        points_marker.color.a = 1.0

        # Add all points
        for p in points:
            pt = Point()
            pt.x = float(p["x"])
            pt.y = float(p["y"])
            pt.z = 0.0
            points_marker.points.append(pt)

        m_array.markers.append(points_marker)

        # Add text labels for clarity
        for i, p in enumerate(points):
            txt = Marker()
            txt.header.stamp = now
            txt.header.frame_id = "world"

            txt.ns = "labels"
            txt.id = 1000 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD

            txt.scale.z = 0.3  # text height
            txt.color.r = 0.0
            txt.color.g = 1.0
            txt.color.b = 0.0
            txt.color.a = 1.0

            txt.pose.position.x = float(p["x"])
            txt.pose.position.y = float(p["y"])
            txt.pose.position.z = 0.3

            txt.text = p.get("label", "unknown")

            m_array.markers.append(txt)

        # Publish
        self.pub.publish(m_array)

if __name__ == "__main__":
    try:
        StoreCoordinateVisualizer()
    except rospy.ROSInterruptException:
        pass
