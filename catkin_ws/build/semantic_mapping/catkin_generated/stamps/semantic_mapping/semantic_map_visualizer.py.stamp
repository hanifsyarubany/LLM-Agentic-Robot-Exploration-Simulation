#!/usr/bin/env python3
import os
import json
import hashlib

import rospy
import rospkg

from visualization_msgs.msg import Marker, MarkerArray
import tf.transformations as tft


class SemanticMapVisualizer:
    def __init__(self):
        rospy.init_node("semantic_map_visualizer")

        # Parameters
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.check_rate = rospy.get_param("~check_rate", 1.0)  # Hz

        # Resolve default JSON path: <pkg>/maps/semantic_map.json
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("semantic_mapping")
        default_json_path = os.path.join(
            pkg_path, "maps", "semantic_map.json"
        )
        self.json_path = rospy.get_param("~semantic_map_path", default_json_path)

        rospy.loginfo("[SemanticMapVisualizer] Using JSON path: %s", self.json_path)

        self.marker_pub = rospy.Publisher(
            "semantic_map_markers", MarkerArray, queue_size=1, latch=True
        )

        self.last_mtime = None
        self.last_hash = None

    def run(self):
        rate = rospy.Rate(self.check_rate)
        while not rospy.is_shutdown():
            self.check_and_publish()
            rate.sleep()

    def check_and_publish(self):
        # Check if file exists
        if not os.path.exists(self.json_path):
            rospy.logwarn_throttle(
                10.0,
                "[SemanticMapVisualizer] JSON file not found: %s" % self.json_path,
            )
            return

        try:
            mtime = os.path.getmtime(self.json_path)
        except OSError as e:
            rospy.logwarn_throttle(
                10.0,
                "[SemanticMapVisualizer] Could not stat JSON file: %s" % str(e),
            )
            return

        # If file timestamp did not change, no need to reload
        if self.last_mtime is not None and mtime == self.last_mtime:
            return

        # Load file
        try:
            with open(self.json_path, "r") as f:
                data = json.load(f)
        except Exception as e:
            rospy.logwarn(
                "[SemanticMapVisualizer] Failed to load JSON: %s", str(e)
            )
            return

        # Hash the content to avoid re-publishing identical data
        try:
            data_str = json.dumps(data, sort_keys=True)
        except TypeError:
            data_str = str(data)
        new_hash = hashlib.md5(data_str.encode("utf-8")).hexdigest()

        if self.last_hash is not None and new_hash == self.last_hash:
            # Same content, even if mtime changed
            self.last_mtime = mtime
            return

        # If we reach here, content changed -> publish markers
        rospy.loginfo(
            "[SemanticMapVisualizer] JSON updated, publishing %d junction(s)...",
            len(data) if isinstance(data, list) else 0,
        )
        self.last_mtime = mtime
        self.last_hash = new_hash

        self.publish_markers(data)

    # ---------------- helper: formatting ----------------

    def _direction_letter(self, dir_str):
        """Convert 'left'/'right'/'straight' → 'L'/'R'/'S'."""
        d = (dir_str or "").strip().lower()
        if d == "left":
            return "L"
        if d == "right":
            return "R"
        if d == "straight":
            return "S"
        return "?"

    def _junction_label_text(self, junction_entry):
        """
        Build label like:
          'J2: R - pharmacy & S - convenience_store'
        from:
          {
            "name": "junction_2",
            "poi_pairs": ["right --- pharmacy", "straight --- convenience_store"],
            ...
          }
        """
        name = junction_entry.get("name", "junction_?")
        # Extract index from "junction_2" → "J2"
        if name.startswith("junction_"):
            suffix = name[len("junction_"):]
            jname = f"J{suffix}"
        else:
            jname = name

        pairs = junction_entry.get("poi_pairs", [])
        components = []

        for pair in pairs:
            # pair like "right --- pharmacy"
            if not isinstance(pair, str):
                continue
            parts = pair.split("---")
            if len(parts) != 2:
                continue
            dir_str = parts[0].strip()
            poi_str = parts[1].strip()
            letter = self._direction_letter(dir_str)
            components.append(f"{letter}-> {poi_str}")

        if not components:
            # Fallback if no valid pairs
            text_core = "(no pairs)"
        else:
            text_core = "\n".join(components)

        return f"{jname}\n{text_core}"

    def publish_markers(self, entries):
        if not isinstance(entries, list):
            rospy.logwarn(
                "[SemanticMapVisualizer] JSON root is not a list, skipping."
            )
            return

        now = rospy.Time.now()
        marker_array = MarkerArray()

        # First, send DELETEALL to clear old markers
        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame_id
        delete_marker.header.stamp = now
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # Then add new markers (one arrow + one text per junction)
        for i, entry in enumerate(entries):
            robot_pose = entry.get("robot_pose_2d", {})
            try:
                x = float(robot_pose["x"])
                y = float(robot_pose["y"])
                yaw = float(robot_pose["yaw"])
            except Exception:
                rospy.logwarn(
                    "[SemanticMapVisualizer] Invalid robot_pose_2d in junction %d, skipping.",
                    i,
                )
                continue

            # Convert yaw -> quaternion
            q = tft.quaternion_from_euler(0.0, 0.0, yaw)

            # You could choose color based on something; here: neutral light blue
            color = (0.2, 0.8, 1.0, 0.9)

            # --- Arrow marker (robot pose & yaw) ---
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = now
            arrow.ns = "semantic_junction_pose"
            arrow.id = 2 * i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD

            arrow.pose.position.x = x
            arrow.pose.position.y = y
            arrow.pose.position.z = 0.0
            arrow.pose.orientation.x = q[0]
            arrow.pose.orientation.y = q[1]
            arrow.pose.orientation.z = q[2]
            arrow.pose.orientation.w = q[3]

            # Arrow size
            arrow.scale.x = 0.7   # length
            arrow.scale.y = 0.12  # shaft diameter
            arrow.scale.z = 0.12  # head diameter

            arrow.color.r = color[0]
            arrow.color.g = color[1]
            arrow.color.b = color[2]
            arrow.color.a = color[3]

            arrow.lifetime = rospy.Duration(0.0)  # forever

            marker_array.markers.append(arrow)

            # --- Text marker (junction label) ---
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "semantic_junction_label"
            text.id = 2 * i + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD

            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.5  # a bit above the ground
            text.pose.orientation.x = 0.0
            text.pose.orientation.y = 0.0
            text.pose.orientation.z = 0.0
            text.pose.orientation.w = 1.0

            text.text = self._junction_label_text(entry)

            # Only scale.z is used for text size
            text.scale.z = 0.35

            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0

            text.lifetime = rospy.Duration(0.0)

            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    node = SemanticMapVisualizer()
    node.run()

