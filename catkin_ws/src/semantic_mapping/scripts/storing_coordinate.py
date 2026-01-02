#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import math
from collections import Counter

import rospy
import rospkg

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Int32, String


class CoordinateStorer(object):
    def __init__(self):
        # --------------------------------------------------------------
        # Config
        # --------------------------------------------------------------
        self.poi_labels = {"cafe", "convenience_store", "hamburger_store", "pharmacy"}

        # Topics
        self.pose_topic = rospy.get_param("~pose_topic", "/orb_slam3/pose_corrected")
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo_rs/xy")
        self.trigger_topic = rospy.get_param("~trigger_topic", "/storing_coordinate")

        # Sampling settings
        self.sample_duration = float(rospy.get_param("~sample_duration", 1.0))
        self.max_radius = float(rospy.get_param("~max_radius", 0.6))

        # Last robot pose (x, y)
        self.last_pose_xy = None  # (x, y)

        # Anchor points loaded from store_coordinates.json
        # each entry: {"x": float, "y": float, "store": str, ...}
        self.points = []
        self.json_data = {}

        # Sampling state
        self.sampling_active = False
        self.sampling_start_time = None
        self.sampled_labels = []
        self.current_anchor_index = None  # index into self.points

        # --------------------------------------------------------------
        # Init paths, load + reset store fields
        # --------------------------------------------------------------
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("semantic_mapping")

        self.anchor_file = os.path.join(pkg_path, "maps", "store_coordinates.json")

        self._load_points()
        self._reset_all_stores_to_unknown()   # <-- as you requested

        # --------------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------------
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=10
        )
        self.yolo_sub = rospy.Subscriber(
            self.yolo_topic, PointStamped, self.yolo_callback, queue_size=50
        )
        self.trigger_sub = rospy.Subscriber(
            self.trigger_topic, String, self.trigger_callback, queue_size=1
        )

        rospy.loginfo("[storing_coordinate] Node initialised.")
        rospy.loginfo("[storing_coordinate] Pose topic: %s", self.pose_topic)
        rospy.loginfo("[storing_coordinate] YOLO topic: %s", self.yolo_topic)
        rospy.loginfo("[storing_coordinate] Trigger topic: %s", self.trigger_topic)
        rospy.loginfo("[storing_coordinate] JSON file: %s", self.anchor_file)

    # ----------------------------------------------------------
    # File helpers
    # ----------------------------------------------------------
    def _load_points(self):
        """Load points from store_coordinates.json into self.json_data + self.points."""
        if not os.path.exists(self.anchor_file):
            rospy.logerr("[storing_coordinate] JSON file not found: %s", self.anchor_file)
            self.json_data = {"points": []}
            self.points = []
            return

        try:
            with open(self.anchor_file, "r") as f:
                self.json_data = json.load(f)

            pts = self.json_data.get("points", [])
            # Normalize minimal fields; preserve extra keys
            self.points = []
            for p in pts:
                if "x" not in p or "y" not in p:
                    continue
                q = dict(p)
                q["x"] = float(q["x"])
                q["y"] = float(q["y"])
                if "store" not in q:
                    q["store"] = "unknown"
                self.points.append(q)

            # Keep the normalized points in json_data too
            self.json_data["points"] = self.points

            rospy.loginfo("[storing_coordinate] Loaded %d points.", len(self.points))
        except Exception as e:
            rospy.logerr("[storing_coordinate] Failed to load JSON: %s", str(e))
            self.json_data = {"points": []}
            self.points = []

    def _atomic_write_json(self, path, data_obj):
        """Write JSON atomically to avoid partial/corrupt writes."""
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(data_obj, f, indent=2)
        os.replace(tmp_path, path)

    def _save_points(self):
        """Save current self.json_data (including updated self.points) into store_coordinates.json."""
        try:
            # Ensure directory exists
            d = os.path.dirname(self.anchor_file)
            if d and not os.path.exists(d):
                os.makedirs(d)

            self.json_data["points"] = self.points
            self._atomic_write_json(self.anchor_file, self.json_data)
        except Exception as e:
            rospy.logerr("[storing_coordinate] Failed to write JSON: %s", str(e))

    def _reset_all_stores_to_unknown(self):
        """
        On node startup: force every point["store"] = "unknown" and save.
        (This matches your requirement for the first launch.)
        """
        if not self.points:
            rospy.logwarn("[storing_coordinate] No points loaded; cannot reset stores.")
            return

        changed = False
        for p in self.points:
            if p.get("store", "unknown") != "unknown":
                p["store"] = "unknown"
                changed = True
            elif "store" not in p:
                p["store"] = "unknown"
                changed = True

        # Even if already unknown, still “make sure and edit” by writing once
        self._save_points()
        rospy.loginfo("[storing_coordinate] Reset all point stores to 'unknown' in %s", self.anchor_file)

    # ----------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------
    def pose_callback(self, msg):
        """Keep track of the robot's current x, y."""
        self.last_pose_xy = (msg.pose.position.x, msg.pose.position.y)

    def trigger_callback(self, msg):
        """
        Trigger when /storing_coordinate == 1.

        Steps:
          1) Use current robot pose (x,y)
          2) Find closest point in store_coordinates.json
          3) If distance > max_radius -> do nothing
          4) Else: sample /yolo_rs/xy for sample_duration seconds, majority-vote label
          5) Write that label into points[i]["store"] and save the same JSON file
        """
        entering_result = msg.data.strip().lower()
        rospy.loginfo("[storing_coordinate] successful_mark_entering_store: %s", entering_result)

        if self.last_pose_xy is None:
            rospy.logwarn("[storing_coordinate] Trigger received but no robot pose yet.")
            return

        if not self.points:
            rospy.logwarn("[storing_coordinate] Trigger received but no points loaded.")
            return

        rx, ry = self.last_pose_xy

        best_i = None
        best_dist = None
        for i, p in enumerate(self.points):
            dx = rx - p["x"]
            dy = ry - p["y"]
            dist = math.hypot(dx, dy)
            if best_i is None or dist < best_dist:
                best_i = i
                best_dist = dist

        if best_i is None:
            rospy.logwarn("[storing_coordinate] No anchor point found (unexpected).")
            return

        bp = self.points[best_i]
        rospy.loginfo("[storing_coordinate] Closest anchor idx=%d: (%.3f, %.3f), dist=%.3f",
                      best_i, bp["x"], bp["y"], best_dist)

        if best_dist > self.max_radius:
            rospy.loginfo("[storing_coordinate] Distance %.3f > %.3f m. Not storing.",
                          best_dist, self.max_radius)
            return

        # Start sampling
        self.current_anchor_index = best_i
        self.sampled_labels = []
        self.sampling_active = True
        self.sampling_start_time = rospy.Time.now().to_sec()

        rospy.loginfo("[storing_coordinate] Start sampling labels for %.2fs at idx=%d (%.3f, %.3f)",
                      self.sample_duration, best_i, bp["x"], bp["y"])

    def yolo_callback(self, msg):
        """
        While sampling:
          - collect POI labels from msg.header.frame_id
          - after sampling window: finalize once
        """
        if not self.sampling_active:
            return

        now_t = msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() \
            else rospy.Time.now().to_sec()

        elapsed = now_t - self.sampling_start_time

        if elapsed <= self.sample_duration:
            label = msg.header.frame_id
            if label in self.poi_labels:
                self.sampled_labels.append(label)

        if elapsed >= self.sample_duration:
            self._finalize_sampling()

    # ----------------------------------------------------------
    # Finalize + store into the SAME JSON file
    # ----------------------------------------------------------
    def _finalize_sampling(self):
        if not self.sampling_active:
            return

        # Stop further sampling immediately (prevents multiple finalizations)
        self.sampling_active = False

        if self.current_anchor_index is None:
            rospy.logwarn("[storing_coordinate] Sampling ended but no anchor index set.")
            return

        if not self.sampled_labels:
            rospy.loginfo("[storing_coordinate] No POI labels detected in the window. Not updating JSON.")
            self.current_anchor_index = None
            return

        counts = Counter(self.sampled_labels)
        chosen_label, count = max(counts.items(), key=lambda x: x[1])

        p = self.points[self.current_anchor_index]
        old_store = p.get("store", "unknown")
        p["store"] = chosen_label

        self._save_points()

        rospy.loginfo("[storing_coordinate] Updated idx=%d (%.3f, %.3f): store '%s' -> '%s' (%d votes)",
                      self.current_anchor_index, p["x"], p["y"], old_store, chosen_label, count)

        # Clear state
        self.current_anchor_index = None
        self.sampled_labels = []


def main():
    rospy.init_node("storing_coordinate_node")
    _ = CoordinateStorer()
    rospy.spin()


if __name__ == "__main__":
    main()
