#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
from collections import defaultdict

import rospy
import rospkg
import math

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion


class SemanticMapper(object):
    def __init__(self):
        # ------------------------------------------------------------------
        # Configuration
        # ------------------------------------------------------------------
        self.direction_labels = {"left", "right", "straight"}
        self.poi_labels = {"pickup", "cafe", "convenience_store",
                           "hamburger_store", "pharmacy"}

        # Topics (overridable via rosparam / launch args)
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo_rs/xy")
        self.pose_topic = rospy.get_param("~pose_topic",
                                          "/orb_slam3/pose_corrected")
        self.detecting_sign_topic = rospy.get_param(
            "~detecting_sign_topic", "/detecting_sign"
        )

        # Pose-duplicate thresholds (for "very very close" junction merging)
        self.pose_dup_dist_thresh = float(
            rospy.get_param("~pose_dup_dist_thresh", 0.3)
        )  # meters
        yaw_deg = float(
            rospy.get_param("~pose_dup_yaw_thresh_deg", 20.0)
        )  # degrees
        self.pose_dup_yaw_thresh = math.radians(yaw_deg)  # radians

        # Sampling window (seconds)
        self.initial_duration = float(rospy.get_param("~initial_duration", 1.0))
        self.max_duration = float(rospy.get_param("~max_duration", 5.0))

        # Robot pose (flattened to z=0, yaw)
        self.last_pose_2d = None  # {"x","y","z","yaw"}

        self.db = []
        self.map_file = self._init_db_file()
        self._reset_db_on_start()

        # Session state (per /detecting_sign trigger)
        self.session_active = False
        self.session_finished = False
        self.session_start_time = None
        self.session_last_time = None

        # All YOLO detections for the current session:
        #   key = header.stamp.secs
        #   value = list of detections in that second
        self.detections_by_secs = defaultdict(list)

        # Subscribers
        self.yolo_sub = rospy.Subscriber(
            self.yolo_topic, PointStamped, self.yolo_callback, queue_size=50
        )
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=10
        )
        self.detect_sub = rospy.Subscriber(
            self.detecting_sign_topic, Int32,
            self.detecting_sign_callback, queue_size=1
        )

        # Latched publisher for success/fail mark
        self.mark_pub = rospy.Publisher(
            "/semantic_mapping_mark", String, queue_size=10, latch=True
        )
        # Publisher for new/updated junctions (JSON array, usually length 1)
        self.entries_pub = rospy.Publisher(
            "/semantic_mapping_entries", String, queue_size=1, latch=True
        )

        rospy.loginfo("[semantic_mapping] Node initialised.")
        rospy.loginfo("[semantic_mapping] YOLO topic: %s", self.yolo_topic)
        rospy.loginfo("[semantic_mapping] Pose topic: %s", self.pose_topic)
        rospy.loginfo("[semantic_mapping] Detecting_sign topic: %s",
                      self.detecting_sign_topic)
        rospy.loginfo("[semantic_mapping] DB file: %s", self.map_file)
        rospy.loginfo("[semantic_mapping] pose_dup_dist_thresh=%.3f m, "
                      "pose_dup_yaw_thresh=%.1f deg",
                      self.pose_dup_dist_thresh, yaw_deg)

    # ----------------------------------------------------------------------
    # DB helpers
    # ----------------------------------------------------------------------
    def _init_db_file(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("semantic_mapping")
        maps_dir = os.path.join(pkg_path, "maps")
        if not os.path.exists(maps_dir):
            os.makedirs(maps_dir)
        return os.path.join(maps_dir, "semantic_map.json")

    def _reset_db_on_start(self):
        """Clear DB every time node starts."""
        self.db = []
        try:
            with open(self.map_file, "w") as f:
                json.dump(self.db, f, indent=2)
            rospy.loginfo("[semantic_mapping] DB reset at startup: %s",
                          self.map_file)
        except Exception as e:
            rospy.logerr("[semantic_mapping] Failed to reset DB '%s': %s",
                         self.map_file, str(e))

    def _save_db(self):
        try:
            with open(self.map_file, "w") as f:
                json.dump(self.db, f, indent=2)
        except Exception as e:
            rospy.logerr("[semantic_mapping] Failed to write DB: %s", str(e))

    # ----------------------------------------------------------------------
    # Callbacks
    # ----------------------------------------------------------------------
    def pose_callback(self, msg):
        """Store last robot pose, flattened to (x, y, z=0, yaw)."""
        p = msg.pose.position
        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        self.last_pose_2d = {
            "x": p.x,
            "y": p.y,
            "z": 0.0,
            "yaw": yaw,
        }

    def detecting_sign_callback(self, msg):
        """
        Trigger semantic mapping process.

        - When msg.data == 1:
            Start a new semantic mapping session (sample for 1–5 seconds)
            IF no session is currently active.

        - When msg.data != 1:
            Cancel the current session (no mark, no save).
        """
        val = int(msg.data)

        if val == 1:
            if not self.session_active:
                rospy.loginfo("[semantic_mapping] /detecting_sign=1 -> start new session")
                self._start_new_session()
            else:
                rospy.loginfo("[semantic_mapping] /detecting_sign=1 but session already active, ignoring")
        else:
            if self.session_active:
                rospy.loginfo("[semantic_mapping] /detecting_sign=%d -> cancel current session", val)
                self._cancel_session()

    def _start_new_session(self):
        self.session_active = True
        self.session_finished = False
        self.session_start_time = None
        self.session_last_time = None
        self.detections_by_secs = defaultdict(list)

    def _cancel_session(self):
        self.session_active = False
        self.session_finished = False
        self.session_start_time = None
        self.session_last_time = None
        self.detections_by_secs = defaultdict(list)

    def _end_session(self):
        self.session_active = False
        self.session_finished = True
        self.session_start_time = None
        self.session_last_time = None
        self.detections_by_secs = defaultdict(list)

    def yolo_callback(self, msg):
        """
        Collect YOLO detections only while a mapping session is active.
        """
        if not self.session_active or self.session_finished:
            return

        stamp = msg.header.stamp
        t = stamp.to_sec()
        secs_key = stamp.secs

        # Initialize session time window
        if self.session_start_time is None:
            self.session_start_time = t
        self.session_last_time = t

        det = {
            "label": msg.header.frame_id,
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
            "seq": msg.header.seq,
            "secs": stamp.secs,
            "nsecs": stamp.nsecs,
        }

        self.detections_by_secs[secs_key].append(det)

        # Check if we should run the mapping logic now
        self._maybe_run_mapping()

    # ----------------------------------------------------------------------
    # Core mapping logic
    # ----------------------------------------------------------------------
    def _maybe_run_mapping(self):
        """
        Run semantic mapping logic after enough sequences have been sampled.
        """
        if not self.session_active or self.session_finished:
            return
        if not self.detections_by_secs:
            return
        if self.session_start_time is None or self.session_last_time is None:
            return

        duration = self.session_last_time - self.session_start_time

        # Need at least 1 second of data before making any decision
        if duration < self.initial_duration:
            return

        # Compute mapping from the sampled data (distance-based)
        mapping_entries, direction_seen = self._compute_mapping()
        pose_ready = self.last_pose_2d is not None

        assigned_dirs = {entry["D"] for entry in mapping_entries}
        full_success = (len(direction_seen) > 0 and
                        assigned_dirs == direction_seen)

        timed_out = (duration >= self.max_duration)

        # Case A: full success and pose ready -> store + SUCCESS and stop immediately
        if full_success and mapping_entries and pose_ready:
            self._store_mapping(mapping_entries)
            self._publish_mark("success")
            rospy.loginfo(
                "[semantic_mapping] SUCCESS (all directions mapped), duration=%.2fs",
                duration,
            )
            self._end_session()
            return

        # Case B: timeout
        if timed_out:
            if mapping_entries and pose_ready:
                self._store_mapping(mapping_entries)
                rospy.loginfo("[semantic_mapping] TIMEOUT: stored partial mapping.")
            else:
                rospy.loginfo("[semantic_mapping] TIMEOUT: no valid pairs to store.")

            if full_success and pose_ready:
                self._publish_mark("success")
            else:
                self._publish_mark("failed")

            self._end_session()
            return
        # Else: keep sampling

    def _compute_mapping(self):
        """
        Build direction–POI pairs purely based on XY distance.
        """
        direction_seen = set()
        candidate_pairs = []

        for secs, dets in self.detections_by_secs.items():
            directions = [d for d in dets if d["label"] in self.direction_labels]
            pois = [d for d in dets if d["label"] in self.poi_labels]

            for d in directions:
                direction_seen.add(d["label"])

            if not directions or not pois:
                continue

            for d in directions:
                best_p = None
                best_d2 = None
                for p in pois:
                    dx = p["x"] - d["x"]
                    dy = p["y"] - d["y"]
                    dist2 = dx * dx + dy * dy
                    if best_p is None or dist2 < best_d2:
                        best_p = p
                        best_d2 = dist2

                if best_p is not None:
                    candidate_pairs.append({
                        "D": d["label"],
                        "P": best_p["label"],
                        "direction_det": d,
                        "poi_det": best_p,
                        "secs": secs,
                        "dist2": best_d2,
                    })

        if not candidate_pairs:
            return [], direction_seen

        candidate_pairs.sort(key=lambda c: c["dist2"])

        mapping_entries = []
        used_dirs = set()
        used_pois = set()

        for c in candidate_pairs:
            D = c["D"]
            P = c["P"]
            if D in used_dirs:
                continue
            if P in used_pois:
                continue

            used_dirs.add(D)
            used_pois.add(P)
            mapping_entries.append(c)

        return mapping_entries, direction_seen

    # ----------------------------------------------------------------------
    # Pose-based junction search
    # ----------------------------------------------------------------------
    def _find_close_junction(self, new_pose):
        """
        Return index of an existing junction whose robot_pose_2d is
        very close to new_pose (within thresholds), or None if none found.
        """
        new_x = new_pose["x"]
        new_y = new_pose["y"]
        new_yaw = new_pose["yaw"]

        for idx, j in enumerate(self.db):
            pose = j.get("robot_pose_2d", None)
            if pose is None:
                continue

            px = pose.get("x", None)
            py = pose.get("y", None)
            pyaw = pose.get("yaw", None)
            if px is None or py is None or pyaw is None:
                continue

            dx = px - new_x
            dy = py - new_y
            dist = math.sqrt(dx * dx + dy * dy)

            dyaw = pyaw - new_yaw
            dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
            dyaw_abs = abs(dyaw)

            if dist <= self.pose_dup_dist_thresh and dyaw_abs <= self.pose_dup_yaw_thresh:
                rospy.loginfo(
                    "[semantic_mapping] Found existing junction '%s' (dist=%.3f, dyaw=%.2f deg)",
                    j.get("name", "unknown"),
                    dist,
                    math.degrees(dyaw_abs),
                )
                return idx

        return None

    # ----------------------------------------------------------------------
    # Storing & mark (junction-based)
    # ----------------------------------------------------------------------
    def _store_mapping(self, mapping_entries):
        """
        Take mapping_entries from a session and:
          - Build set of "D --- P" strings (poi_pairs) for this junction.
          - Merge into an existing junction if pose is close.
          - Otherwise create a new junction_X.
        """
        if self.last_pose_2d is None:
            rospy.logwarn("[semantic_mapping] No robot pose, cannot store pairs.")
            return
        if not mapping_entries:
            return

        # 1) Build unique poi_pairs for this session
        new_pairs = set()
        for m in mapping_entries:
            D = m["D"]
            P = m["P"]
            pair_str = f"{D} --- {P}"
            new_pairs.add(pair_str)

        if not new_pairs:
            rospy.loginfo("[semantic_mapping] No valid pairs to add.")
            return

        new_pose = self.last_pose_2d.copy()
        now_ts = time.time()

        # 2) Check if we already have a nearby junction (pose-based)
        idx = self._find_close_junction(new_pose)

        if idx is not None:
            # Update existing junction
            junction = self.db[idx]
            old_pairs = set(junction.get("poi_pairs", []))
            before_count = len(old_pairs)
            merged_pairs = sorted(old_pairs.union(new_pairs))

            junction["poi_pairs"] = merged_pairs
            # (Optionally update created_at_unix or add updated_at_unix)
            # For now, keep original created_at_unix as "first seen".

            added = len(merged_pairs) - before_count
            rospy.loginfo(
                "[semantic_mapping] Updated junction '%s': +%d pair(s), total=%d.",
                junction.get("name", "unknown"),
                added,
                len(merged_pairs),
            )

            self._save_db()

            # Publish this (updated) junction as JSON array with 1 element
            self._publish_entries(junction)

        else:
            # 3) Create a new junction
            name = "junction_%d" % (len(self.db) + 1)
            junction = {
                "name": name,
                "poi_pairs": sorted(new_pairs),
                "created_at_unix": now_ts,
                "robot_pose_2d": {
                    "x": new_pose["x"],
                    "y": new_pose["y"],
                    "z": new_pose["z"],
                    "yaw": new_pose["yaw"],
                },
            }
            self.db.append(junction)
            self._save_db()

            rospy.loginfo(
                "[semantic_mapping] Created new junction '%s' with %d pair(s).",
                name,
                len(junction["poi_pairs"]),
            )

            # Publish this new junction
            self._publish_entries(junction)

    def _publish_entries(self, entries_list):
        """
        Publish list of junction dicts on /semantic_mapping_entries as JSON.
        Typically entries_list has length 1 (the new/updated junction).
        """
        try:
            msg = String()
            msg.data = json.dumps(entries_list)
            self.entries_pub.publish(msg)
            rospy.loginfo(
                "[semantic_mapping] Published %d junction entry(ies) on /semantic_mapping_entries.",
                len(entries_list),
            )
        except Exception as e:
            rospy.logerr(
                "[semantic_mapping] Failed to JSON-encode junction(s) for publishing: %s",
                str(e),
            )

    def _publish_mark(self, status):
        """
        Publish "success" or "failed" on /semantic_mapping_mark (latched).
        """
        msg = String()
        msg.data = status
        self.mark_pub.publish(msg)
        rospy.loginfo("[semantic_mapping] semantic_mapping_mark: '%s'", status)


def main():
    rospy.init_node("semantic_mapper_node")
    mapper = SemanticMapper()
    rospy.spin()


if __name__ == "__main__":
    main()
