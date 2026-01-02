#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import threading
from datetime import datetime

import rospy
from std_msgs.msg import String

import rospkg


class HistoricalMappingNode:
    def __init__(self):
        # Topics (override via rosparam if desired)
        self.semantic_topic = rospy.get_param("~semantic_topic", "/semantic_mapping_entries")
        self.action_topic = rospy.get_param("~action_topic", "/next_controller_action")

        # Default history file path from semantic_mapping package, as requested
        default_history_path = self._default_history_file_from_pkg()

        # Allow override, but default to package/maps/historical_robot_trajectory.json
        self.history_file = rospy.get_param("~history_file", default_history_path)

        # Ensure parent dir exists
        parent = os.path.dirname(self.history_file)
        if parent and not os.path.exists(parent):
            os.makedirs(parent, exist_ok=True)

        self._lock = threading.Lock()

        # NEW: clear previous history when node starts
        self._clear_history_file_once()

        # NEW: order/target grasping file polling (1 Hz)
        default_order_path = self._default_order_list_file_from_pkg()
        self.order_list_file = rospy.get_param("~order_list_file", default_order_path)
        self._last_order_state = None  # baseline set on first successful read
        self._order_timer = rospy.Timer(rospy.Duration(1.0), self.order_timer_cb)

        # Subscribers
        self._semantic_sub = rospy.Subscriber(self.semantic_topic, String, self.semantic_cb, queue_size=50)
        self._action_sub = rospy.Subscriber(self.action_topic, String, self.action_cb, queue_size=50)

        rospy.loginfo("historical_mapping_node started.")
        rospy.loginfo("  semantic_topic: %s", self.semantic_topic)
        rospy.loginfo("  action_topic:   %s", self.action_topic)
        rospy.loginfo("  history_file:   %s", self.history_file)
        rospy.loginfo("  order_list_file:%s", self.order_list_file)

    def _clear_history_file_once(self) -> None:
        """
        Truncate the history file at startup so old history is removed.
        """
        try:
            # "w" truncates (or creates) the file
            with open(self.history_file, "w", encoding="utf-8") as f:
                f.write("")
            rospy.loginfo("Cleared history file on startup: %s", self.history_file)
        except Exception as e:
            rospy.logwarn("Failed to clear history file '%s': %s", self.history_file, str(e))

    def _default_history_file_from_pkg(self) -> str:
        """
        Build:
          rospack = rospkg.RosPack()
          pkg_path = rospack.get_path("semantic_mapping")
          semantic_map_path = os.path.join(pkg_path, "maps", "historical_robot_trajectory.json")
        """
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path("semantic_mapping")
            return os.path.join(pkg_path, "maps", "historical_robot_trajectory.json")
        except rospkg.ResourceNotFound:
            # Fallback so node still runs
            rospy.logwarn("Package 'semantic_mapping' not found via rospkg. Falling back to /tmp.")
            return "/tmp/historical_robot_trajectory.json"
        except Exception as e:
            rospy.logwarn("Failed to resolve history file path from rospkg (%s). Falling back to /tmp.", str(e))
            return "/tmp/historical_robot_trajectory.json"

    def _default_order_list_file_from_pkg(self) -> str:
        """
        Build:
          rospack = rospkg.RosPack()
          pkg_path = rospack.get_path("semantic_mapping")
          order_list_path = os.path.join(pkg_path, "maps", "target_object_grasping.json")
        """
        try:
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path("semantic_mapping")
            return os.path.join(pkg_path, "maps", "target_object_grasping.json")
        except rospkg.ResourceNotFound:
            rospy.logwarn("Package 'semantic_mapping' not found via rospkg. Falling back to /tmp.")
            return "/tmp/target_object_grasping.json"
        except Exception as e:
            rospy.logwarn("Failed to resolve order list path from rospkg (%s). Falling back to /tmp.", str(e))
            return "/tmp/target_object_grasping.json"

    def _load_order_list(self) -> dict:
        """Read target_object_grasping.json and return a {str: int} dict."""
        try:
            if not os.path.exists(self.order_list_file):
                return None
            with open(self.order_list_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return None
            out = {}
            for k, v in data.items():
                key = str(k).strip()
                if not key:
                    continue
                try:
                    out[key] = int(v)
                except Exception:
                    continue
            return out
        except Exception as e:
            rospy.logwarn("Failed reading order list file '%s': %s", self.order_list_file, str(e))
            return None

    def order_timer_cb(self, event) -> None:
        """Poll order_list_file at 1 Hz and log decreases as successful retrievals."""
        current = self._load_order_list()
        if current is None:
            return

        # Update baseline snapshot safely
        with self._lock:
            prev = self._last_order_state
            if prev is None:
                self._last_order_state = current
                return
            prev_copy = dict(prev)
            self._last_order_state = current

        # Preserve a reasonable order: previous keys first, then any new keys
        ordered_keys = list(prev_copy.keys())
        for k in current.keys():
            if k not in prev_copy:
                ordered_keys.append(k)

        retrieved = []
        for k in ordered_keys:
            p = int(prev_copy.get(k, 0))
            n = int(current.get(k, 0))
            d = p - n
            if d > 0:
                retrieved.append((k, d))

        if retrieved:
            ts = self._ts()
            parts = [f"{cnt} {name}" for (name, cnt) in retrieved]
            line = f"[{ts}] the robot successfully retrieved {', '.join(parts)}."
            self._append_line(line)

    def _ts(self, unix_ts=None) -> str:
        """Return ISO8601 timestamp (local time) for file lines."""
        try:
            if unix_ts is None:
                unix_ts = rospy.Time.now().to_sec()
            return datetime.fromtimestamp(float(unix_ts)).isoformat(timespec="seconds")
        except Exception:
            return datetime.now().isoformat(timespec="seconds")

    def _append_line(self, line: str) -> None:
        with self._lock:
            with open(self.history_file, "a", encoding="utf-8") as f:
                f.write(line.rstrip() + "\n")

    @staticmethod
    def _humanize_name(name: str) -> str:
        # "junction_1" -> "junction 1"
        return (name or "unknown").replace("_", " ").strip()

    def semantic_cb(self, msg: String) -> None:
        raw = msg.data
        try:
            data = json.loads(raw)
            name = self._humanize_name(str(data.get("name", "unknown")))

            created_at_unix = data.get("created_at_unix", None)
            ts = self._ts(created_at_unix)

            poi_pairs = data.get("poi_pairs", [])
            if isinstance(poi_pairs, str):
                poi_pairs = [poi_pairs]
            poi_pairs = [str(x).strip() for x in poi_pairs if str(x).strip()]

            pose = data.get("robot_pose_2d", {})
            pose_str = ""
            try:
                x = pose.get("x", None)
                y = pose.get("y", None)
                yaw = pose.get("yaw", None)
                if x is not None and y is not None and yaw is not None:
                    pose_str = f" (pose: x={float(x):.3f}, y={float(y):.3f}, yaw={float(yaw):.3f})"
            except Exception:
                pose_str = ""

            if poi_pairs:
                obs = "; ".join(poi_pairs)
                line = f"[{ts}] the robot has stopped at {name}{pose_str}, and it observes: {obs}."
            else:
                line = f"[{ts}] the robot has stopped at {name}{pose_str}."

            self._append_line(line)

        except Exception as e:
            ts = self._ts()
            self._append_line(f"[{ts}] (semantic_mapping_entries parse error) raw: {raw}")
            rospy.logwarn("semantic_cb parse error: %s", str(e))

    def action_cb(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        ts = self._ts()

        cleaned = raw.replace(" --- ", "---").replace("--- ", "---").replace(" ---", "---")

        if "---" not in cleaned:
            self._append_line(f"[{ts}] (next_controller_action malformed) raw: {raw}")
            return

        direction, decision = cleaned.split("---", 1)
        direction = direction.strip().lower()
        decision = decision.strip().replace(" ", "_")

        decision_text = "just continue" if decision.lower() == "continue" else f"enter the {decision}"

        if direction == "left":
            line = f"[{ts}] the robot is turning to the left, and then {decision_text}."
        elif direction == "right":
            line = f"[{ts}] the robot is turning to the right, and then {decision_text}."
        elif direction in ("straight", "forward"):
            line = f"[{ts}] the robot is going straight forward, and then {decision_text}."
        else:
            line = f"[{ts}] the robot will move '{direction}', and then {decision_text}."

        self._append_line(line)


def main():
    rospy.init_node("historical_mapping_node", anonymous=False)
    HistoricalMappingNode()
    rospy.spin()


if __name__ == "__main__":
    main()

