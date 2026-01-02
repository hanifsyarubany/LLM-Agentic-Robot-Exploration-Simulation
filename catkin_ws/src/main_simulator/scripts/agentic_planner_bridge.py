#!/usr/bin/env python3
import os
import math
import yaml

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as tft


class AgenticPlannerBridge:
    """
    Jembatan antara:
      - perintah high-level (String) / goal langsung (PoseStamped)
      - dan move_base (melalui /move_base_simple/goal)

    Sekaligus:
      - melacak state (IDLE / NAVIGATING / RECOVERY)
      - deteksi stuck sederhana dan publish event recovery
    """

    def __init__(self):
        rospy.init_node("agentic_planner_bridge")

        # -------------------
        # Load store YAML
        # -------------------
        self.stores = {}
        yaml_path = rospy.get_param("~store_yaml", "")
        if yaml_path and os.path.exists(yaml_path):
            try:
                with open(yaml_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                self.stores = data.get("stores", {})
                rospy.loginfo(f"[agentic_planner] Loaded stores from {yaml_path}: {list(self.stores.keys())}")
            except Exception as e:
                rospy.logwarn(f"[agentic_planner] Failed to load YAML: {e}")
        else:
            rospy.loginfo("[agentic_planner] No store YAML provided or file missing")

        self.frame_id = rospy.get_param("~frame_id", "odom")

        # -------------------
        # Pub/Sub
        # -------------------
        # ke move_base
        self.pub_goal = rospy.Publisher("/move_base_simple/goal",
                                        PoseStamped,
                                        queue_size=1)

        # state + event buat node lain
        self.pub_state = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.pub_recovery = rospy.Publisher("~recovery_event", String, queue_size=1)

        # perintah high-level
        rospy.Subscriber("/agent_command", String, self.cmd_cb, queue_size=10)

        # goal langsung (PoseStamped) dari node lain
        rospy.Subscriber("~direct_goal", PoseStamped, self.direct_goal_cb, queue_size=1)

        # odometry buat stuck detection
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=10)

        # -------------------
        # Stuck detection params
        # -------------------
        self.stuck_timeout = rospy.get_param("~stuck_timeout", 5.0)        # detik
        self.stuck_dist_eps = rospy.get_param("~stuck_dist_eps", 0.02)     # meter
        self.navigation_active = False
        self.last_moving_time = None
        self.last_odom_xy = None

        # simpan pose terakhir apapun
        self.current_odom_xy = None

        self.set_state("IDLE")
        rospy.loginfo("[agentic_planner] Ready.")

    # -------------------
    # Helper state
    # -------------------
    def set_state(self, s):
        self.state = s
        self.pub_state.publish(String(data=s))
        rospy.loginfo(f"[agentic_planner] State -> {s}")

    # -------------------
    # Goal sending
    # -------------------
    def send_goal_pose(self, pose_msg):
        """
        pose_msg: PoseStamped dengan frame_id sudah sesuai (odom/map)
        """
        pose_msg.header.stamp = rospy.Time.now()
        self.pub_goal.publish(pose_msg)

        self.navigation_active = True
        self.last_moving_time = rospy.Time.now()
        # reset anchor untuk stuck detection
        if self.current_odom_xy is not None:
            self.last_odom_xy = self.current_odom_xy

        self.set_state("NAVIGATING")

    def send_goal_xyz_yaw(self, x, y, yaw_deg):
        yaw = math.radians(yaw_deg)
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.send_goal_pose(msg)

    # -------------------
    # Callback function
    # -------------------
    def cmd_cb(self, msg):
        """
        /agent_command: String
        contoh: "GOTO_STORE1", "GOTO_BURGER_STORE", "STOP"
        """
        cmd = msg.data.strip()
        upper = cmd.upper()
        rospy.loginfo(f"[agentic_planner] /agent_command: {cmd}")

        if upper.startswith("GOTO_"):
            # ex: GOTO_STORE1 -> "STORE1" -> "store1"
            name = upper.replace("GOTO_", "")
            key = name.lower()

            if key in self.stores:
                g = self.stores[key]
                x = float(g["x"])
                y = float(g["y"])
                yaw_deg = float(g.get("yaw_deg", 0.0))
                rospy.loginfo(f"[agentic_planner] Resolving {cmd} -> ({x:.2f}, {y:.2f}, {yaw_deg:.1f}deg)")
                self.send_goal_xyz_yaw(x, y, yaw_deg)
            else:
                rospy.logwarn(f"[agentic_planner] Unknown store name '{key}'. Known: {list(self.stores.keys())}")

        elif upper == "STOP":
            # versi minimal: cuma broadcast event STOP dan ubah state
            self.navigation_active = False
            self.set_state("IDLE")
            self.pub_recovery.publish(String(data="STOP"))
            rospy.loginfo("[agentic_planner] STOP requested (higher-level node should cancel move_base goal).")

        else:
            rospy.logwarn(f"[agentic_planner] Unknown command: {cmd}")

    def direct_goal_cb(self, msg):
        """
        direct goal dari node lain (mis. AprilTag node).
        Misalnya frame_id sudah 'odom' atau 'map'.
        """
        rospy.loginfo(f"[agentic_planner] Received direct goal in frame '{msg.header.frame_id}'")
        self.send_goal_pose(msg)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_odom_xy = (x, y)

        if not self.navigation_active:
            # kalo lagi IDLE, bisa tetap update anchor
            self.last_odom_xy = (x, y)
            self.last_moving_time = rospy.Time.now()
            return

        # kalo sedang NAVIGATING, cek seberapa jauh geraknya
        if self.last_odom_xy is None:
            self.last_odom_xy = (x, y)
            self.last_moving_time = rospy.Time.now()
            return

        dx = x - self.last_odom_xy[0]
        dy = y - self.last_odom_xy[1]
        dist = math.hypot(dx, dy)

        now = rospy.Time.now()

        if dist > self.stuck_dist_eps:
            # dianggap gerak, reset anchor & timer
            self.last_odom_xy = (x, y)
            self.last_moving_time = now
        else:
            # ga banyak gerak, cek berapa lama
            if self.last_moving_time is None:
                self.last_moving_time = now
            dt = (now - self.last_moving_time).to_sec()
            if dt > self.stuck_timeout:
                rospy.logwarn(f"[agentic_planner] STUCK detected (no movement > {self.stuck_timeout}s)")
                self.navigation_active = False
                self.set_state("RECOVERY")
                self.pub_recovery.publish(String(data="STUCK"))
                # NOTE: node lain yang memutuskan apa yg dilakukan (clear costmap, cmd_vel khusus, dll.)


if __name__ == "__main__":
    try:
        node = AgenticPlannerBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
