#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class AgenticPlanner:
    def __init__(self):
        rospy.init_node("agentic_planner")

        # ====== PARAMETER ======
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.input_topic   = rospy.get_param("~input_topic", "/planner_input")
        self.state_topic   = rospy.get_param("~state_topic", "/planner_state")

        # topic sensor ultrasonic (dari URM04 di xacro)
        self.front_topic = rospy.get_param("~front_topic", "/front_sensor")
        self.left_topic  = rospy.get_param("~left_topic", "/left_sensor")
        self.right_topic = rospy.get_param("~right_topic", "/right_sensor")

        # kecepatan dasar
        self.v_forward   = rospy.get_param("~v_forward", 0.20)
        self.w_turn      = rospy.get_param("~w_turn", 0.7)

        # threshold jarak (meter)
        self.front_safe  = rospy.get_param("~front_safe", 0.35)
        self.side_margin = rospy.get_param("~side_margin", 0.10)
        self.recovery_trigger = rospy.get_param("~recovery_trigger", 0.18)

        # ====== STATE INTERNAL ======
        self.mode = "IDLE"      # "IDLE", "EXPLORE", "RECOVERY", "GOTO"
        self.goal = None        # (x, y) kalau GO x y
        self.recovery_start = None

        # data sensor
        self.front = None
        self.left  = None
        self.right = None

        # ====== PUB/SUB ======
        self.cmd_pub   = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=10)

        rospy.Subscriber(self.input_topic, String, self.input_cb)
        rospy.Subscriber(self.front_topic, Range, self.front_cb)
        rospy.Subscriber(self.left_topic,  Range, self.left_cb)
        rospy.Subscriber(self.right_topic, Range, self.right_cb)

        # timer control loop
        rate = rospy.get_param("~rate", 10.0)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.control_loop)

        rospy.loginfo("[Planner] started. mode=IDLE")

    # ========== CALLBACKS ==========
    def input_cb(self, msg):
        text = msg.data.strip()
        rospy.loginfo("[Planner] input: %s", text)

        # --- case: angka lokasi stop (misal "1" = stop) ---
        if text == "1" or text.upper() == "STOP":
            self.mode = "IDLE"
            self.goal = None
            self.recovery_start = None
            rospy.loginfo("[Planner] --> mode IDLE (stop)")
            return

        # --- eksplorasi ---
        if text.upper() == "EXPLORE":
            self.mode = "EXPLORE"
            self.goal = None
            self.recovery_start = None
            rospy.loginfo("[Planner] --> mode EXPLORE")
            return

        # --- paksa recovery ---
        if text.upper() == "RECOVERY":
            self.mode = "RECOVERY"
            self.recovery_start = rospy.Time.now()
            rospy.loginfo("[Planner] --> mode RECOVERY (forced)")
            return

        # --- GO x y ---
        # contoh format: "GO 1.0 2.5"
        parts = text.split()
        if len(parts) == 3 and parts[0].upper() == "GO":
            try:
                x = float(parts[1])
                y = float(parts[2])
                self.goal = (x, y)
                self.mode = "GOTO"
                self.recovery_start = None
                rospy.loginfo("[Planner] --> mode GOTO (%.2f, %.2f)", x, y)
            except ValueError:
                rospy.logwarn("[Planner] cannot parse GO command: %s", text)
            return

        rospy.logwarn("[Planner] unknown command: '%s'", text)

    def front_cb(self, msg):
        self.front = msg.range

    def left_cb(self, msg):
        self.left = msg.range

    def right_cb(self, msg):
        self.right = msg.range

    # ========== CORE LOOP ==========
    def control_loop(self, event):
        cmd = Twist()

        # publish state tiap loop
        self.state_pub.publish(String(data=self.mode))

        # kalau belum ada data sensor sama sekali, jangan gerak
        if self.front is None or self.left is None or self.right is None:
            self.cmd_pub.publish(cmd)
            return

        if self.mode == "IDLE":
            # robot diam
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0

        elif self.mode == "EXPLORE":
            self.explore_logic(cmd)

            # auto trigger recovery kalau terlalu dekat
            if self.front < self.recovery_trigger:
                rospy.loginfo("[Planner] front=%.2f < %.2f -> RECOVERY",
                              self.front, self.recovery_trigger)
                self.mode = "RECOVERY"
                self.recovery_start = rospy.Time.now()

        elif self.mode == "RECOVERY":
            self.recovery_logic(cmd)

        elif self.mode == "GOTO":
            # belum diimplementasi, untuk sekarang: jalan lurus pelan
            # (biar nggak kosong total)
            cmd.linear.x  = 0.10
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ========== LOGIC PER STATE ==========
    def explore_logic(self, cmd):
        """Simple reaktif: maju kalau depan aman, belok kalau sempit."""
        # depan aman?
        if self.front > self.front_safe:
            # agak jaga jarak ke sisi yang lebih dekat
            cmd.linear.x = self.v_forward
            diff = (self.left - self.right)

            if abs(diff) > self.side_margin:
                # belok menjauhi sisi yang terlalu dekat
                cmd.angular.z = -0.4 * math.copysign(1.0, diff)
            else:
                cmd.angular.z = 0.0
        else:
            # depan sempit: pilih belok ke sisi yang lebih luas
            cmd.linear.x = 0.0
            if self.left > self.right:
                cmd.angular.z = +self.w_turn
            else:
                cmd.angular.z = -self.w_turn

    def recovery_logic(self, cmd):
        if self.recovery_start is None:
            self.recovery_start = rospy.Time.now()
        t = (rospy.Time.now() - self.recovery_start).to_sec()

        if t < 1.0:
            # 1 detik pertama: mundur
            cmd.linear.x  = -0.15
            cmd.angular.z = 0.0
        elif t < 2.5:
            # kemudian: putar di tempat
            cmd.linear.x  = 0.0
            cmd.angular.z = self.w_turn
        else:
            # selesai recovery -> balik EXPLORE
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.0
            self.mode = "EXPLORE"
            rospy.loginfo("[Planner] recovery done -> EXPLORE")


if __name__ == "__main__":
    try:
        planner = AgenticPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
