#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int32, String


class EnteringStoreNode(object):
    def __init__(self):
        # ---------------- Parameters ----------------
        self.yolo_topic        = rospy.get_param("~yolo_topic", "/yolo_rs/xy")
        self.cmd_vel_topic     = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.gate_topic        = rospy.get_param("~gate_topic", "/activate_entering_store")
        self.target_label_topic = rospy.get_param("~target_label_topic",
                                                  "/entering_store_target_label")

        # Default target label (YOLO frame_id)
        self.target_label      = rospy.get_param("~target_label", "cafe")

        # Image size (pixels) – you can override from launch
        self.image_width       = rospy.get_param("~image_width", 960.0)
        self.image_height      = rospy.get_param("~image_height", 540.0)
        self.center_tolerance  = rospy.get_param("~center_tolerance_px", 20.0)

        # Stage 1: rotation
        self.rotation_speed    = rospy.get_param("~rotation_speed", 0.5)  # rad/s
        self.rotation_angle    = rospy.get_param("~rotation_angle", math.pi / 2.0)  # 90 deg
        # derived
        self.turn_duration     = abs(self.rotation_angle) / max(1e-6, abs(self.rotation_speed))

        # Stage 2: sideways translation
        self.sideways_speed    = rospy.get_param("~sideways_speed", 0.25)  # m/s

        # Detection timeout
        self.timeout           = rospy.get_param("~timeout", 0.5)  # [s]

        # Control rate
        self.control_rate      = rospy.get_param("~control_rate", 20.0)  # Hz

        # ---------------- Internal state ----------------
        # States: "IDLE", "WAIT_DETECTION", "STAGE1_ROTATE", "STAGE2_SIDEWAYS", "DONE"
        self.state = "IDLE"

        self.active = False  # Gated OFF by default

        # Last detection info for target label
        self.last_detection_x = None
        self.last_detection_time = None

        # Stage decisions
        self.initial_side = None          # "LEFT" or "RIGHT"
        self.turn_direction = None        # "LEFT" or "RIGHT"
        self.sideways_direction = None    # "LEFT" or "RIGHT"

        self.rotation_start_time = None

        # ---------------- ROS I/O ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        self.yolo_sub = rospy.Subscriber(
            self.yolo_topic, PointStamped, self.yolo_callback, queue_size=1
        )

        self.gate_sub = rospy.Subscriber(
            self.gate_topic, Int32, self.gate_callback, queue_size=1
        )

        self.target_label_sub = rospy.Subscriber(
            self.target_label_topic, String, self.target_label_callback, queue_size=1
        )

        # Timer for main control loop
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop
        )

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("[EnteringStoreNode] Initialized.")
        rospy.loginfo("  yolo_topic   = %s", self.yolo_topic)
        rospy.loginfo("  cmd_vel_topic= %s", self.cmd_vel_topic)
        rospy.loginfo("  gate_topic   = %s", self.gate_topic)
        rospy.loginfo("  target_label = '%s'", self.target_label)
        rospy.loginfo("  image_width  = %.1f", self.image_width)

    # ---------------- Gate & label callbacks ----------------

    def gate_callback(self, msg):
        """
        /activate_entering_store: Int32
          1 -> active
          0 -> inactive (stop & reset)
        """
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[EnteringStoreNode] Gate=1 -> ACTIVATED")
            self.active = True
            self.reset_for_new_sequence()
            self.state = "WAIT_DETECTION"
        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[EnteringStoreNode] Gate=0 -> DEACTIVATED, stopping and resetting.")
            self.active = False
            self.stop()
            self.reset_for_new_sequence()
            self.state = "IDLE"
        else:
            rospy.logwarn("[EnteringStoreNode] Unknown gate value %d (expected 0 or 1)", msg.data)

    def target_label_callback(self, msg):
        """
        /entering_store_target_label: String
        Set which YOLO label (frame_id) we want to approach (e.g. 'cafe').
        """
        self.target_label = msg.data.strip()
        rospy.loginfo("[EnteringStoreNode] Target label set to '%s'", self.target_label)

    def reset_for_new_sequence(self):
        """Reset all internal state for a fresh approach sequence."""
        self.last_detection_x = None
        self.last_detection_time = None
        self.initial_side = None
        self.turn_direction = None
        self.sideways_direction = None
        self.rotation_start_time = None

    # ---------------- YOLO callback ----------------

    def yolo_callback(self, msg):
        """
        /yolo_rs/xy: PointStamped

        Example:
          header.frame_id = "hamburger_store"
          point.x = 569.0 (pixel x)
          point.y = 372.0 (pixel y)
        """
        if not self.active:
            return

        label = msg.header.frame_id

        # Only care about our target label
        if label != self.target_label:
            return

        self.last_detection_x = msg.point.x
        self.last_detection_time = rospy.Time.now()

        rospy.loginfo("[Detected X] x=%.2f , label= %s",self.last_detection_x,label)
        # If we are waiting for a detection to start the sequence
        if self.state in ["WAIT_DETECTION", "IDLE"]:
            self.decide_stage1_from_detection(self.last_detection_x)

    def decide_stage1_from_detection(self, x_px):
        """
        Decide rotation direction and sideways direction from initial detection.
        If object is to the right of image center => rotate RIGHT 90°, then move LEFT.
        If object is to the left of image center => rotate LEFT 90°, then move RIGHT.
        """
        cx = self.image_width * 0.5
        dx = x_px - cx

        if dx >= 0:
            self.initial_side = "RIGHT"
            self.turn_direction = "RIGHT"
            self.sideways_direction = "LEFT"
        else:
            self.initial_side = "LEFT"
            self.turn_direction = "LEFT"
            self.sideways_direction = "RIGHT"

        self.rotation_start_time = rospy.Time.now()
        self.state = "STAGE1_ROTATE"

        rospy.loginfo(
            "[EnteringStoreNode] Initial detection at x=%.1f (center=%.1f) -> side=%s, "
            "turn=%s 90deg, then move %s.",
            x_px, cx, self.initial_side, self.turn_direction, self.sideways_direction
        )

    # ---------------- Velocity helpers ----------------

    def yaw_speed_for_direction(self, direction):
        """
        "RIGHT" => negative yaw (clockwise)
        "LEFT"  => positive yaw (counter-clockwise)
        """
        if direction == "RIGHT":
            return -abs(self.rotation_speed)
        else:
            return abs(self.rotation_speed)

    def sideways_speed_for_direction(self, direction):
        """
        For a mecanum robot in base_link frame:
          vy > 0 => move left
          vy < 0 => move right
        """
        if direction == "LEFT":
            return abs(self.sideways_speed)
        else:
            return -abs(self.sideways_speed)

    def publish_velocity(self, vx, vy, wz):
        twist = Twist()
        twist.linear.x  = vx
        twist.linear.y  = vy
        twist.linear.z  = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

    def stop(self):
        self.publish_velocity(0.0, 0.0, 0.0)

    # ---------------- Main control loop ----------------

    def control_loop(self, event):
        if not self.active:
            # No command spam, just do nothing while inactive
            return

        now = rospy.Time.now()

        # ---------------- STATE: IDLE ----------------
        if self.state == "IDLE":
            # Wait until gate turns us into WAIT_DETECTION
            self.stop()
            return

        # ---------------- STATE: WAIT_DETECTION ----------------
        if self.state == "WAIT_DETECTION":
            # Just wait for yolo_callback() to see target_label and start STAGE1_ROTATE
            self.stop()
            return

        # ---------------- STATE: STAGE1_ROTATE ----------------
        if self.state == "STAGE1_ROTATE":
            if self.rotation_start_time is None or self.turn_direction is None:
                # Something inconsistent; reset to WAIT_DETECTION
                rospy.logwarn("[EnteringStoreNode] STAGE1_ROTATE but rotation_start_time/turn_direction not set. Resetting.")
                self.state = "WAIT_DETECTION"
                self.stop()
                return

            elapsed = (now - self.rotation_start_time).to_sec()

            if elapsed < self.turn_duration:
                wz = self.yaw_speed_for_direction(self.turn_direction)
                self.publish_velocity(0.0, 0.0, wz)
            else:
                rospy.loginfo(
                    "[EnteringStoreNode] Finished 90° %s rotation (%.2fs). "
                    "Switching to STAGE2_SIDEWAYS.",
                    self.turn_direction, elapsed
                )
                self.state = "STAGE2_SIDEWAYS"
                self.stop()
            return

        # ---------------- STATE: STAGE2_SIDEWAYS ----------------
        if self.state == "STAGE2_SIDEWAYS":
            # Determine if we currently have a recent detection of target_label
            has_detection = False
            if self.last_detection_time is not None:
                dt = (now - self.last_detection_time).to_sec()
                if dt <= self.timeout:
                    has_detection = True

            vy = self.sideways_speed_for_direction(self.sideways_direction)

            if has_detection and self.last_detection_x is not None:
                cx = self.image_width * 0.5
                dx = self.last_detection_x - cx

                # If object is roughly in the center, we are done
                if abs(dx) <= self.center_tolerance:
                    rospy.loginfo(
                        "[EnteringStoreNode] Target '%s' centered (x=%.1f, center=%.1f, tol=%.1f). "
                        "Sideways approach complete.",
                        self.target_label, self.last_detection_x, cx, self.center_tolerance
                    )
                    self.stop()
                    self.state = "DONE"
                    return

                # Otherwise: keep moving sideways in the pre-decided direction
                rospy.logdebug_throttle(
                    1.0,
                    "[EnteringStoreNode] STAGE2_SIDEWAYS: detection x=%.1f (center=%.1f), "
                    "moving %s (vy=%.3f)",
                    self.last_detection_x, cx, self.sideways_direction, vy
                )
                self.publish_velocity(0.0, vy, 0.0)
            else:
                # No detection: keep moving sideways until object reappears or gate is turned off
                rospy.logdebug_throttle(
                    2.0,
                    "[EnteringStoreNode] STAGE2_SIDEWAYS: no detection, moving %s (vy=%.3f)",
                    self.sideways_direction, vy
                )
                self.publish_velocity(0.0, vy, 0.0)
            return

        # ---------------- STATE: DONE ----------------
        if self.state == "DONE":
            # Hold position until gate is toggled or external node decides what to do next
            self.stop()
            return

    # ---------------- Shutdown ----------------

    def on_shutdown(self):
        rospy.loginfo("[EnteringStoreNode] Shutdown, stopping robot.")
        self.stop()
        rospy.sleep(0.1)
        self.stop()


if __name__ == "__main__":
    rospy.init_node("entering_store_node")
    try:
        EnteringStoreNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
