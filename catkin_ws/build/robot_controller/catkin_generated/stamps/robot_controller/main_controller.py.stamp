#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import Int32, String
from apriltag_ros.msg import AprilTagDetectionArray


class MainControllerNode(object):
    def __init__(self):
        # Parameters (can be overridden from launch)
        self.tag_topic                  = rospy.get_param("~tag_topic", "/tag_detections")
        self.wall_gate_topic            = rospy.get_param("~wall_gate_topic", "/activate_wall_avoider")
        self.semantic_map_topic         = rospy.get_param("~semantic_map_topic", "/detecting_sign")
        self.apriltag_gate_topic        = rospy.get_param("~apriltag_gate_topic", "/activate_apriltag_approacher")
        self.success_topic              = rospy.get_param("~success_topic", "/successful_mark")
        self.semantic_mapping_duration  = rospy.get_param("~semantic_mapping_duration", 2.2)
        self.main_gate_topic            = rospy.get_param("~main_gate_topic", "/activate_main_controller")

        # How long a tag is considered "recent" after last detection
        self.tag_timeout       = rospy.get_param("~tag_timeout", 0.5)   # [s]
        # Cooldown after a successful approach before allowing another one
        self.approach_cooldown = rospy.get_param("~approach_cooldown", 3.0)  # [s]

        # Publishers to gate the behavior nodes (latched!)
        self.wall_gate_pub = rospy.Publisher(
            self.wall_gate_topic, Int32, queue_size=1, latch=True
        )
        self.apriltag_gate_pub = rospy.Publisher(
            self.apriltag_gate_topic, Int32, queue_size=1, latch=True
        )
        self.semantic_map_pub = rospy.Publisher(
            self.semantic_map_topic, Int32, queue_size=1, latch=True
        )

        # Subscribers
        self.tag_sub = rospy.Subscriber(
            self.tag_topic, AprilTagDetectionArray, self.tag_callback, queue_size=1
        )
        self.success_sub = rospy.Subscriber(
            self.success_topic, Int32, self.success_callback, queue_size=1
        )
        self.state_sub = rospy.Subscriber(
            "/wall_avoider_state", String, self.state_callback, queue_size=1
        )
        self.semantic_mapping_entries_sub = rospy.Subscriber(
            "/semantic_mapping_entries",
            String,
            self.semantic_mapping_entries_callback,
            queue_size=1
        )
        self.main_gate_sub = rospy.Subscriber(
            self.main_gate_topic, Int32, self.main_gate_callback, queue_size=1
        )

        # Internal state
        # States: "WALL_AVOID", "APRILTAG_APPROACH", "SEMANTIC_MAPPING"
        self.state = "WALL_AVOID"

        self.last_tag_time = None
        self.last_success = 0
        self.last_approach_success_time = None  # used for cooldown

        self.semantic_map_entries = None

        # Semantic mapping timing
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Dont Interrupt (when wall avoider is turning)
        self.dont_interrupt = False

        rospy.loginfo("[MainController] Initializing...")

        self.active = True

        rospy.sleep(0.5)

        # Ensure semantic mapping starts OFF
        self.semantic_map_pub.publish(Int32(data=0))

        # Start in wall-avoid mode
        self.switch_to_wall_avoid()

        # Periodic control loop
        self.timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)

        rospy.loginfo("[MainController] Ready. Starting in WALL_AVOID mode.")

    # ---------------------- callbacks ----------------------

    def main_gate_callback(self, msg):
        """
        Gate control for the main controller:
          - 1 = active: FSM runs, behaviors controlled normally
          - 0 = inactive: all behaviors OFF, FSM frozen
        """
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[MainController] Gate=1 -> ACTIVATED, switching to WALL_AVOID.")
                self.active = True

                # Reset any semantic mapping window
                self.semantic_mapping_active = False
                self.semantic_mapping_start_time = None

                # Re-start in wall-avoid mode
                self.switch_to_wall_avoid()
        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[MainController] Gate=0 -> DEACTIVATED, disabling all behaviors.")
            self.active = False

            # Stop any semantic mapping window
            self.semantic_mapping_active = False
            self.semantic_mapping_start_time = None

            # Turn OFF all behavior gates
            off = Int32(data=0)
            self.wall_gate_pub.publish(off)
            self.apriltag_gate_pub.publish(off)
            self.semantic_map_pub.publish(off)
        else:
            rospy.logwarn("[MainController] Unknown gate value %d (expected 0 or 1)", msg.data)

    def semantic_mapping_entries_callback(self, msg):
        """
        msg.data is a JSON array string, e.g.:
        '[{"metadata_name": "...", "direction_label": "left", ...}, {...}]'
        """
        try:
            self.semantic_map_entries = json.loads(msg.data)
        except json.JSONDecodeError as e:
            rospy.logerr("Failed to decode semantic mapping JSON: %s", str(e))

    def state_callback(self, msg):
        """
        msg.data: "FORWARD", "FIRST_TURN", "SECOND_TURN"
        Used only to avoid interrupting wall-avoider while it is turning.
        """
        if msg.data == "FORWARD":
            self.dont_interrupt = False
        else:
            self.dont_interrupt = True

    def tag_callback(self, msg):
        """
        Any AprilTag detection can trigger the switch to approach mode.
        """
        if msg.detections:
            self.last_tag_time = rospy.Time.now()

    def success_callback(self, msg):
        """
        /successful_mark from apriltag_approach_node.
        When it becomes 1 while in APRILTAG_APPROACH:
            -> Go to SEMANTIC_MAPPING state
            -> control_loop handles the 2s window and cooldown + return to wall avoid.
        """
        self.last_success = msg.data

        if self.state == "APRILTAG_APPROACH" and msg.data == 1:
            rospy.loginfo("[MainController] SUCCESS detected. Switching to SEMANTIC_MAPPING...")
            self.start_semantic_mapping()

    # ---------------------- mode switches ----------------------

    def switch_to_wall_avoid(self):
        """
        Activate local_map_wall_avoider, deactivate apriltag_approach_node.
        NOTE: does NOT touch semantic mapping gate; that is controlled separately.
        """
        if self.state != "WALL_AVOID":
            rospy.loginfo("[MainController] Switching to WALL_AVOID mode.")

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        # Clear last semantic map entries (optional)
        # self.semantic_map_entries = None

        # Wall avoider ON, Apriltag approacher OFF
        self.wall_gate_pub.publish(msg_on)
        self.apriltag_gate_pub.publish(msg_off)

        self.state = "WALL_AVOID"

    def switch_to_apriltag_approach(self):
        """
        Activate apriltag_approach_node, deactivate local_map_wall_avoider.
        """
        if self.state != "APRILTAG_APPROACH":
            rospy.loginfo("[MainController] Switching to APRILTAG_APPROACH mode (tag detected).")

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        # AprilTag approach ON, wall avoider OFF
        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_on)

        self.state = "APRILTAG_APPROACH"

    def start_semantic_mapping(self):
        """
        Enter SEMANTIC_MAPPING state:
          - both motion behaviors OFF
          - semantic mapper activated
          - robot holds while mapping is done
        """
        rospy.loginfo(
            "[MainController] Entering SEMANTIC_MAPPING state for %.2fs.",
            self.semantic_mapping_duration
        )

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        # Turn OFF motion behaviors
        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_off)

        # Turn ON semantic mapping
        self.semantic_map_pub.publish(msg_on)

        self.semantic_mapping_active = True
        self.semantic_mapping_start_time = rospy.Time.now()
        self.state = "SEMANTIC_MAPPING"

    def stop_semantic_mapping(self, now):
        """
        Stop semantic mapping, start cooldown, return to wall avoid.
        """
        rospy.loginfo("[MainController] Semantic mapping finished. Turning OFF and returning to WALL_AVOID.")

        # Turn OFF semantic mapping gate
        self.semantic_map_pub.publish(Int32(data=0))
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Start cooldown AFTER mapping, so we don't immediately re-trigger
        self.last_approach_success_time = now

        # Back to wall avoid behavior
        self.switch_to_wall_avoid()

    # ------------------------ main loop ------------------------

    def control_loop(self, event):
        if not self.active:
            rospy.logdebug_throttle(
                2.0, "[MainController] Inactive (main gate=0), skipping FSM."
            )
            return

        now = rospy.Time.now()

        # ------------------------------------------------------------
        # 1. SEMANTIC MAPPING state: just wait for the duration
        # ------------------------------------------------------------
        if self.state == "SEMANTIC_MAPPING":
            if not self.semantic_mapping_active or self.semantic_mapping_start_time is None:
                rospy.logwarn("[MainController] SEMANTIC_MAPPING state but not active, reverting to WALL_AVOID.")
                self.switch_to_wall_avoid()
                return

            elapsed = (now - self.semantic_mapping_start_time).to_sec()

            if elapsed < self.semantic_mapping_duration:
                # Still in mapping window: do nothing else
                rospy.logwarn("[Semantic MAPPING] Still Mapping...")
                return

            # Mapping window ended
            self.stop_semantic_mapping(now)
            return

        # ------------------------------------------------------------
        # 2. Normal state machine (WALL_AVOID / APRILTAG_APPROACH)
        # ------------------------------------------------------------

        # Is a tag "recently" visible?
        tag_recent = False
        if self.last_tag_time is not None:
            if (now - self.last_tag_time).to_sec() <= self.tag_timeout:
                tag_recent = True

        # Cooldown after a success to avoid immediate re-triggering
        cooldown_active = False
        if self.last_approach_success_time is not None:
            if (now - self.last_approach_success_time).to_sec() < self.approach_cooldown:
                cooldown_active = True

        # WALL_AVOID → APRILTAG_APPROACH
        if self.state == "WALL_AVOID":
            if tag_recent and not cooldown_active and not self.dont_interrupt:
                self.switch_to_apriltag_approach()

        # APRILTAG_APPROACH → WALL_AVOID (if tag lost)
        elif self.state == "APRILTAG_APPROACH":
            if not tag_recent:
                rospy.logwarn_throttle(
                    2.0,
                    "[MainController] Tag lost during approach, still waiting....."
                )
                # self.switch_to_wall_avoid()


if __name__ == "__main__":
    rospy.init_node("main_controller_node")
    try:
        MainControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
