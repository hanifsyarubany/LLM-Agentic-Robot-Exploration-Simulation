#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import Int32, String
from apriltag_ros.msg import AprilTagDetectionArray


class MainControllerNode(object):
    def __init__(self):
        # -------------------------------------------------------------
        # Parameters
        # -------------------------------------------------------------
        self.tag_topic                 = rospy.get_param("~tag_topic", "/tag_detections")
        self.wall_gate_topic           = rospy.get_param("~wall_gate_topic", "/activate_wall_avoider")
        self.semantic_map_topic        = rospy.get_param("~semantic_map_topic", "/detecting_sign")
        self.apriltag_gate_topic       = rospy.get_param("~apriltag_gate_topic", "/activate_apriltag_approacher")
        self.success_topic             = rospy.get_param("~success_topic", "/successful_mark")
        self.semantic_mapping_duration = rospy.get_param("~semantic_mapping_duration", 2.2)
        self.main_gate_topic           = rospy.get_param("~main_gate_topic", "/activate_main_controller")

        # Pre-entering / entering-store integration
        self.pre_enter_topic           = rospy.get_param("~pre_enter_topic", "/activate_pre_entering")
        self.pre_enter_finished_topic  = rospy.get_param("~pre_enter_finished_topic", "/pre_entering_finished")
        self.entering_gate_topic       = rospy.get_param("~entering_gate_topic", "/activate_entering_store")
        self.entering_target_topic     = rospy.get_param("~entering_target_topic", "/entering_store_target_label")
        self.entering_success_topic    = rospy.get_param("~entering_success_topic", "/successful_mark_entering_store")
        self.next_action_topic         = rospy.get_param("~next_action_topic", "/next_controller_action")

        # How long a tag is considered "recent" after last detection
        self.tag_timeout       = rospy.get_param("~tag_timeout", 0.5)   # [s]
        # Cooldown after a successful approach (to avoid re-triggering immediately)
        self.approach_cooldown = rospy.get_param("~approach_cooldown", 3.0)  # [s]

        # -------------------------------------------------------------
        # Publishers (latched gates)
        # -------------------------------------------------------------
        self.wall_gate_pub = rospy.Publisher(
            self.wall_gate_topic, Int32, queue_size=1, latch=True
        )
        self.apriltag_gate_pub = rospy.Publisher(
            self.apriltag_gate_topic, Int32, queue_size=1, latch=True
        )
        self.semantic_map_pub = rospy.Publisher(
            self.semantic_map_topic, Int32, queue_size=1, latch=True
        )
        self.pre_enter_pub = rospy.Publisher(
            self.pre_enter_topic, String, queue_size=1, latch=True
        )
        self.entering_gate_pub = rospy.Publisher(
            self.entering_gate_topic, Int32, queue_size=1, latch=True
        )
        self.entering_target_pub = rospy.Publisher(
            self.entering_target_topic, String, queue_size=1, latch=True
        )

        # -------------------------------------------------------------
        # Subscribers
        # -------------------------------------------------------------
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
        self.next_action_sub = rospy.Subscriber(
            self.next_action_topic, String, self.next_action_callback, queue_size=1
        )
        self.pre_enter_finished_sub = rospy.Subscriber(
            self.pre_enter_finished_topic, String, self.pre_enter_finished_callback, queue_size=1
        )
        self.entering_success_sub = rospy.Subscriber(
            self.entering_success_topic, String, self.entering_success_callback, queue_size=1
        )

        # -------------------------------------------------------------
        # Internal state
        #   States: "WALL_AVOID", "APRILTAG_APPROACH",
        #           "SEMANTIC_MAPPING", "WAIT_NEXT_ACTION",
        #           "STORE_PRE_ENTER", "STORE_ENTERING"
        # -------------------------------------------------------------
        self.state = "WALL_AVOID"

        self.last_tag_time = None
        self.last_success = 0
        self.last_approach_success_time = None  # used for cooldown

        # Latest junction info (from semantic mapping) – optional
        self.semantic_map_entries = None

        # Semantic mapping timing
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Don't interrupt wall-avoider while it's rotating
        self.dont_interrupt = False

        # Main controller gate
        self.active = False  # IMPORTANT: start INACTIVE

        # Next-controller-action pipeline
        self.pending_next_action = None          # {"direction":..., "target":...}
        self.current_preenter_direction = None   # "left" / "right" / "straight"
        self.current_action_target = None        # "continue" or store label
        self.pre_enter_result = None             # "success" / "failed" / None
        self.entering_result = None              # "success" / "failed" / None

        rospy.loginfo("[MainController] Initializing...")

        rospy.sleep(0.5)

        # Ensure all behaviors start OFF
        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.semantic_map_pub.publish(off)
        self.entering_gate_pub.publish(off)

        rospy.loginfo("[MainController] Start INACTIVE. "
                      "Waiting for /activate_main_controller = 1.")

        # Periodic control loop
        self.timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)

    # -------------------------------------------------------------
    # Gate callback for the whole main controller
    # -------------------------------------------------------------
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

                # Clear store / entering pipeline
                self.pending_next_action = None
                self.pre_enter_result = None
                self.entering_result = None
                self.current_preenter_direction = None
                self.current_action_target = None

                # Start in wall-avoid mode
                self.switch_to_wall_avoid()

        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[MainController] Gate=0 -> DEACTIVATED, disabling all behaviors.")
            self.active = False

            # Stop any semantic mapping window
            self.semantic_mapping_active = False
            self.semantic_mapping_start_time = None

            # Reset FSM-ish flags
            self.state = "WALL_AVOID"
            self.pending_next_action = None
            self.pre_enter_result = None
            self.entering_result = None
            self.current_preenter_direction = None
            self.current_action_target = None

            # Turn OFF all behavior gates
            off = Int32(data=0)
            self.wall_gate_pub.publish(off)
            self.apriltag_gate_pub.publish(off)
            self.semantic_map_pub.publish(off)
            self.entering_gate_pub.publish(off)
        else:
            rospy.logwarn("[MainController] Unknown gate value %d (expected 0 or 1)", msg.data)

    # -------------------------------------------------------------
    # Other callbacks
    # -------------------------------------------------------------
    def semantic_mapping_entries_callback(self, msg):
        """
        /semantic_mapping_entries sends ONE JSON object per message, e.g.:

          {
            "name": "junction_4",
            "poi_pairs": ["right --- pharmacy", "straight --- hamburger_store"],
            "created_at_unix": ...,
            "robot_pose_2d": {...}
          }

        We keep it around for any external debugging or future logic,
        but the *decision* is now made via /next_controller_action.
        """
        try:
            junction = json.loads(msg.data)
            if not isinstance(junction, dict):
                raise ValueError("semantic_mapping_entries must be a JSON object")

            self.semantic_map_entries = junction
            name = junction.get("name", "<no_name>")
            num_pairs = len(junction.get("poi_pairs", []))
            rospy.loginfo(
                "[MainController] Received semantic junction '%s' with %d poi_pairs.",
                name, num_pairs
            )
        except Exception as e:
            rospy.logerr("[MainController] Failed to decode semantic mapping JSON: %s", str(e))

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
            -> control_loop handles mapping window and then WAIT_NEXT_ACTION.
        """
        self.last_success = msg.data

        if self.state == "APRILTAG_APPROACH" and msg.data == 1:
            rospy.loginfo("[MainController] SUCCESS detected. Switching to SEMANTIC_MAPPING...")
            self.start_semantic_mapping()

    def next_action_callback(self, msg):
        """
        /next_controller_action: e.g. "straight---cafe", "right---continue", "left---pharmacy".
        We store it and use it when in WAIT_NEXT_ACTION.
        """
        text = msg.data.strip()
        if not text:
            return

        try:
            dir_part, target_part = [p.strip().lower() for p in text.split("---", 1)]
        except ValueError:
            rospy.logwarn("[MainController] Invalid next_controller_action format: '%s'", text)
            return

        if dir_part not in ("left", "right", "straight"):
            rospy.logwarn("[MainController] next_controller_action has invalid direction '%s'", dir_part)
            return

        self.pending_next_action = {
            "direction": dir_part,
            "target":   target_part,  # "continue" or store label
        }
        rospy.loginfo("[MainController] Received next_controller_action: direction=%s, target=%s",
                      dir_part, target_part)

    def pre_enter_finished_callback(self, msg):
        """
        /pre_entering_finished: expected "success" or "failed".
        """
        self.pre_enter_result = msg.data.strip().lower()
        rospy.loginfo("[MainController] pre_entering_finished: %s", self.pre_enter_result)

    def entering_success_callback(self, msg):
        """
        /successful_mark_entering_store: expected "success" or "failed".
        """
        self.entering_result = msg.data.strip().lower()
        rospy.loginfo("[MainController] successful_mark_entering_store: %s", self.entering_result)

    # -------------------------------------------------------------
    # Mode switches
    # -------------------------------------------------------------
    def switch_to_wall_avoid(self):
        """
        Activate local_map_wall_avoider, deactivate apriltag_approach_node.
        NOTE: does NOT touch semantic mapping gate or entering-store gate.
        """
        if self.state != "WALL_AVOID":
            rospy.loginfo("[MainController] Switching to WALL_AVOID mode.")

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

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

        # Clear any old next_action pending
        self.pending_next_action = None

        self.semantic_mapping_active = True
        self.semantic_mapping_start_time = rospy.Time.now()
        self.state = "SEMANTIC_MAPPING"

    def stop_semantic_mapping(self, now):
        """
        Stop semantic mapping, go to WAIT_NEXT_ACTION and let /next_controller_action decide.
        """
        rospy.loginfo("[MainController] Semantic mapping finished. Turning OFF and waiting for /next_controller_action.")

        # Turn OFF semantic mapping gate
        self.semantic_map_pub.publish(Int32(data=0))
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Start cooldown AFTER mapping, so we don't immediately re-trigger Apriltag
        self.last_approach_success_time = now

        # Ensure motion behaviors are OFF while waiting
        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.entering_gate_pub.publish(off)

        # Clear store pipeline state
        self.pre_enter_result = None
        self.entering_result = None
        self.current_preenter_direction = None
        self.current_action_target = None

        # Now just wait for /next_controller_action
        self.state = "WAIT_NEXT_ACTION"
        rospy.loginfo("[MainController] Now in WAIT_NEXT_ACTION. Robot should be standing still.")

    # -------------------------------------------------------------
    # Helper handlers for store pipeline
    # -------------------------------------------------------------
    def handle_next_action(self, direction, target):
        """
        Called while in WAIT_NEXT_ACTION when a valid next_controller_action arrives.
        'direction' in {left,right,straight}, 'target' is e.g. 'cafe' or 'continue'.
        """
        self.pre_enter_result = None
        self.entering_result = None
        self.current_preenter_direction = direction
        self.current_action_target = target

        # In all other cases, we first do pre-entering.
        # (For straight---store, pre_entering node will just move forward 1m, no rotation.)
        rospy.loginfo("[MainController] Starting pre-entering: direction=%s, target=%s",
                      direction, target)

        # Trigger pre_entering_node by publishing the direction string
        self.pre_enter_pub.publish(String(data=direction))

        self.state = "STORE_PRE_ENTER"

    def handle_store_pre_enter_state(self):
        """
        Wait for /pre_entering_finished. Then either:
          - if target == 'continue' -> WALL_AVOID
          - else -> start entering_store_node with that target label.
        """
        if self.pre_enter_result is None:
            # pre-entering still running
            return

        if self.pre_enter_result != "success":
            rospy.logwarn("[MainController] pre_entering finished with status '%s', "
                          "returning to WALL_AVOID.",
                          self.pre_enter_result)
            self.current_preenter_direction = None
            self.current_action_target = None
            self.pre_enter_result = None
            self.switch_to_wall_avoid()
            return

        # success
        target = self.current_action_target
        direction = self.current_preenter_direction

        rospy.loginfo("[MainController] pre_entering SUCCESS. direction=%s, target=%s",
                      direction, target)

        self.current_preenter_direction = None
        self.pre_enter_result = None

        if target == "continue":
            rospy.loginfo("[MainController] next_action target=continue -> WALL_AVOID.")
            self.current_action_target = None
            self.switch_to_wall_avoid()
        else:
            # Start entering_store_node with this target
            labels = [target]
            self.entering_result = None
            try:
                payload = json.dumps(labels)
            except Exception:
                payload = json.dumps([str(target)])

            self.entering_target_pub.publish(String(data=payload))
            self.entering_gate_pub.publish(Int32(data=1))
            rospy.loginfo("[MainController] Activating entering_store for target(s): %s", labels)

            self.state = "STORE_ENTERING"

    def handle_store_entering_state(self):
        """
        Wait for /successful_mark_entering_store, then disable that node
        and go back to WALL_AVOID.
        """
        if self.entering_result is None:
            # entering_store still running
            return

        # Stop entering_store node
        self.entering_gate_pub.publish(Int32(data=0))

        if self.entering_result != "success":
            rospy.logwarn("[MainController] entering_store finished with status '%s'. "
                          "Returning to WALL_AVOID.",
                          self.entering_result)
        else:
            rospy.loginfo("[MainController] entering_store SUCCESS. Returning to WALL_AVOID.")

        self.current_action_target = None
        self.entering_result = None
        self.switch_to_wall_avoid()

    # -------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------
    def control_loop(self, event):
        if not self.active:
            rospy.logdebug_throttle(
                2.0, "[MainController] Inactive (main gate=0), skipping FSM."
            )
            return

        now = rospy.Time.now()

        # 1) SEMANTIC MAPPING state: wait for duration
        if self.state == "SEMANTIC_MAPPING":
            if not self.semantic_mapping_active or self.semantic_mapping_start_time is None:
                rospy.logwarn("[MainController] SEMANTIC_MAPPING state but not active, "
                              "reverting to WALL_AVOID.")
                self.switch_to_wall_avoid()
                return

            elapsed = (now - self.semantic_mapping_start_time).to_sec()

            if elapsed < self.semantic_mapping_duration:
                # Still in mapping window: do nothing else
                rospy.logdebug_throttle(1.0, "[MainController] Semantic mapping in progress...")
                return

            # Mapping window ended
            self.stop_semantic_mapping(now)
            return

        # 2) WAIT_NEXT_ACTION: wait for /next_controller_action
        if self.state == "WAIT_NEXT_ACTION":
            if self.pending_next_action is None:
                # Just stand still; all gates are OFF
                rospy.logdebug_throttle(1.0, "[MainController] WAIT_NEXT_ACTION: waiting for command...")
                return

            # Consume and handle one action
            action = self.pending_next_action
            self.pending_next_action = None
            direction = action["direction"]
            target = action["target"]
            self.handle_next_action(direction, target)
            return

        # 3) STORE_PRE_ENTER: wait for pre_entering result
        if self.state == "STORE_PRE_ENTER":
            self.handle_store_pre_enter_state()
            return

        # 4) STORE_ENTERING: wait for entering_store result
        if self.state == "STORE_ENTERING":
            self.handle_store_entering_state()
            return

        # 5) Normal state machine (WALL_AVOID / APRILTAG_APPROACH)
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
                    "[MainController] Tag lost during approach, returning to wall avoidance."
                )
                #self.switch_to_wall_avoid()


if __name__ == "__main__":
    rospy.init_node("main_controller_node")
    try:
        MainControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
