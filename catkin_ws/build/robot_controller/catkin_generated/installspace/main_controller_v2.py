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
        self.tag_timeout       = rospy.get_param("~tag_timeout", 0.5)    # [s]
        # Cooldown after a successful approach before allowing another one
        self.approach_cooldown = rospy.get_param("~approach_cooldown", 3.0)  # [s]

        # ------------------------------------------------------------------
        # Publishers to gate the behaviour nodes (latched!)
        # ------------------------------------------------------------------
        self.wall_gate_pub = rospy.Publisher(
            self.wall_gate_topic, Int32, queue_size=1, latch=True
        )
        self.apriltag_gate_pub = rospy.Publisher(
            self.apriltag_gate_topic, Int32, queue_size=1, latch=True
        )
        self.semantic_map_pub = rospy.Publisher(
            self.semantic_map_topic, Int32, queue_size=1, latch=True
        )

        # Store-entering related publishers
        self.pre_entering_trigger_pub = rospy.Publisher(
            "/activate_pre_entering", String, queue_size=1, latch=True
        )
        self.entering_store_gate_pub = rospy.Publisher(
            "/activate_entering_store", Int32, queue_size=1, latch=True
        )
        self.entering_store_target_pub = rospy.Publisher(
            "/entering_store_target_label", String, queue_size=1, latch=True
        )

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        # States: "WALL_AVOID", "APRILTAG_APPROACH", "SEMANTIC_MAPPING",
        #         "PRE_ENTERING", "ENTERING_STORE"
        self.state = "WALL_AVOID"

        self.last_tag_time = None
        self.last_success = 0
        self.last_approach_success_time = None  # used for cooldown

        # Latest semantic mapping entries (list of junction dicts)
        self.semantic_map_entries = None

        # Semantic mapping timing
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Dont Interrupt (when wall avoider is turning)
        self.dont_interrupt = False

        # Main gate (start INACTIVE)
        self.active = False

        # Store targets (from /store_targets), e.g. ["hamburger_store","cafe"]
        self.store_targets = []

        # Pre-entering / entering-store orchestration
        self.current_store_direction = None      # "left", "right", "straight"
        self.pre_entering_result = None          # last /pre_entering_finished
        self.entering_store_result = None        # last /successful_mark_entering_store

         # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
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
        self.store_targets_sub = rospy.Subscriber(
            "/store_targets", String, self.store_targets_callback, queue_size=1
        )
        self.pre_entering_finished_sub = rospy.Subscriber(
            "/pre_entering_finished", String, self.pre_entering_finished_callback, queue_size=1
        )
        self.entering_store_result_sub = rospy.Subscriber(
            "/successful_mark_entering_store", String, self.entering_store_result_callback, queue_size=1
        )


        rospy.loginfo("[MainController] Initializing...")

        rospy.sleep(0.5)

        # Ensure all behaviour gates are OFF at startup
        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.semantic_map_pub.publish(off)
        self.entering_store_gate_pub.publish(off)

        # DO NOT start wall-avoid until /activate_main_controller=1
        rospy.loginfo("[MainController] Started INACTIVE. Waiting for /activate_main_controller = 1.")

        # Periodic control loop
        self.timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)

    # ---------------------- callbacks ----------------------

    def main_gate_callback(self, msg):
        """
        Gate control for the main controller:
          - 1 = active: FSM runs, behaviours controlled normally
          - 0 = inactive: all behaviours OFF, FSM frozen
        """
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[MainController] Gate=1 -> ACTIVATED, switching to WALL_AVOID.")
                self.active = True

                # Reset mapping & store stages
                self.semantic_mapping_active = False
                self.semantic_mapping_start_time = None
                self.pre_entering_result = None
                self.entering_store_result = None
                self.current_store_direction = None

                # Start in wall-avoid mode
                self.switch_to_wall_avoid()
        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[MainController] Gate=0 -> DEACTIVATED, disabling all behaviours.")
            self.active = False

            # Stop any semantic mapping window
            self.semantic_mapping_active = False
            self.semantic_mapping_start_time = None

            # Turn OFF all behaviour gates
            off = Int32(data=0)
            self.wall_gate_pub.publish(off)
            self.apriltag_gate_pub.publish(off)
            self.semantic_map_pub.publish(off)
            self.entering_store_gate_pub.publish(off)
        else:
            rospy.logwarn("[MainController] Unknown gate value %d (expected 0 or 1)", msg.data)

    def semantic_mapping_entries_callback(self, msg):
        """
        /semantic_mapping_entries sends ONE JSON object per message, e.g.:

          {
            "name": "junction_4",
            "poi_pairs": ["right --- pharmacy", "straight --- hamburger_store"],
            "created_at_unix": 1765287730.9595,
            "robot_pose_2d": {...}
          }
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


    def store_targets_callback(self, msg):
        """
        /store_targets: JSON array of store labels, e.g.
          data: '[\"hamburger_store\",\"cafe\"]'
        """
        try:
            arr = json.loads(msg.data)
            if not isinstance(arr, list):
                raise ValueError("store_targets must be a JSON list")
            self.store_targets = [str(x) for x in arr]
            rospy.loginfo("[MainController] Updated store_targets: %s", self.store_targets)
        except Exception as e:
            rospy.logerr("[MainController] Failed to parse /store_targets JSON: %s", str(e))

    def pre_entering_finished_callback(self, msg):
        """
        /pre_entering_finished: expects "success" when pre-entering is done.
        """
        self.pre_entering_result = msg.data
        rospy.loginfo("[MainController] /pre_entering_finished = '%s'", msg.data)

    def entering_store_result_callback(self, msg):
        """
        /successful_mark_entering_store: expects "success" when entering-store is done.
        """
        self.entering_store_result = msg.data
        rospy.loginfo("[MainController] /successful_mark_entering_store = '%s'", msg.data)

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
            -> control_loop handles the mapping window and then
               either starts store-entering or returns to wall-avoid.
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
          - both motion behaviours OFF
          - semantic mapper activated
          - robot holds while mapping is done
        """
        rospy.loginfo(
            "[MainController] Entering SEMANTIC_MAPPING state for %.2fs.",
            self.semantic_mapping_duration
        )

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        # Turn OFF motion behaviours
        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_off)

        # Turn ON semantic mapping
        self.semantic_map_pub.publish(msg_on)

        self.semantic_mapping_active = True
        self.semantic_mapping_start_time = rospy.Time.now()
        self.state = "SEMANTIC_MAPPING"

    def stop_semantic_mapping(self, now):
        """
        Stop semantic mapping, decide whether to enter a store, or return to wall avoid.
        """
        rospy.loginfo("[MainController] Semantic mapping finished. Turning OFF and deciding next action.")

        # Turn OFF semantic mapping gate
        self.semantic_map_pub.publish(Int32(data=0))
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        # Start cooldown AFTER mapping so we don't immediately re-trigger
        self.last_approach_success_time = now

        # Decide whether to enter a store
        store_dir = self.choose_store_direction()

        # We won't reuse same mapping entries again
        self.semantic_map_entries = None

        if store_dir is not None:
            rospy.loginfo("[MainController] Found store direction '%s' -> starting PRE_ENTERING.", store_dir)
            self.start_pre_entering(store_dir)
        else:
            rospy.loginfo("[MainController] No target store at this junction -> back to WALL_AVOID.")
            self.switch_to_wall_avoid()

    # -------------------- store-entering helpers --------------------

    def choose_store_direction(self):
        """
        Use latest semantic_map_entries (ONE junction dict) and store_targets
        to decide which direction ("left/right/straight") to approach.

        Priority: order in self.store_targets.
        """
        if not self.store_targets:
            rospy.loginfo("[MainController] No store_targets set -> skip store-entering.")
            return None

        if not self.semantic_map_entries or not isinstance(self.semantic_map_entries, dict):
            rospy.loginfo("[MainController] No semantic_map_entries -> skip store-entering.")
            return None

        j = self.semantic_map_entries
        poi_pairs = j.get("poi_pairs", [])

        # Parse "right --- pharmacy" -> ("right", "pharmacy")
        parsed_pairs = []
        for s in poi_pairs:
            try:
                dir_part, poi_part = [p.strip() for p in s.split("---", 1)]
                parsed_pairs.append((dir_part, poi_part))
            except ValueError:
                continue

        # Priority by order in store_targets
        for target in self.store_targets:
            for direction, poi in parsed_pairs:
                if poi == target:
                    d_low = direction.lower()
                    if d_low in ("left", "right", "straight"):
                        rospy.loginfo(
                            "[MainController] Selected target '%s' with direction '%s' at %s.",
                            target,
                            d_low,
                            j.get("name", "unknown"),
                        )
                        return d_low

        # No matching store found at this junction
        return None


    def start_pre_entering(self, direction):
        """
        Start PRE_ENTERING phase:
          - all motion behaviours OFF
          - trigger pre_entering_node via /activate_pre_entering
        """
        msg_off = Int32(data=0)
        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_off)
        self.semantic_map_pub.publish(msg_off)

        self.current_store_direction = direction
        self.pre_entering_result = None
        self.entering_store_result = None

        # Trigger pre_entering_node
        self.pre_entering_trigger_pub.publish(String(data=direction))
        rospy.loginfo("[MainController] Sent /activate_pre_entering = '%s'", direction)

        self.state = "PRE_ENTERING"

    def start_entering_store(self):
        """
        Start ENTERING_STORE phase:
          - all motion behaviours OFF
          - /activate_entering_store = 1
          - /entering_store_target_label = same array as /store_targets
        """
        if not self.store_targets:
            rospy.logwarn("[MainController] start_entering_store called but store_targets empty -> back to WALL_AVOID.")
            self.switch_to_wall_avoid()
            return

        msg_off = Int32(data=0)
        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_off)
        self.semantic_map_pub.publish(msg_off)

        self.entering_store_result = None

        # Activate entering_store_node
        self.entering_store_gate_pub.publish(Int32(data=1))

        # Send target labels array
        labels_msg = String()
        labels_msg.data = json.dumps(self.store_targets)
        self.entering_store_target_pub.publish(labels_msg)

        rospy.loginfo("[MainController] Started ENTERING_STORE with targets: %s", self.store_targets)

        self.state = "ENTERING_STORE"

    # ------------------------ main loop ------------------------

    def control_loop(self, event):
        if not self.active:
            rospy.logdebug_throttle(
                2.0, "[MainController] Inactive (main gate=0), skipping FSM."
            )
            return

        now = rospy.Time.now()

        # ------------------------------------------------------------
        # A. PRE_ENTERING state: wait for /pre_entering_finished
        # ------------------------------------------------------------
        if self.state == "PRE_ENTERING":
            if self.pre_entering_result is None:
                # Still running pre_entering_node
                return

            # Once we have a result, decide next
            if self.pre_entering_result == "success":
                rospy.loginfo("[MainController] PRE_ENTERING finished SUCCESS -> starting ENTERING_STORE.")
                self.start_entering_store()
            else:
                rospy.logwarn(
                    "[MainController] PRE_ENTERING finished with '%s' -> back to WALL_AVOID.",
                    self.pre_entering_result
                )
                self.switch_to_wall_avoid()
            return

        # ------------------------------------------------------------
        # B. ENTERING_STORE state: wait for /successful_mark_entering_store
        # ------------------------------------------------------------
        if self.state == "ENTERING_STORE":
            if self.entering_store_result is None:
                # Still running entering_store_node
                return

            # Turn off entering_store gate
            self.entering_store_gate_pub.publish(Int32(data=0))

            if self.entering_store_result == "success":
                rospy.loginfo("[MainController] ENTERING_STORE finished SUCCESS -> back to WALL_AVOID.")
            else:
                rospy.logwarn(
                    "[MainController] ENTERING_STORE finished with '%s' -> back to WALL_AVOID.",
                    self.entering_store_result
                )

            self.switch_to_wall_avoid()
            return

        # ------------------------------------------------------------
        # C. SEMANTIC MAPPING state: just wait for the duration
        # ------------------------------------------------------------
        if self.state == "SEMANTIC_MAPPING":
            if not self.semantic_mapping_active or self.semantic_mapping_start_time is None:
                rospy.logwarn("[MainController] SEMANTIC_MAPPING state but not active, reverting to WALL_AVOID.")
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

        # ------------------------------------------------------------
        # D. Normal state machine (WALL_AVOID / APRILTAG_APPROACH)
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
                self.switch_to_wall_avoid()


if __name__ == "__main__":
    rospy.init_node("main_controller_node")
    try:
        MainControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
