#!/usr/bin/env python3
import rospy
import json
import math
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
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

        # Object grasping / storing integration
        self.object_grasp_gate_topic    = rospy.get_param(
            "~object_grasp_gate_topic", "/activate_object_approaching_grasping_node"
        )
        self.multi_object_success_topic = rospy.get_param(
            "~multi_object_success_topic", "/multi_object_task_success"
        )

        # NEW: pickup controller gate (SPECIAL CASE)
        self.object_pickup_gate_topic   = rospy.get_param("~object_pickup_gate_topic", "/activate_object_pickup")

        # Post-grasp turn
        self.previous_turn_topic        = rospy.get_param("~previous_turn_topic", "/previous_turn_direction")
        self.cmd_vel_topic              = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.post_grasp_turn_degrees    = float(rospy.get_param("~post_grasp_turn_degrees", 85.0))
        self.post_grasp_turn_speed      = float(rospy.get_param("~post_grasp_turn_speed", 0.6))  # [rad/s], +z = left

        # Tag-lost far/near behavior
        self.tag_lost_far_threshold     = float(rospy.get_param("~tag_lost_far_threshold", 0.735))  # [m]

        self.tag_timeout       = rospy.get_param("~tag_timeout", 0.5)        # [s]
        self.approach_cooldown = rospy.get_param("~approach_cooldown", 3.0)  # [s]

        # -------------------------------------------------------------
        # Publishers (latched gates)
        # -------------------------------------------------------------
        self.wall_gate_pub = rospy.Publisher(self.wall_gate_topic, Int32, queue_size=1, latch=True)
        self.apriltag_gate_pub = rospy.Publisher(self.apriltag_gate_topic, Int32, queue_size=1, latch=True)
        self.semantic_map_pub = rospy.Publisher(self.semantic_map_topic, Int32, queue_size=1, latch=True)

        self.pre_enter_pub = rospy.Publisher(self.pre_enter_topic, String, queue_size=1, latch=True)

        self.entering_gate_pub = rospy.Publisher(self.entering_gate_topic, Int32, queue_size=1, latch=True)
        self.entering_target_pub = rospy.Publisher(self.entering_target_topic, String, queue_size=1, latch=True)

        self.object_grasp_gate_pub = rospy.Publisher(self.object_grasp_gate_topic, Int32, queue_size=1, latch=True)

        # NEW: pickup gate publisher (latched)
        self.object_pickup_gate_pub = rospy.Publisher(self.object_pickup_gate_topic, Int32, queue_size=1, latch=True)

        # cmd_vel publisher for post-grasp turning
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        # -------------------------------------------------------------
        # Subscribers
        # -------------------------------------------------------------
        self.tag_sub = rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self.tag_callback, queue_size=1)
        self.success_sub = rospy.Subscriber(self.success_topic, Int32, self.success_callback, queue_size=1)
        self.state_sub = rospy.Subscriber("/wall_avoider_state", String, self.state_callback, queue_size=1)

        self.semantic_mapping_entries_sub = rospy.Subscriber(
            "/semantic_mapping_entries", String, self.semantic_mapping_entries_callback, queue_size=1
        )

        self.main_gate_sub = rospy.Subscriber(self.main_gate_topic, Int32, self.main_gate_callback, queue_size=1)
        self.next_action_sub = rospy.Subscriber(self.next_action_topic, String, self.next_action_callback, queue_size=1)

        self.pre_enter_finished_sub = rospy.Subscriber(
            self.pre_enter_finished_topic, String, self.pre_enter_finished_callback, queue_size=1
        )
        self.entering_success_sub = rospy.Subscriber(
            self.entering_success_topic, String, self.entering_success_callback, queue_size=1
        )

        self.multi_object_success_sub = rospy.Subscriber(
            self.multi_object_success_topic, Int32, self.multi_object_success_callback, queue_size=1
        )

        self.previous_turn_sub = rospy.Subscriber(
            self.previous_turn_topic, String, self.previous_turn_callback, queue_size=1
        )

        # -------------------------------------------------------------
        # Internal state
        #   States: "WALL_AVOID", "APRILTAG_APPROACH",
        #           "SEMANTIC_MAPPING", "WAIT_NEXT_ACTION",
        #           "STORE_PRE_ENTER", "STORE_ENTERING",
        #           "OBJECT_GRASPING", "POST_GRASP_TURN",
        #           "PICKUP_STATE"
        # -------------------------------------------------------------
        self.state = "WALL_AVOID"

        self.last_tag_time = None
        self.last_tag_distance = None
        self.last_success = 0
        self.last_approach_success_time = None

        self.semantic_map_entries = None

        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        self.dont_interrupt = False
        self.active = False

        self.pending_next_action = None
        self.current_preenter_direction = None
        self.current_action_target = None
        self.pre_enter_result = None
        self.entering_result = None

        self.multi_object_task_success = 0
        self.previous_turn_direction = None

        self.turn_start_time = None
        self.turn_duration = None
        self.turn_ang_z = 0.0

        # NEW: latch to avoid republishing pickup activation repeatedly
        self.pickup_activated = False

        rospy.loginfo("[MainController] Initializing...")
        rospy.sleep(0.5)

        # Ensure all behaviors start OFF
        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.semantic_map_pub.publish(off)
        self.entering_gate_pub.publish(off)
        self.object_grasp_gate_pub.publish(off)
        self.object_pickup_gate_pub.publish(off)

        # Ensure cmd_vel is zero at start (one-shot)
        self.cmd_vel_pub.publish(Twist())

        rospy.loginfo("[MainController] Start INACTIVE. Waiting for /activate_main_controller = 1.")

        self.timer = rospy.Timer(rospy.Duration(0.01), self.control_loop)

    # -------------------------------------------------------------
    # Gate callback for the whole main controller
    # -------------------------------------------------------------
    def main_gate_callback(self, msg):
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[MainController] Gate=1 -> ACTIVATED, switching to WALL_AVOID.")
                self.active = True

                self.semantic_mapping_active = False
                self.semantic_mapping_start_time = None

                self.pending_next_action = None
                self.pre_enter_result = None
                self.entering_result = None
                self.current_preenter_direction = None
                self.current_action_target = None

                self.multi_object_task_success = 0
                self.turn_start_time = None
                self.turn_duration = None
                self.turn_ang_z = 0.0

                self.pickup_activated = False

                self.switch_to_wall_avoid()

        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[MainController] Gate=0 -> DEACTIVATED, disabling all behaviors.")
            self.active = False

            self.semantic_mapping_active = False
            self.semantic_mapping_start_time = None

            self.state = "WALL_AVOID"
            self.pending_next_action = None
            self.pre_enter_result = None
            self.entering_result = None
            self.current_preenter_direction = None
            self.current_action_target = None

            self.multi_object_task_success = 0
            self.turn_start_time = None
            self.turn_duration = None
            self.turn_ang_z = 0.0

            self.pickup_activated = False

            off = Int32(data=0)
            self.wall_gate_pub.publish(off)
            self.apriltag_gate_pub.publish(off)
            self.semantic_map_pub.publish(off)
            self.entering_gate_pub.publish(off)
            self.object_grasp_gate_pub.publish(off)
            self.object_pickup_gate_pub.publish(off)

            # Stop robot motion (one-shot)
            self.cmd_vel_pub.publish(Twist())
        else:
            rospy.logwarn("[MainController] Unknown gate value %d (expected 0 or 1)", msg.data)

    # -------------------------------------------------------------
    # Other callbacks
    # -------------------------------------------------------------
    def semantic_mapping_entries_callback(self, msg):
        try:
            junction = json.loads(msg.data)
            if not isinstance(junction, dict):
                raise ValueError("semantic_mapping_entries must be a JSON object")

            self.semantic_map_entries = junction
            name = junction.get("name", "<no_name>")
            num_pairs = len(junction.get("poi_pairs", []))
            rospy.loginfo("[MainController] Received semantic junction '%s' with %d poi_pairs.", name, num_pairs)
        except Exception as e:
            rospy.logerr("[MainController] Failed to decode semantic mapping JSON: %s", str(e))

    def state_callback(self, msg):
        self.dont_interrupt = (msg.data != "FORWARD")

    def tag_callback(self, msg):
        if msg.detections:
            self.last_tag_time = rospy.Time.now()
            z_list = []
            for det in msg.detections:
                try:
                    z_list.append(det.pose.pose.pose.position.z)
                except Exception:
                    continue
            if z_list:
                self.last_tag_distance = min(z_list)

    def success_callback(self, msg):
        self.last_success = msg.data
        if self.state == "APRILTAG_APPROACH" and msg.data == 1:
            rospy.loginfo("[MainController] SUCCESS detected. Switching to SEMANTIC_MAPPING...")
            self.start_semantic_mapping()

    def next_action_callback(self, msg):
        # If we are already in pickup forever-state, ignore any further commands
        if self.state == "PICKUP_STATE":
            return

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

        self.pending_next_action = {"direction": dir_part, "target": target_part}
        rospy.loginfo("[MainController] Received next_controller_action: direction=%s, target=%s",
                      dir_part, target_part)

    def pre_enter_finished_callback(self, msg):
        self.pre_enter_result = msg.data.strip().lower()
        rospy.loginfo("[MainController] pre_entering_finished: %s", self.pre_enter_result)

    def entering_success_callback(self, msg):
        self.entering_result = msg.data.strip().lower()
        rospy.loginfo("[MainController] successful_mark_entering_store: %s", self.entering_result)

    def multi_object_success_callback(self, msg):
        self.multi_object_task_success = msg.data
        rospy.loginfo("[MainController] /multi_object_task_success = %d", self.multi_object_task_success)

    def previous_turn_callback(self, msg):
        v = msg.data.strip().lower()
        if v in ("left", "right"):
            self.previous_turn_direction = v
        else:
            rospy.logwarn_throttle(2.0, "[MainController] /previous_turn_direction invalid: '%s'", msg.data)

    # -------------------------------------------------------------
    # Mode switches
    # -------------------------------------------------------------
    def switch_to_wall_avoid(self):
        if self.state != "WALL_AVOID":
            rospy.loginfo("[MainController] Switching to WALL_AVOID mode.")

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        self.wall_gate_pub.publish(msg_on)
        self.apriltag_gate_pub.publish(msg_off)
        self.state = "WALL_AVOID"

    def switch_to_apriltag_approach(self):
        if self.state != "APRILTAG_APPROACH":
            rospy.loginfo("[MainController] Switching to APRILTAG_APPROACH mode (tag detected).")

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_on)
        self.state = "APRILTAG_APPROACH"

    def start_semantic_mapping(self):
        rospy.loginfo("[MainController] Entering SEMANTIC_MAPPING state for %.2fs.", self.semantic_mapping_duration)

        msg_on = Int32(data=1)
        msg_off = Int32(data=0)

        self.wall_gate_pub.publish(msg_off)
        self.apriltag_gate_pub.publish(msg_off)
        self.semantic_map_pub.publish(msg_on)

        self.pending_next_action = None

        self.semantic_mapping_active = True
        self.semantic_mapping_start_time = rospy.Time.now()
        self.state = "SEMANTIC_MAPPING"

    def stop_semantic_mapping(self, now):
        rospy.loginfo("[MainController] Semantic mapping finished. Turning OFF and waiting for /next_controller_action.")

        self.semantic_map_pub.publish(Int32(data=0))
        self.semantic_mapping_active = False
        self.semantic_mapping_start_time = None

        self.last_approach_success_time = now

        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.entering_gate_pub.publish(off)

        self.pre_enter_result = None
        self.entering_result = None
        self.current_preenter_direction = None
        self.current_action_target = None

        self.state = "WAIT_NEXT_ACTION"
        rospy.loginfo("[MainController] Now in WAIT_NEXT_ACTION. Robot should be standing still.")

    # -------------------------------------------------------------
    # Helper handlers for store pipeline
    # -------------------------------------------------------------
    def handle_next_action(self, direction, target):
        self.pre_enter_result = None
        self.entering_result = None
        self.current_preenter_direction = direction
        self.current_action_target = target

        rospy.loginfo("[MainController] Starting pre-entering: direction=%s, target=%s", direction, target)

        self.pre_enter_pub.publish(String(data=direction))
        self.state = "STORE_PRE_ENTER"

    def _enter_pickup_state(self):
        """
        SPECIAL: Activate /activate_object_pickup = 1, then remain in PICKUP_STATE forever.
        """
        if not self.pickup_activated:
            rospy.loginfo("[MainController] Target='pickup' -> Activating /activate_object_pickup = 1 and entering PICKUP_STATE forever.")
            self.object_pickup_gate_pub.publish(Int32(data=1))
            self.pickup_activated = True

        # Make sure other behaviors are OFF (one-shot publishes are ok; these are latched topics anyway)
        off = Int32(data=0)
        self.wall_gate_pub.publish(off)
        self.apriltag_gate_pub.publish(off)
        self.semantic_map_pub.publish(off)
        self.entering_gate_pub.publish(off)
        self.object_grasp_gate_pub.publish(off)

        self.state = "PICKUP_STATE"

    def handle_store_pre_enter_state(self):
        if self.pre_enter_result is None:
            return

        if self.pre_enter_result != "success":
            rospy.logwarn("[MainController] pre_entering finished with status '%s', returning to WALL_AVOID.", self.pre_enter_result)
            self.current_preenter_direction = None
            self.current_action_target = None
            self.pre_enter_result = None
            self.switch_to_wall_avoid()
            return

        target = self.current_action_target
        direction = self.current_preenter_direction

        rospy.loginfo("[MainController] pre_entering SUCCESS. direction=%s, target=%s", direction, target)

        self.current_preenter_direction = None
        self.pre_enter_result = None

        # SPECIAL CASE: pickup
        if target == "pickup":
            self.current_action_target = None
            self._enter_pickup_state()
            return

        if target == "continue":
            rospy.loginfo("[MainController] next_action target=continue -> WALL_AVOID.")
            self.current_action_target = None
            self.switch_to_wall_avoid()
            return

        # Normal store entering
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
        if self.entering_result is None:
            return

        self.entering_gate_pub.publish(Int32(data=0))

        if self.entering_result != "success":
            rospy.logwarn("[MainController] entering_store finished with status '%s'. Returning to WALL_AVOID.", self.entering_result)
            self.current_action_target = None
            self.entering_result = None
            self.switch_to_wall_avoid()
            return

        rospy.loginfo("[MainController] entering_store SUCCESS. Triggering object grasp/store node...")

        self.multi_object_task_success = 0
        self.object_grasp_gate_pub.publish(Int32(data=1))

        self.current_action_target = None
        self.entering_result = None
        self.state = "OBJECT_GRASPING"

    # -------------------------------------------------------------
    # Post-grasp turning helpers
    # -------------------------------------------------------------
    def start_post_grasp_turn(self, now):
        direction = self.previous_turn_direction if self.previous_turn_direction in ("left", "right") else "left"
        direction = "right" if direction == "left" else "left"

        angle_rad = abs(self.post_grasp_turn_degrees) * math.pi / 180.0
        speed = abs(self.post_grasp_turn_speed) if abs(self.post_grasp_turn_speed) > 1e-6 else 0.6
        duration = angle_rad / speed

        ang_z = speed if direction == "left" else -speed

        self.turn_start_time = now
        self.turn_duration = duration
        self.turn_ang_z = ang_z
        self.state = "POST_GRASP_TURN"

        rospy.loginfo("[MainController] Post-grasp turn: dir=%s, deg=%.1f, duration=%.2fs, ang_z=%.3f",
                      direction, self.post_grasp_turn_degrees, duration, ang_z)

        t = Twist()
        t.angular.z = ang_z
        self.cmd_vel_pub.publish(t)

    def stop_post_grasp_turn(self):
        self.cmd_vel_pub.publish(Twist())
        self.turn_start_time = None
        self.turn_duration = None
        self.turn_ang_z = 0.0

    # -------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------
    def control_loop(self, event):
        if not self.active:
            rospy.logdebug_throttle(2.0, "[MainController] Inactive (main gate=0), skipping FSM.")
            return

        now = rospy.Time.now()

        # PICKUP_STATE: stay here forever, do nothing else
        if self.state == "PICKUP_STATE":
            rospy.logdebug_throttle(2.0, "[MainController] PICKUP_STATE active (forever).")
            return

        # OBJECT_GRASPING
        if self.state == "OBJECT_GRASPING":
            if self.multi_object_task_success == 1:
                rospy.loginfo("[MainController] Object grasp/store done. Starting post-grasp turn...")

                self.object_grasp_gate_pub.publish(Int32(data=0))
                self.multi_object_task_success = 0

                self.start_post_grasp_turn(now)
            else:
                rospy.logdebug_throttle(1.0, "[MainController] OBJECT_GRASPING: waiting for /multi_object_task_success...")
            return

        # POST_GRASP_TURN
        if self.state == "POST_GRASP_TURN":
            if self.turn_start_time is None or self.turn_duration is None:
                rospy.logwarn("[MainController] POST_GRASP_TURN missing timing; stopping and returning to WALL_AVOID.")
                self.stop_post_grasp_turn()
                self.switch_to_wall_avoid()
                return

            elapsed = (now - self.turn_start_time).to_sec()
            if elapsed < self.turn_duration:
                t = Twist()
                t.angular.z = self.turn_ang_z
                self.cmd_vel_pub.publish(t)
                rospy.logdebug_throttle(1.0, "[MainController] POST_GRASP_TURN in progress...")
                return

            rospy.loginfo("[MainController] Post-grasp turn complete. Returning to WALL_AVOID.")
            self.stop_post_grasp_turn()
            self.switch_to_wall_avoid()
            return

        # SEMANTIC_MAPPING
        if self.state == "SEMANTIC_MAPPING":
            if not self.semantic_mapping_active or self.semantic_mapping_start_time is None:
                rospy.logwarn("[MainController] SEMANTIC_MAPPING state but not active, reverting to WALL_AVOID.")
                self.switch_to_wall_avoid()
                return

            elapsed = (now - self.semantic_mapping_start_time).to_sec()
            if elapsed < self.semantic_mapping_duration:
                rospy.logdebug_throttle(1.0, "[MainController] Semantic mapping in progress...")
                return

            self.stop_semantic_mapping(now)
            return

        # WAIT_NEXT_ACTION
        if self.state == "WAIT_NEXT_ACTION":
            if self.pending_next_action is None:
                rospy.logdebug_throttle(1.0, "[MainController] WAIT_NEXT_ACTION: waiting for command...")
                return

            action = self.pending_next_action
            self.pending_next_action = None
            self.handle_next_action(action["direction"], action["target"])
            return

        # STORE_PRE_ENTER
        if self.state == "STORE_PRE_ENTER":
            self.handle_store_pre_enter_state()
            return

        # STORE_ENTERING
        if self.state == "STORE_ENTERING":
            self.handle_store_entering_state()
            return

        # Normal WALL_AVOID / APRILTAG_APPROACH
        tag_recent = False
        if self.last_tag_time is not None:
            if (now - self.last_tag_time).to_sec() <= self.tag_timeout:
                tag_recent = True

        cooldown_active = False
        if self.last_approach_success_time is not None:
            if (now - self.last_approach_success_time).to_sec() < self.approach_cooldown:
                cooldown_active = True

        if self.state == "WALL_AVOID":
            if tag_recent and not cooldown_active and not self.dont_interrupt:
                self.switch_to_apriltag_approach()

        elif self.state == "APRILTAG_APPROACH":
            if not tag_recent:
                dist = self.last_tag_distance
                if dist is None:
                    rospy.logwarn_throttle(2.0, "[MainController] Tag lost during approach (no distance). Switching to wall avoidance.")
                    self.switch_to_wall_avoid()
                    return

                if dist > self.tag_lost_far_threshold:
                    rospy.logwarn_throttle(
                        2.0,
                        "[MainController] Tag lost during approach and was far (z=%.2f > %.2f). Switching to wall avoidance.",
                        dist, self.tag_lost_far_threshold
                    )
                    self.switch_to_wall_avoid()
                else:
                    rospy.logwarn_throttle(
                        2.0,
                        "[MainController] Tag lost during approach but was near (z=%.2f <= %.2f). Staying in APRILTAG_APPROACH.",
                        dist, self.tag_lost_far_threshold
                    )


if __name__ == "__main__":
    rospy.init_node("main_controller_node")
    try:
        MainControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
