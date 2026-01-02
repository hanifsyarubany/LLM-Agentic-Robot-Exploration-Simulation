#!/usr/bin/env python3
import rospy
import math
from threading import Lock
import json

from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Int32, String


class EnteringStoreNode(object):
    def __init__(self):
        # ---------------- Parameters ----------------
        self.cmd_vel_topic   = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.yolo_topic      = rospy.get_param("~yolo_topic", "/yolo_usb/center_point")
        self.gate_topic      = rospy.get_param("~gate_topic", "/activate_entering_store")
        self.target_label_topic = rospy.get_param("~target_label_topic",
                                                  "/entering_store_target_label")
        self.success_topic   = rospy.get_param("~success_topic",
                                               "/successful_mark_entering_store")

        # NEW: publish chosen final turn direction ("left"/"right")
        self.prev_turn_topic = rospy.get_param("~previous_turn_direction_topic",
                                               "/previous_turn_direction")

        # Camera image width (pixels) – you can override from launch
        self.image_width     = rospy.get_param("~image_width", 640)
        self.center_tolerance = rospy.get_param("~center_tolerance", 30.0)  # pixels

        # Control params
        self.rotation_speed  = rospy.get_param("~rotation_speed", 0.5)   # rad/s
        self.forward_speed   = rospy.get_param("~forward_speed", 0.25)   # m/s
        self.side_speed      = rospy.get_param("~side_speed", 0.13)      # m/s

        # Stage 1 rotation angles
        self.scan_angle_deg  = rospy.get_param("~scan_angle_deg", 30.0)  # for search
        self.full_angle_deg  = rospy.get_param("~full_angle_deg", 77.0)  # final face

        self.scan_angle      = math.radians(self.scan_angle_deg)
        self.full_angle      = math.radians(self.full_angle_deg)

        # Derived durations
        w = max(1e-6, abs(self.rotation_speed))
        self.scan_duration   = self.scan_angle / w
        self.full_duration   = self.full_angle / w

        # Forward distance in stage-1 fallback
        self.forward_distance = rospy.get_param("~forward_distance", 1.5)   # meters
        self.forward_duration = self.forward_distance / max(1e-6, self.forward_speed)

        # Detection timeout
        self.det_timeout     = rospy.get_param("~det_timeout", 0.5)  # seconds

        # Sideways max duration (safety)
        self.sideways_timeout = rospy.get_param("~sideways_timeout", 15.0)

        # ---------------- Publishers / Subscribers ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)

        self.gate_sub = rospy.Subscriber(
            self.gate_topic, Int32, self.gate_callback, queue_size=1
        )
        self.yolo_sub = rospy.Subscriber(
            self.yolo_topic, PointStamped, self.yolo_callback, queue_size=10
        )
        self.target_label_sub = rospy.Subscriber(
            self.target_label_topic, String, self.target_label_callback, queue_size=1
        )

        self.success_pub = rospy.Publisher(
            self.success_topic, String, queue_size=1, latch=True
        )

        # NEW: latched so late subscribers still get last decision
        self.prev_turn_pub = rospy.Publisher(
            self.prev_turn_topic, String, queue_size=1, latch=True
        )

        # ---------------- Internal state ----------------
        self.active = False            # gate state
        self.state = "IDLE"           # FSM state

        self.target_label = None      # e.g. ["cafe", "hamburger_store"] or similar

        # Last detection of target label from /yolo_rs/xy
        self.last_det_lock = Lock()
        self.last_det_time = None
        self.last_det_x = None  # pixel x
        self.last_det_y = None  # pixel y

        # Stage / motion bookkeeping
        self.search_attempt = 0           # Stage-1 attempt count (0 or 1)
        self.side_of_store = None         # "LEFT" or "RIGHT"
        self.has_pre_rotation = False     # True if we already rotated 45° toward store
        self.turn_direction = None        # "LEFT" or "RIGHT" for rotations
        self.sideways_direction = None    # "LEFT" or "RIGHT" for stage-3

        self.rotation_start_time = None
        self.forward_start_time = None
        self.sideways_start_time = None
        self.current_rotate_duration = None

        rospy.loginfo("[EnteringStoreNode] Initialised. Waiting for gate=1...")
        # Timer for control loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    # ----------------------------------------------------
    # Helpers
    # ----------------------------------------------------
    def publish_velocity(self, vx, vy, wz):
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.cmd_pub.publish(twist)

    def stop(self):
        self.publish_velocity(0.0, 0.0, 0.0)

    def yaw_speed_for_direction(self, direction):
        # RIGHT -> negative yaw, LEFT -> positive yaw
        return -abs(self.rotation_speed) if direction == "RIGHT" else abs(self.rotation_speed)

    def side_speed_for_direction(self, direction):
        # LEFT -> +y, RIGHT -> -y (assuming standard ROS base_link)
        return self.side_speed if direction == "LEFT" else -self.side_speed

    def has_recent_detection(self):
        """Check if we saw target label recently; return (bool, x_pixel)."""
        if self.target_label is None:
            return False, None

        with self.last_det_lock:
            if self.last_det_time is None:
                return False, None
            dt = (rospy.Time.now() - self.last_det_time).to_sec()
            if dt > self.det_timeout:
                return False, None
            return True, self.last_det_x

    def publish_success(self, status):
        msg = String()
        msg.data = status
        self.success_pub.publish(msg)
        rospy.loginfo("[EnteringStoreNode] successful_mark_entering_store: '%s'", status)

    # NEW
    def publish_turn_direction(self, direction):
        """Publish final turn direction decision as 'left' or 'right'."""
        if direction not in ["LEFT", "RIGHT"]:
            return
        msg = String()
        msg.data = direction.lower()
        self.prev_turn_pub.publish(msg)
        rospy.loginfo("[EnteringStoreNode] previous_turn_direction: '%s'", msg.data)

    def reset_internal(self):
        """Reset FSM internals but keep gate + target_label."""
        self.state = "IDLE" if not self.active else "STAGE1_CHECK_CURRENT"
        self.search_attempt = 0
        self.side_of_store = None
        self.has_pre_rotation = False
        self.turn_direction = None
        self.sideways_direction = None
        self.rotation_start_time = None
        self.forward_start_time = None
        self.sideways_start_time = None
        self.current_rotate_duration = None
        # We keep last_det_* so we can reuse if within timeout

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------
    def gate_callback(self, msg):
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[EnteringStoreNode] Gate=1 -> ACTIVATED")
            self.active = True
            # Start a fresh sequence
            self.reset_internal()
            self.state = "STAGE1_CHECK_CURRENT"
        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[EnteringStoreNode] Gate=0 -> DEACTIVATED, stopping and resetting.")
            self.active = False
            self.stop()
            self.reset_internal()
        else:
            rospy.logwarn("[EnteringStoreNode] Unknown gate value %d (expected 0 or 1)", msg.data)

    def target_label_callback(self, msg):
        self.target_label = json.loads(msg.data.strip())
        rospy.loginfo("[EnteringStoreNode] Target label set to '%s'", str(self.target_label))

    def yolo_callback(self, msg):
        """
        /yolo_rs/xy:
        header.frame_id: label string
        point.x, point.y: pixel position.
        """
        label = msg.header.frame_id
        if self.target_label is None or not (label in self.target_label):
            return

        with self.last_det_lock:
            self.last_det_time = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            self.last_det_x = msg.point.x
            self.last_det_y = msg.point.y

    # ----------------------------------------------------
    # FSM: Control loop
    # ----------------------------------------------------
    def control_loop(self, event):
        if not self.active:
            # Gate OFF -> do nothing, don't publish zero cmd_vel repeatedly
            return

        now = rospy.Time.now()

        # ---------------- STAGE 1: Identify where the store is ----------------
        if self.state == "STAGE1_CHECK_CURRENT":
            has_det, x = self.has_recent_detection()

            if has_det:
                # Object already in frame – decide left/right w.r.t center
                center_x = self.image_width / 2.0
                self.side_of_store = "LEFT" if x < center_x else "RIGHT"
                self.turn_direction = self.side_of_store  # rotate toward store

                # NEW: expose final turn direction decision
                self.publish_turn_direction(self.turn_direction)

                self.has_pre_rotation = False  # no 45° scan yet
                rospy.loginfo("[EnteringStoreNode] Stage1: object already visible on %s side.",
                              self.side_of_store)
                # Go to Stage 2 directly
                self.state = "STAGE2_ROTATE_FULL"
                self.rotation_start_time = None
                self.current_rotate_duration = None
                return
            else:
                # No detection -> start scan: rotate LEFT 45°
                rospy.loginfo("[EnteringStoreNode] Stage1: no object visible, scanning LEFT 45°.")
                self.state = "STAGE1_ROTATE_LEFT"
                self.rotation_start_time = now
                return

        elif self.state == "STAGE1_ROTATE_LEFT":
            elapsed = (now - self.rotation_start_time).to_sec()
            if elapsed < self.scan_duration:
                # Rotate left
                self.publish_velocity(0.0, 0.0, self.yaw_speed_for_direction("LEFT"))
                return
            else:
                # Done left 45° -> stop ONCE and check detection
                self.stop()
                has_det, _ = self.has_recent_detection()
                if has_det:
                    self.side_of_store = "LEFT"
                    self.turn_direction = "LEFT"
                    self.has_pre_rotation = True  # already rotated 45°

                    # NEW: expose final turn direction decision
                    self.publish_turn_direction(self.turn_direction)

                    rospy.loginfo("[EnteringStoreNode] Stage1: object found on LEFT after 45° left scan.")
                    # Next: Stage 2
                    self.state = "STAGE2_ROTATE_FULL"
                    self.rotation_start_time = None
                    self.current_rotate_duration = None
                    return
                else:
                    # Now rotate RIGHT 90° from current +45° -> end at -45°
                    rospy.loginfo("[EnteringStoreNode] Stage1: not found on LEFT, scanning RIGHT 90°.")
                    self.state = "STAGE1_ROTATE_RIGHT"
                    self.rotation_start_time = now
                    return

        elif self.state == "STAGE1_ROTATE_RIGHT":
            elapsed = (now - self.rotation_start_time).to_sec()
            # Need 90° = 2 * scan_duration (from +45° to -45°)
            if elapsed < 2.0 * self.scan_duration:
                self.publish_velocity(0.0, 0.0, self.yaw_speed_for_direction("RIGHT"))
                return
            else:
                # Done RIGHT 90° -> stop ONCE and check detection
                self.stop()
                has_det, _ = self.has_recent_detection()
                if has_det:
                    self.side_of_store = "RIGHT"
                    self.turn_direction = "RIGHT"
                    self.has_pre_rotation = True  # already rotated 45° toward store

                    # NEW: expose final turn direction decision
                    self.publish_turn_direction(self.turn_direction)

                    rospy.loginfo("[EnteringStoreNode] Stage1: object found on RIGHT after scans.")
                    self.state = "STAGE2_ROTATE_FULL"
                    self.rotation_start_time = None
                    self.current_rotate_duration = None
                    return
                else:
                    # Fail to find in this attempt
                    if self.search_attempt == 0:
                        # First failure: rotate back 45° to forward heading, then go forward
                        rospy.loginfo(
                            "[EnteringStoreNode] Stage1: not found LEFT/RIGHT, rotating back 45° to forward heading."
                        )
                        self.search_attempt = 1
                        self.state = "STAGE1_ROTATE_BACK"
                        self.rotation_start_time = now
                        return
                    else:
                        # Second failure: rotate back 45° to forward heading, then FAIL
                        rospy.logwarn(
                            "[EnteringStoreNode] Stage1: second attempt failed, rotating back 45° to forward heading and giving up."
                        )
                        self.state = "STAGE1_ROTATE_BACK_FINAL"
                        self.rotation_start_time = now
                        return

        elif self.state == "STAGE1_ROTATE_BACK":
            """
            We are currently at about -45° (after left+right scans).
            Rotate LEFT 45° to return to the original straight-forward heading,
            then move forward 1.5m.
            """
            elapsed = (now - self.rotation_start_time).to_sec()
            if elapsed < self.scan_duration:
                # Rotate back LEFT 45°
                self.publish_velocity(0.0, 0.0, self.yaw_speed_for_direction("LEFT"))
                return
            else:
                # Finished returning to forward heading -> stop ONCE, then go forward
                self.stop()
                rospy.loginfo(
                    "[EnteringStoreNode] Stage1: back to forward heading, now moving forward %.2fm and retry.",
                    self.forward_distance,
                )
                self.state = "STAGE1_FORWARD_MOVE"
                self.forward_start_time = now
                return

        elif self.state == "STAGE1_FORWARD_MOVE":
            elapsed = (now - self.forward_start_time).to_sec()
            if elapsed < self.forward_duration:
                self.publish_velocity(self.forward_speed, 0.0, 0.0)
                return
            else:
                # Done moving forward -> stop ONCE and restart Stage1
                self.stop()
                rospy.loginfo("[EnteringStoreNode] Stage1: forward move complete, retrying search.")
                self.state = "STAGE1_CHECK_CURRENT"
                return

        # ---------------- STAGE 2: Fully rotate toward store ----------------
        elif self.state == "STAGE2_ROTATE_FULL":
            if self.side_of_store is None or self.turn_direction is None:
                rospy.logwarn("[EnteringStoreNode] STAGE2 without side_of_store, aborting.")
                self.stop()  # one-time
                self.publish_success("failed")
                self.state = "DONE"
                return

            if self.rotation_start_time is None:
                # Decide how much extra rotation:
                #  - If we already rotated 45° in Stage1 -> only 45° more
                #  - Else -> full 90°
                if self.has_pre_rotation:
                    self.current_rotate_duration = self.scan_duration
                    rospy.loginfo("[EnteringStoreNode] Stage2: rotating additional 45° to face store (%s).",
                                  self.turn_direction)
                else:
                    self.current_rotate_duration = self.full_duration
                    rospy.loginfo("[EnteringStoreNode] Stage2: rotating full 90° to face store (%s).",
                                  self.turn_direction)
                self.rotation_start_time = now

            elapsed = (now - self.rotation_start_time).to_sec()
            if elapsed < self.current_rotate_duration:
                self.publish_velocity(0.0, 0.0, self.yaw_speed_for_direction(self.turn_direction))
                return
            else:
                # Done facing store -> stop ONCE
                self.stop()
                # Decide sideways direction:
                # If store is on RIGHT of the robot, we move LEFT sideways to reach it; and vice versa.
                self.sideways_direction = "LEFT" if self.side_of_store == "RIGHT" else "RIGHT"
                rospy.loginfo("[EnteringStoreNode] Stage2: finished rotation. Stage3 sideways direction: %s",
                              self.sideways_direction)
                # Prepare Stage3
                self.state = "STAGE3_SIDEWAYS"
                self.sideways_start_time = now
                return

        # ---------------- STAGE 3: Sideways until object centered ----------------
        elif self.state == "STAGE3_SIDEWAYS":
            if self.sideways_direction is None:
                rospy.logwarn("[EnteringStoreNode] STAGE3 without sideways_direction, aborting.")
                self.stop()  # one-time
                self.publish_success("failed")
                self.state = "DONE"
                return

            elapsed = (now - self.sideways_start_time).to_sec()
            if elapsed > self.sideways_timeout:
                rospy.logwarn("[EnteringStoreNode] Stage3: sideways timeout reached, failing.")
                self.stop()  # one-time
                self.publish_success("failed")
                self.state = "DONE"
                return

            has_det, x = self.has_recent_detection()
            center_x = self.image_width / 2.0

            if has_det and x is not None:
                offset = x - center_x
                if abs(offset) <= self.center_tolerance:
                    # Object is roughly at center of frame -> success
                    rospy.loginfo("[EnteringStoreNode] Stage3: object centered (offset=%.1f), SUCCESS.", offset)
                    self.stop()  # one-time
                    self.publish_success("success")
                    self.state = "DONE"
                    return

            # Otherwise keep moving sideways toward target
            vy = self.side_speed_for_direction(self.sideways_direction)
            self.publish_velocity(0.0, vy, 0.0)
            return

        # ---------------- IDLE / DONE / any other ----------------
        elif self.state in ["IDLE", "DONE"]:
            # Do NOT publish stop here repeatedly – just let other nodes control cmd_vel.
            return

        elif self.state == "STAGE1_ROTATE_BACK_FINAL":
            """
            Second overall failure case:
            We are currently at about -45°. Rotate LEFT 45° to return
            to the original straight-forward heading, then publish 'failed'
            and stop the FSM.
            """
            elapsed = (now - self.rotation_start_time).to_sec()
            if elapsed < self.scan_duration:
                # Rotate back LEFT 45°
                self.publish_velocity(0.0, 0.0, self.yaw_speed_for_direction("LEFT"))
                return
            else:
                # Finished returning to forward heading -> stop ONCE and fail
                self.stop()
                rospy.logwarn("[EnteringStoreNode] Stage1: final failure, robot heading restored to forward.")
                self.publish_success("failed")
                self.state = "DONE"
                return

        else:
            rospy.logwarn_throttle(2.0, "[EnteringStoreNode] Unknown state '%s'", self.state)
            # No repeated stop here either
            return


def main():
    rospy.init_node("entering_store_node")
    node = EnteringStoreNode()
    rospy.spin()


if __name__ == "__main__":
    main()

