#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int32


class AprilTagApproachNode:
    def __init__(self):
        # Parameters (can be set from launch)
        self.tag_topic       = rospy.get_param("~tag_topic", "/tag_detections")
        self.cmd_vel_topic   = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.target_distance = rospy.get_param("~target_distance", 0.32)
        self.target_sideways = rospy.get_param("~target_sideways", 0.118)
        self.target_id       = rospy.get_param("~target_id", -1)

        self.k_side          = rospy.get_param("~k_side", 0.8)
        self.k_forward       = rospy.get_param("~k_forward", 0.8)
        self.k_yaw           = rospy.get_param("~k_yaw", 1.0)

        self.max_side_vel    = rospy.get_param("~max_side_vel", 0.25)
        self.max_forward_vel = rospy.get_param("~max_forward_vel", 0.3)
        self.max_yaw_vel     = rospy.get_param("~max_yaw_vel", 0.8)

        self.tol_x           = rospy.get_param("~tol_x", 0.01)
        self.tol_z           = rospy.get_param("~tol_z", 0.02)
        self.tol_yaw         = rospy.get_param("~tol_yaw", 0.02)

        self.timeout         = rospy.get_param("~timeout", 0.5)

        # --- ADDED ---
        self.advance_distance = rospy.get_param("~advance_distance", 0.6)
        self.advance_speed    = rospy.get_param("~advance_speed", 0.15)

        # Internal state
        self.current_x = None
        self.current_z = None
        self.current_ori_z = None
        self.last_detection_time = None
        self.stage = 0

        self.active = False

        # --- ADDED advancing state ---
        self.advancing = False
        self.advance_start_time = None
        self.advance_duration = None
        self.advance_done = False

        # Publishers / Subscribers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.tag_sub = rospy.Subscriber(self.tag_topic, AprilTagDetectionArray, self.tag_callback)

        self.gate_sub = rospy.Subscriber("/activate_apriltag_approacher", Int32, self.gate_callback, queue_size=1)
        self.success_pub = rospy.Publisher("/successful_mark", Int32, queue_size=1)
        self.last_success_value = None

        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop)

        rospy.loginfo("AprilTagApproachNode initialized.")
        rospy.loginfo("  target_distance = %.3f m", self.target_distance)
        rospy.loginfo("  target_id       = %d ( -1 means 'any')", self.target_id)
        rospy.loginfo("  advance_distance = %.3f m, advance_speed = %.3f m/s",
                      self.advance_distance, self.advance_speed)

        rospy.on_shutdown(self.on_shutdown)

    def gate_callback(self, msg):
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[AprilTagApproachNode] Gate=1 -> ACTIVATED (resetting stage)")
                # reset stages and advance logic
                self.stage = 0
                self.advancing = False
                self.advance_start_time = None
                self.advance_duration = None
                self.advance_done = False
            self.active = True
        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[AprilTagApproachNode] Gate=0 -> DEACTIVATED, stopping and resetting")
            self.active = False
            self.current_x = None
            self.current_z = None
            self.current_ori_z = None
            self.last_detection_time = None
            # reset advance state
            self.advancing = False
            self.advance_start_time = None
            self.advance_duration = None
            self.advance_done = False
            # reset stage as well
            self.stage = 0
            self._publish_stop()
            self._publish_success(0)
        else:
            rospy.logwarn("[AprilTagApproachNode] Unknown gate value %d", msg.data)


    def _publish_stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def _publish_success(self, value):
        if value != self.last_success_value:
            msg = Int32()
            msg.data = int(value)
            self.success_pub.publish(msg)
            self.last_success_value = value

    def tag_callback(self, msg):
        if not msg.detections:
            return

        chosen_det = None
        chosen_z = None

        for det in msg.detections:
            det_id = det.id[0] if det.id else None
            if self.target_id >= 0 and det_id != self.target_id:
                continue
            z = det.pose.pose.pose.position.z
            if chosen_det is None or z < chosen_z:
                chosen_det = det
                chosen_z = z

        if chosen_det is None:
            return

        pose = chosen_det.pose.pose.pose
        pos = pose.position
        ori = pose.orientation

        x_cam = pos.x
        z_cam = pos.z
        ori_z = ori.z

        self.current_x = x_cam
        self.current_z = z_cam
        self.current_ori_z = ori_z
        rospy.loginfo("[AprilTagApproachNode] %.3f || %.3f || %.3f", self.current_x, self.current_z, self.current_ori_z)
        self.last_detection_time = rospy.Time.now()

    def control_loop(self, event):
        twist = Twist()

        if not self.active:
            self._publish_success(0)
            return

        # Handle advance phase (unchanged)
        if self.advancing:
            elapsed = (rospy.Time.now() - self.advance_start_time).to_sec()
            if elapsed < self.advance_duration:
                forward_vel = min(abs(self.advance_speed), self.max_forward_vel)
                twist.linear.x = forward_vel
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                # self._publish_success(1)
                return
            else:
                rospy.loginfo("[AprilTagApproachNode] Advance completed (%.3f m).", self.advance_distance)
                self.advancing = False
                self.advance_start_time = None
                self.advance_duration = None
                self.advance_done = True
                self._publish_stop()
                self._publish_success(1)
                return

        # If we haven't seen a tag recently, stop
        if self.last_detection_time is None:
            self._publish_stop()
            self._publish_success(0)
            return

        dt = (rospy.Time.now() - self.last_detection_time).to_sec()
        if dt > self.timeout or self.current_x is None or self.current_z is None:
            self._publish_stop()
            self._publish_success(0)
            return

        # Errors in camera frame
        err_x = self.current_x - self.target_sideways                          # sideways error
        err_z = self.current_z - self.target_distance   # depth error

        # Position tolerance check
        position_ok = (abs(err_x) <= self.tol_x) and (abs(err_z) <= self.tol_z)

        # Yaw error (still using ori.z as your proxy)
        yaw_ok = True
        err_yaw = 0.0
        if self.current_ori_z is not None:
            err_yaw = self.current_ori_z
            if abs(err_yaw) > self.tol_yaw:
                yaw_ok = False

        # -------------------------------
        # Combined controller:
        #   - If position not OK: move in x,z AND rotate
        #   - If position OK but yaw not OK: rotate only
        #   - If both OK: trigger advance logic
        # -------------------------------

        if not position_ok:
            # Approach + yaw at the same time
            if abs(err_x) > self.tol_x:
                side_vel = -self.k_side * err_x
                side_vel = max(-self.max_side_vel, min(self.max_side_vel, side_vel))
            else:
                side_vel = 0.0

            if abs(err_z) > self.tol_z:
                forward_vel = self.k_forward * err_z
                forward_vel = max(-self.max_forward_vel, min(self.max_forward_vel, forward_vel))
            else:
                forward_vel = 0.0

            yaw_vel = self.k_yaw * err_yaw if not yaw_ok else 0.0
            yaw_vel = max(-self.max_yaw_vel, min(self.max_yaw_vel, yaw_vel))

            twist.linear.x = forward_vel
            twist.linear.y = side_vel
            twist.angular.z = yaw_vel

            self.cmd_pub.publish(twist)
            self._publish_success(0)
            return

        # Here: position_ok is True
        if not yaw_ok:
            # Final yaw cleanup: rotate in place only
            yaw_vel = self.k_yaw * err_yaw
            yaw_vel = max(-self.max_yaw_vel, min(self.max_yaw_vel, yaw_vel))

            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = yaw_vel

            self.cmd_pub.publish(twist)
            self._publish_success(0)
            return

        # If we reach this point: position_ok and yaw_ok are both True
        if (not self.advance_done) and (not self.advancing):
            actual_speed = min(max(1e-3, abs(self.advance_speed)), self.max_forward_vel)
            self.advance_duration = float(self.advance_distance) / float(actual_speed)
            self.advance_start_time = rospy.Time.now()
            self.advancing = True
            rospy.loginfo(
                "[AprilTagApproachNode] Position+Yaw OK -> starting advance %.3f m at %.3f m/s (duration %.3f s)",
                self.advance_distance, actual_speed, self.advance_duration
            )
            # self._publish_success(1)
            return
        else:
            # self._publish_success(1)
            print("haha")


    def on_shutdown(self):
        self._publish_stop()
        self._publish_success(0)
        rospy.sleep(0.1)
        self._publish_stop()

if __name__ == "__main__":
    rospy.init_node("apriltag_approach_node")
    node = AprilTagApproachNode()
    rospy.spin()
