#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class PreEnteringNode(object):
    def __init__(self):
        # Parameters
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # Motion params (can be overridden in launch)
        self.forward_distance = rospy.get_param("~forward_distance", 1.0)   # meters
        self.forward_speed    = rospy.get_param("~forward_speed", 0.25)    # m/s

        self.rotation_speed   = rospy.get_param("~rotation_speed", 0.5)    # rad/s
        rotation_angle_deg    = rospy.get_param("~rotation_angle_deg", 90.0)
        self.rotation_angle   = math.radians(rotation_angle_deg)           # radians

        # Derived durations
        self.forward_duration  = abs(self.forward_distance) / max(1e-6, abs(self.forward_speed))
        self.rotation_duration = abs(self.rotation_angle) / max(1e-6, abs(self.rotation_speed))

        # Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.done_pub = rospy.Publisher(
            "/pre_entering_finished", String, queue_size=1, latch=True
        )

        # Subscriber: trigger
        self.trigger_sub = rospy.Subscriber(
            "/activate_pre_entering", String, self.trigger_callback, queue_size=1
        )

        # Internal state
        # States: "IDLE", "FORWARD", "ROTATE"
        self.state = "IDLE"
        self.current_direction = None  # "left", "right", "straight"
        self.stage_start_time = None   # rospy.Time

        rospy.loginfo("[pre_entering_node] Initialized.")
        rospy.loginfo(
            "[pre_entering_node] forward_distance=%.2f m, forward_speed=%.2f m/s, rotation_angle=%.1f deg, rotation_speed=%.2f rad/s",
            self.forward_distance,
            self.forward_speed,
            math.degrees(self.rotation_angle),
            self.rotation_speed,
        )

        # Control loop timer
        self.timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)  # 50 Hz

    # ---------------------- helpers ----------------------

    def publish_velocity(self, vx, wz):
        """Helper to publish a Twist in (x, yaw) only."""
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def stop_once(self):
        """Publish a single stop command."""
        self.publish_velocity(0.0, 0.0)

    # ---------------------- callbacks ----------------------

    def trigger_callback(self, msg):
        """
        /activate_pre_entering: expects data in {"left","right","straight"}.
        Starts a new pre-entering sequence if currently IDLE.
        """
        if self.state != "IDLE":
            rospy.logwarn(
                "[pre_entering_node] Received trigger '%s' but already active (state=%s). Ignoring.",
                msg.data, self.state
            )
            return

        direction = msg.data.strip().lower()
        if direction not in {"left", "right", "straight"}:
            rospy.logwarn(
                "[pre_entering_node] Invalid trigger '%s'. Expected one of {left,right,straight}.",
                msg.data
            )
            return

        self.current_direction = direction
        self.state = "FORWARD"
        self.stage_start_time = rospy.Time.now()

        rospy.loginfo(
            "[pre_entering_node] Triggered with direction='%s'. Starting FORWARD stage.",
            direction
        )

    # ---------------------- main control loop ----------------------

    def control_loop(self, event):
        now = rospy.Time.now()

        if self.state == "IDLE":
            # Do nothing; we are not controlling the robot
            return

        # -------------------- FORWARD STAGE --------------------
        if self.state == "FORWARD":
            if self.stage_start_time is None:
                self.stage_start_time = now

            elapsed = (now - self.stage_start_time).to_sec()

            if elapsed < self.forward_duration:
                # Move forward
                vx = self.forward_speed if self.forward_distance >= 0.0 else -self.forward_speed
                self.publish_velocity(vx, 0.0)
            else:
                # Finished forward motion
                rospy.loginfo("[pre_entering_node] Finished FORWARD stage (%.2f s).", elapsed)
                self.stop_once()

                # Decide next stage
                if self.current_direction == "straight":
                    # No rotation needed → we are done
                    self._finish_success()
                else:
                    # Need rotation stage
                    self.state = "ROTATE"
                    self.stage_start_time = now
                    rospy.loginfo(
                        "[pre_entering_node] Starting ROTATE stage (%s 90 deg).",
                        self.current_direction.upper()
                    )

        # -------------------- ROTATE STAGE --------------------
        elif self.state == "ROTATE":
            if self.stage_start_time is None:
                self.stage_start_time = now

            elapsed = (now - self.stage_start_time).to_sec()

            if elapsed < self.rotation_duration:
                # Rotate according to direction
                if self.current_direction == "right":
                    wz = -abs(self.rotation_speed)  # right turn = negative yaw
                elif self.current_direction == "left":
                    wz =  abs(self.rotation_speed)  # left turn = positive yaw
                else:
                    # Should not happen, but fail-safe
                    wz = 0.0
                self.publish_velocity(0.0, wz)
            else:
                # Finished rotation
                rospy.loginfo(
                    "[pre_entering_node] Finished ROTATE stage (%s, %.2f s).",
                    self.current_direction.upper(),
                    elapsed,
                )
                self.stop_once()
                self._finish_success()

    # ---------------------- finish sequence ----------------------

    def _finish_success(self):
        """
        Called when both stages are done.
        Publishes 'success' and returns to IDLE.
        """
        msg = String()
        msg.data = "success"
        self.done_pub.publish(msg)
        rospy.loginfo("[pre_entering_node] Sequence finished. Published /pre_entering_finished = 'success'.")

        # Reset internal state
        self.state = "IDLE"
        self.current_direction = None
        self.stage_start_time = None


def main():
    rospy.init_node("pre_entering_node")
    node = PreEnteringNode()
    rospy.spin()


if __name__ == "__main__":
    main()
