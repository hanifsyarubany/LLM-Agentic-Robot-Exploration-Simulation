#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class CostmapDisappearTurnaround:
    # States
    INACTIVE = 0
    FORWARD  = 1
    ROTATE   = 2
    STOPPED  = 3

    def __init__(self):
        # ----------------------------
        # Params
        # ----------------------------
        self.gate_topic    = rospy.get_param("~gate_topic", "/activate_object_pickup")
        self.costmap_topic = rospy.get_param("~costmap_topic", "/map/local_map/obstacle")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.rotate_degrees = float(rospy.get_param("~rotate_degrees", 180.0))  

        self.forward_speed = float(rospy.get_param("~forward_speed", 0.15))   # m/s
        self.rotate_speed  = float(rospy.get_param("~rotate_speed", 0.6))     # rad/s (positive = CCW)
        self.publish_hz    = float(rospy.get_param("~publish_hz", 10.0))

        # Debounce: require this many consecutive “all-zero” costmap frames
        self.zero_frames_required = int(rospy.get_param("~zero_frames_required", 3))

        # If your costmap might contain -1 (unknown) and you still want to treat it as "zero"
        self.treat_unknown_as_zero = bool(rospy.get_param("~treat_unknown_as_zero", False))

        # Optional: if > 0 and costmap messages stop arriving while in FORWARD, treat as disappeared
        # (default disabled)
        self.costmap_timeout = float(rospy.get_param("~costmap_timeout", 0.0))

        # ----------------------------
        # ROS I/O
        # ----------------------------
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.gate_sub = rospy.Subscriber(self.gate_topic, Int32, self._gate_cb, queue_size=1)
        self.costmap_sub = rospy.Subscriber(self.costmap_topic, OccupancyGrid, self._costmap_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1e-6, self.publish_hz)), self._tick)

        # ----------------------------
        # Internal state
        # ----------------------------
        self.active = False
        self.state = self.INACTIVE

        self.zero_count = 0
        self.disappeared_latched = False

        self.last_costmap_time = None

        self.rot_start_time = None
        self.rot_duration = None

        # publish-zero-once behavior
        self._stop_pending = False

        rospy.loginfo("[costmap_disappear_turnaround] Ready. Gate: %s | Costmap: %s | CmdVel: %s",
                      self.gate_topic, self.costmap_topic, self.cmd_vel_topic)

    # ----------------------------
    # Callbacks
    # ----------------------------
    def _gate_cb(self, msg: Int32):
        if msg.data == 1 and not self.active:
            self.active = True
            self._reset_for_new_run()
            self._set_state(self.FORWARD)
            rospy.loginfo("[costmap_disappear_turnaround] ACTIVATED -> FORWARD")
        elif msg.data == 0 and self.active:
            self.active = False
            self._set_state(self.INACTIVE)
            self._request_stop_once()
            rospy.loginfo("[costmap_disappear_turnaround] INACTIVATED -> INACTIVE (stop once)")

    def _costmap_cb(self, msg: OccupancyGrid):
        self.last_costmap_time = rospy.Time.now()

        data = msg.data
        if not data:
            # empty map: treat as disappeared-ish
            all_zero = True
        else:
            all_zero = True
            if self.treat_unknown_as_zero:
                # consider any value not in {0, -1} as non-zero
                for v in data:
                    if v != 0 and v != -1:
                        all_zero = False
                        break
            else:
                # strict: any value != 0 counts as non-zero
                for v in data:
                    if v != 0:
                        all_zero = False
                        break

        if all_zero:
            self.zero_count += 1
        else:
            self.zero_count = 0
            self.disappeared_latched = False  # allow future detection

        # Only trigger “sudden disappear” while moving forward and active
        if self.active and self.state == self.FORWARD:
            if (not self.disappeared_latched) and (self.zero_count >= self.zero_frames_required):
                self.disappeared_latched = True
                rospy.logwarn("[costmap_disappear_turnaround] Costmap all-zeros for %d frames -> ROTATE 180",
                              self.zero_frames_required)
                self._start_rotation_180()

    # ----------------------------
    # State machine helpers
    # ----------------------------
    def _reset_for_new_run(self):
        self.zero_count = 0
        self.disappeared_latched = False
        self.rot_start_time = None
        self.rot_duration = None
        self._stop_pending = False

    def _set_state(self, new_state: int):
        self.state = new_state

    def _request_stop_once(self):
        # publish Twist() exactly once (next tick) then do not publish further zeros
        self._stop_pending = True

    def _publish_stop_once_if_needed(self):
        if self._stop_pending:
            self.cmd_pub.publish(Twist())  # all zeros
            self._stop_pending = False

    def _start_rotation_180(self):
        w = abs(self.rotate_speed)
        if w < 1e-6:
            rospy.logerr("[costmap_disappear_turnaround] rotate_speed too small; cannot rotate. Stopping.")
            self._set_state(self.STOPPED)
            self._request_stop_once()
            return
        angle_rad = abs(self.rotate_degrees) * math.pi / 180.0
        self.rot_start_time = rospy.Time.now()
        self.rot_duration = rospy.Duration(angle_rad / w)
        self._set_state(self.ROTATE)
    # ----------------------------
    # Timer tick: publish only when needed
    # ----------------------------
    def _tick(self, _evt):
        # Always allow a single stop publication if pending
        self._publish_stop_once_if_needed()

        if not self.active:
            return

        # Optional timeout behavior (disabled by default)
        if self.state == self.FORWARD and self.costmap_timeout > 0.0:
            if self.last_costmap_time is None:
                pass
            else:
                age = (rospy.Time.now() - self.last_costmap_time).to_sec()
                if age > self.costmap_timeout:
                    rospy.logwarn("[costmap_disappear_turnaround] Costmap timeout (%.2fs) -> ROTATE 180", age)
                    self._start_rotation_180()

        if self.state == self.FORWARD:
            cmd = Twist()
            cmd.linear.x = self.forward_speed
            self.cmd_pub.publish(cmd)

        elif self.state == self.ROTATE:
            if self.rot_start_time is None or self.rot_duration is None:
                # safety fallback
                self._set_state(self.STOPPED)
                self._request_stop_once()
                return

            if (rospy.Time.now() - self.rot_start_time) < self.rot_duration:
                cmd = Twist()
                cmd.angular.z = self.rotate_speed
                self.cmd_pub.publish(cmd)
            else:
                rospy.loginfo("[costmap_disappear_turnaround] Rotation done -> STOPPED (stop once)")
                self._set_state(self.STOPPED)
                self._request_stop_once()

        elif self.state == self.STOPPED:
            # Do NOT keep publishing zeros here.
            pass


def main():
    rospy.init_node("costmap_disappear_turnaround", anonymous=False)
    CostmapDisappearTurnaround()
    rospy.spin()


if __name__ == "__main__":
    main()
