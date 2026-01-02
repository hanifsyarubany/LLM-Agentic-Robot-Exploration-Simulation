#!/usr/bin/env python3
import rospy
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

class CorridorCentering:
    def __init__(self):
        # Params
        self.map_topic        = rospy.get_param("~map_topic", "/map/local_map/obstacle")
        self.cmd_vel_topic    = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.forward_axis     = rospy.get_param("~forward_axis", "x")   # assume "x"
        self.obst_threshold   = rospy.get_param("~obstacle_threshold", 50)

        # Where ahead we look for walls (meters)
        self.ahead_min_dist   = rospy.get_param("~ahead_min_dist", 0.5)
        self.ahead_max_dist   = rospy.get_param("~ahead_max_dist", 2.0)

        # How far sideways we search for walls (meters)
        self.max_lat_dist     = rospy.get_param("~max_lateral_dist", 2.0)

        # Controller parameters
        self.center_tolerance = rospy.get_param("~center_tolerance", 0.1)   # m, |left-right| < tol -> stop
        self.k_lat            = rospy.get_param("~k_lateral", 0.5)          # proportional gain
        self.max_lateral_vel  = rospy.get_param("~max_lateral_vel", 0.3)    # m/s
        self.forward_vel      = rospy.get_param("~forward_vel", 0.0)        # could be small >0 if you want

        # NEW: hysteresis so we stop once centered and don't jitter
        self.recenter_threshold = rospy.get_param(
            "~recenter_threshold",
            2.0 * self.center_tolerance  # default: twice the enter tolerance
        )
        self.centered = False

        # ROS I/O
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid,
                                        self.map_callback, queue_size=1)

        # Map buffer
        self.map_msg = None
        self.map_lock = Lock()
        self.has_map = False

        rospy.loginfo("[CorridorCentering] Listening to %s, publishing to %s",
                      self.map_topic, self.cmd_vel_topic)

        # Control loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)  # 10 Hz

    # ---------------------- callbacks ----------------------

    def map_callback(self, msg):
        with self.map_lock:
            self.map_msg = msg
            self.has_map = True

    # -------------------- core logic -----------------------

    def compute_left_right_distances(self):
        """
        From the local occupancy grid, estimate the minimal lateral distance to
        walls on the left and right, within a forward slice [ahead_min_dist, ahead_max_dist].

        Returns (min_left, min_right) in meters, or (None, None) if not available.
        """
        with self.map_lock:
            if not self.has_map or self.map_msg is None:
                return (None, None)

            grid = self.map_msg
            res = grid.info.resolution
            width = grid.info.width
            height = grid.info.height
            origin = grid.info.origin
            data = grid.data

            # index of camera (0,0) in grid coords
            try:
                cx = int((0.0 - origin.position.x) / res)
                cy = int((0.0 - origin.position.y) / res)
            except ZeroDivisionError:
                return (None, None)

            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                # fallback: assume camera somewhere near center
                cx = width // 2
                cy = height // 2

            ahead_min_cells = max(0, int(self.ahead_min_dist / res))
            ahead_max_cells = max(ahead_min_cells + 1, int(self.ahead_max_dist / res))
            max_lat_cells   = max(1, int(self.max_lat_dist / res))

            min_left  = None  # distance in meters
            min_right = None

            if self.forward_axis == "x":
                x_start = max(0, cx + ahead_min_cells)
                x_end   = min(width - 1, cx + ahead_max_cells)

                # Left side: y > cy
                y_left_min = cy + 1
                y_left_max = min(height - 1, cy + max_lat_cells)

                # Right side: y < cy
                y_right_min = max(0, cy - max_lat_cells)
                y_right_max = cy - 1

                # Scan forward slice
                for x in range(x_start, x_end + 1):
                    for y in range(y_left_min, y_left_max + 1):
                        idx = x + y * width
                        occ = data[idx]
                        if occ >= self.obst_threshold:
                            # lateral distance to left is (y - cy)*res
                            d_lat = (y - cy) * res
                            if min_left is None or d_lat < min_left:
                                min_left = d_lat

                    for y in range(y_right_min, y_right_max + 1):
                        idx = x + y * width
                        occ = data[idx]
                        if occ >= self.obst_threshold:
                            # lateral distance to right is (cy - y)*res
                            d_lat = (cy - y) * res
                            if min_right is None or d_lat < min_right:
                                min_right = d_lat

            else:
                # If you use forward_axis = "y", you'd mirror indices here.
                # For now we assume forward_axis == "x" (camera looking along +x).
                rospy.logwarn_throttle(5.0, "[CorridorCentering] forward_axis != 'x' not implemented.")
                return (None, None)

            return (min_left, min_right)

    def publish_cmd(self, vx, vy):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def control_loop(self, event):
        if not self.has_map:
            rospy.logdebug_throttle(2.0, "[CorridorCentering] Waiting for local map...")
            self.publish_cmd(0.0, 0.0)
            return

        min_left, min_right = self.compute_left_right_distances()

        if min_left is None or min_right is None:
            # Not enough information: don't try to center
            rospy.logdebug_throttle(2.0, "[CorridorCentering] Cannot estimate left/right walls.")
            self.publish_cmd(0.0, 0.0)
            return

        # Difference: positive -> more clearance on left; negative -> more on right
        diff = min_left - min_right

        rospy.loginfo_throttle(0.5,
            "[CorridorCentering] min_left=%.3f m, min_right=%.3f m, diff=%.3f m",
            min_left, min_right, diff)

        # --- If we are already latched as centered ---
        if self.centered:
            # If still close enough to center, hold position (no sideways motion)
            if abs(diff) <= self.recenter_threshold:
                self.publish_cmd(self.forward_vel, 0.0)
                return
            else:
                # Drifted too far away -> re-enable control
                rospy.loginfo("[CorridorCentering] Drifted from center (|diff|=%.3f > %.3f), re-centering",
                              abs(diff), self.recenter_threshold)
                self.centered = False  # fall through to active control

        # Already centered enough
        if abs(diff) < self.center_tolerance:
            self.publish_cmd(self.forward_vel, 0.0)
            return

        # Proportional controller on lateral velocity.
        # We want: if left < right (too close to left wall), move right (vy < 0).
        # diff = min_left - min_right
        # -> if diff < 0 (min_left < min_right), vy should be negative.
        vy_cmd = self.k_lat * diff

        # Saturate
        if vy_cmd > self.max_lateral_vel:
            vy_cmd = self.max_lateral_vel
        elif vy_cmd < -self.max_lateral_vel:
            vy_cmd = -self.max_lateral_vel

        self.publish_cmd(self.forward_vel, vy_cmd)


if __name__ == "__main__":
    rospy.init_node("corridor_centering")
    try:
        CorridorCentering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
