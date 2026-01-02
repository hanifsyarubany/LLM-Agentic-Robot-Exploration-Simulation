#!/usr/bin/env python3
import rospy
import math
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32, String   # gating message type


class LocalMapWallAvoider:
    def __init__(self):
        # Topics
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.map_topic     = rospy.get_param("~map_topic", "/map/local_map/obstacle")

        # Behaviour params
        self.state_topic       = rospy.get_param("~state_topic", "/wall_avoider_state")
        self.forward_speed     = rospy.get_param("~forward_speed", 0.2)        # m/s
        self.rotation_speed    = rospy.get_param("~rotation_speed", 0.5)       # rad/s
        self.rotation_angle    = rospy.get_param("~rotation_angle", math.pi/2) # 90 deg
        self.turn_around_angle = rospy.get_param("~turn_around_angle", math.pi) # 180 deg
        self.safe_distance     = rospy.get_param("~safe_distance", 1.0)        # obstacle distance threshold
        self.min_turn_distance = rospy.get_param("~min_turn_distance", 0.3)    # not strictly used now, but kept
        self.lateral_width     = rospy.get_param("~lateral_width", 0.5)        # m
        self.obst_threshold    = rospy.get_param("~obstacle_threshold", 50)    # occupancy value
        self.forward_axis      = rospy.get_param("~forward_axis", "x")         # "x" or "y"
        self.max_scan_distance = rospy.get_param("~max_scan_distance", self.safe_distance)

        # Corridor classification thresholds (unused in new logic, but kept for completeness)
        self.side_blocked_threshold = rospy.get_param("~side_blocked_threshold", 0.3)
        self.side_open_threshold    = rospy.get_param("~side_open_threshold", 0.5)

        # Turn direction: always rotate in this direction when avoiding
        self.turn_direction = rospy.get_param("~turn_direction", "RIGHT")  # "RIGHT" or "LEFT"

        # Derived durations for 90° and 180°
        w = max(1e-6, abs(self.rotation_speed))
        self.first_turn_duration  = abs(self.rotation_angle)     / w  # 90 deg
        self.second_turn_duration = abs(self.turn_around_angle)  / w  # 180 deg

        # ROS I/O
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid,
                                        self.map_callback, queue_size=1)

        # NEW: state publisher
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1)

        # Gating subscriber
        self.gate_sub = rospy.Subscriber("/activate_wall_avoider",
                                         Int32,
                                         self.gate_callback,
                                         queue_size=1)

        # Map buffer
        self.map_msg = None
        self.map_lock = Lock()
        self.has_map = False

        # FSM states: "FORWARD", "FIRST_TURN", "SECOND_TURN"
        self.state = "FORWARD"
        self.rotation_start_time = None

        # Gate state (True = active, False = paused)
        self.active = False  # start active; set to False by publishing 0 on /activate_wall_avoider

        rospy.loginfo("[LocalMapWallAvoider] Started. map_topic=%s, cmd_vel_topic=%s, turn_direction=%s",
                      self.map_topic, self.cmd_vel_topic, self.turn_direction)

        # Main loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    # --------------------------- callbacks ---------------------------

    def map_callback(self, msg):
        with self.map_lock:
            self.map_msg = msg
            self.has_map = True

    def gate_callback(self, msg):
        """
        Gate control: 1 = active, 0 = inactive.
        When inactive, robot stops once and FSM is reset to FORWARD.
        """
        if msg.data == 1:
            if not self.active:
                rospy.loginfo("[LocalMapWallAvoider] Gate=1 -> ACTIVATED")
            self.active = True

        elif msg.data == 0:
            if self.active:
                rospy.loginfo("[LocalMapWallAvoider] Gate=0 -> DEACTIVATED, stopping and resetting FSM")
            self.active = False
            # Reset FSM so we don't resume in the middle of a turn
            self.state = "FORWARD"
            self.rotation_start_time = None
            # IMPORTANT: stop ONCE here
            self.stop()

        else:
            rospy.logwarn("[LocalMapWallAvoider] Unknown gate value %d (expected 0 or 1)", msg.data)

    # ------------------------- map utilities -------------------------

    def compute_distance_to_wall_ahead(self):
        """
        Scan a corridor ahead and return the minimum distance to an obstacle,
        up to max_scan_distance. Return None if no obstacle in that corridor.
        """
        with self.map_lock:
            if not self.has_map or self.map_msg is None:
                return None

            grid = self.map_msg
            res = grid.info.resolution
            width = grid.info.width
            height = grid.info.height
            origin = grid.info.origin
            data = grid.data

            try:
                cx = int((0.0 - origin.position.x) / res)
                cy = int((0.0 - origin.position.y) / res)
            except ZeroDivisionError:
                return None

            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                cx = width // 2
                cy = height // 2

            # --- use max_scan_distance instead of safe_distance for scanning ---
            max_ahead_cells = max(1, int(self.max_scan_distance / res))
            half_lat_cells  = max(1, int(self.lateral_width / res / 2.0))

            min_dist = None

            if self.forward_axis == "x":
                x_start = cx
                x_end   = min(width - 1, cx + max_ahead_cells)
                y_min   = max(0, cy - half_lat_cells)
                y_max   = min(height - 1, cy + half_lat_cells)

                for x in range(x_start, x_end + 1):
                    dx = (x - cx) * res
                    if dx < 0:
                        continue
                    for y in range(y_min, y_max + 1):
                        idx = x + y * width
                        occ = data[idx]
                        if occ >= self.obst_threshold:
                            if min_dist is None or dx < min_dist:
                                min_dist = dx

            else:
                y_start = cy
                y_end   = min(height - 1, cy + max_ahead_cells)
                x_min   = max(0, cx - half_lat_cells)
                x_max   = min(width - 1, cx + half_lat_cells)

                for y in range(y_start, y_end + 1):
                    dy = (y - cy) * res
                    if dy < 0:
                        continue
                    for x in range(x_min, x_max + 1):
                        idx = x + y * width
                        occ = data[idx]
                        if occ >= self.obst_threshold:
                            if min_dist is None or dy < min_dist:
                                min_dist = dy

            return min_dist


    # ------------------------- velocity helpers ----------------------

    def yaw_speed_for_direction(self, direction):
        return -abs(self.rotation_speed) if direction == "RIGHT" else abs(self.rotation_speed)

    def publish_velocity(self, vx, wz):
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    # ------------------------- state publisher -----------------------

    def publish_state(self):
        """
        Publish the current control state as a string:
          "FORWARD", "FIRST_TURN", "SECOND_TURN"
        (You can also infer active/inactive from /activate_wall_avoider.)
        """
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)
    
    # --------------------------- main loop ---------------------------

    def control_loop(self, event):
        # Always publish state (even when inactive), for monitoring
        dist_wall = self.compute_distance_to_wall_ahead()
        if dist_wall:
            rospy.loginfo("DISTANCE: %.5f",dist_wall)
        self.publish_state()
        # When inactive, do NOTHING (do not keep publishing zeros!)
        if not self.active:
            rospy.logdebug_throttle(
                2.0,
                "[LocalMapWallAvoider] Inactive (gate=0), not publishing cmd_vel"
            )
            return

        if not self.has_map:
            rospy.logdebug_throttle(2.0, "[LocalMapWallAvoider] Waiting for local map...")
            self.stop()
            return

        # === STATE MACHINE ===
        if self.state == "FORWARD":
            dist_wall = self.compute_distance_to_wall_ahead()

            if dist_wall is not None and dist_wall <= self.safe_distance:
                # Wall ahead -> start 90° turn
                self.state = "FIRST_TURN"
                self.rotation_start_time = rospy.Time.now()
                rospy.loginfo("[LocalMapWallAvoider] Wall at %.2f m, starting 90° turn (%s)",
                              dist_wall, self.turn_direction)
                self.publish_velocity(0.0, self.yaw_speed_for_direction(self.turn_direction))
            else:
                # No wall within safe distance -> keep going straight
                self.publish_velocity(self.forward_speed, 0.0)

        elif self.state == "FIRST_TURN":
            elapsed = (rospy.Time.now() - self.rotation_start_time).to_sec()

            if elapsed < self.first_turn_duration:
                # Continue 90° rotation
                self.publish_velocity(0.0, self.yaw_speed_for_direction(self.turn_direction))
            else:
                # Finished 90° -> stop and evaluate again
                self.stop()
                dist_after = self.compute_distance_to_wall_ahead()
                rospy.loginfo("[LocalMapWallAvoider] After 90° turn, dist_wall=%.3f",
                              dist_after if dist_after is not None else -1.0)

                if dist_after is not None:
                    # Still wall ahead -> turn around 180°
                    self.state = "SECOND_TURN"
                    self.rotation_start_time = rospy.Time.now()
                    rospy.loginfo("[LocalMapWallAvoider] Path still blocked, turning additional 180° (%s)",
                                  self.turn_direction)
                    self.publish_velocity(0.0, self.yaw_speed_for_direction(self.turn_direction))
                else:
                    # Path clear -> go forward
                    rospy.loginfo("[LocalMapWallAvoider] Path clear after 90°, going FORWARD")
                    self.state = "FORWARD"
                    self.rotation_start_time = None
                    self.publish_velocity(self.forward_speed, 0.0)

        elif self.state == "SECOND_TURN":
            elapsed = (rospy.Time.now() - self.rotation_start_time).to_sec()

            if elapsed < self.second_turn_duration:
                # Continue 180° rotation
                self.publish_velocity(0.0, self.yaw_speed_for_direction(self.turn_direction))
            else:
                # Done turning around -> go forward
                rospy.loginfo("[LocalMapWallAvoider] Finished 180° turn, going FORWARD")
                self.state = "FORWARD"
                self.rotation_start_time = None
                self.publish_velocity(self.forward_speed, 0.0)


if __name__ == "__main__":
    rospy.init_node("local_map_wall_avoider")
    try:
        LocalMapWallAvoider()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
