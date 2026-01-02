#!/usr/bin/env python3
import rospy
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


class CorridorAligner(object):
    def __init__(self):
        # --- params ---
        self.map_topic         = rospy.get_param("~map_topic", "/map/local_map/obstacle")
        self.cmd_vel_topic     = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.occupied_thresh   = rospy.get_param("~occupied_thresh", 50)
        self.front_min_x       = rospy.get_param("~front_min_x", 0.1)   # m
        self.front_max_x       = rospy.get_param("~front_max_x", 2.0)   # m
        self.max_side_y        = rospy.get_param("~max_side_y", 2.0)    # m

        self.near_band         = rospy.get_param("~near_band", 0.20)    # kept but unused
        self.min_points        = rospy.get_param("~min_points", 15)

        # desired orientation of the wall in the local map frame
        # e.g. 0.0  -> wall along +x direction
        #      1.57 -> wall along +y direction (π/2)
        self.target_wall_angle = rospy.get_param("~target_wall_angle", math.pi/2.0)

        self.k_p               = rospy.get_param("~k_p", 1.0)           # P gain for angular speed
        self.max_ang_vel       = rospy.get_param("~max_ang_vel", 0.1)   # rad/s
        self.angle_tolerance   = rospy.get_param("~angle_tolerance", 0.1)  # rad (~5.7 deg)

        # choose ONE reference wall side when node starts
        self.reference_side = None   # "left" or "right"
        self.side_sign = 0.0         # +1 for left (y>=0), -1 for right (y<0)

        # ---- NEW: smoothing for measured wall angle ----
        self.wall_angle_filtered = None
        self.smoothing_alpha = rospy.get_param("~smoothing_alpha", 0.2)  # 0..1
        # ------------------------------------------------

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)

        rospy.loginfo("corridor_aligner: listening to %s, publishing %s",
                      self.map_topic, self.cmd_vel_topic)

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def map_callback(self, msg: OccupancyGrid):
        # Convert occupied cells to (x,y)
        res = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        points = []

        data = msg.data  # 1D array, row-major
        for j in range(height):      # y index
            for i in range(width):   # x index
                occ = data[j * width + i]
                if occ < self.occupied_thresh:
                    continue

                # center of cell in world/local coordinates
                x = origin_x + (i + 0.5) * res
                y = origin_y + (j + 0.5) * res

                # keep only points in front of the robot and within corridor width
                if x < self.front_min_x or x > self.front_max_x:
                    continue
                if abs(y) > self.max_side_y:
                    continue

                points.append([x, y])

        if len(points) < self.min_points:
            # not enough info: stop rotation
            self.publish_zero()
            return

        pts = np.array(points)

        # --- choose reference wall side ONCE (left/right) ---
        if self.reference_side is None:
            dists_all = np.linalg.norm(pts, axis=1)
            idx_min = np.argmin(dists_all)
            y_closest = pts[idx_min, 1]

            self.side_sign = 1.0 if y_closest >= 0.0 else -1.0
            self.reference_side = "left" if self.side_sign > 0 else "right"
            rospy.loginfo("corridor_aligner: using %s wall as reference",
                          self.reference_side)

        # keep only points on that chosen side
        side_mask = pts[:, 1] * self.side_sign >= 0.0
        pts_side = pts[side_mask]

        if pts_side.shape[0] < self.min_points:
            self.publish_zero()
            return

        # use ALL points on that wall side
        wall_pts = pts_side

        # --- PCA to get wall direction vector ---
        mean = np.mean(wall_pts, axis=0)
        centered = wall_pts - mean
        cov = centered.T.dot(centered) / centered.shape[0]
        eigvals, eigvecs = np.linalg.eig(cov)

        # principal direction (largest eigenvalue)
        idx = np.argmax(eigvals)
        v = eigvecs[:, idx]       # [vx, vy]

        # fix eigenvector sign so it always points "forward" (x >= 0)
        if v[0] < 0.0:
            v = -v

        wall_angle = math.atan2(v[1], v[0])  # directed angle

        # ---- NEW: smooth the wall angle to avoid jitter ----
        if self.wall_angle_filtered is None:
            self.wall_angle_filtered = wall_angle
        else:
            a = self.smoothing_alpha
            self.wall_angle_filtered = (1.0 - a) * self.wall_angle_filtered + a * wall_angle

        wall_angle_used = self.wall_angle_filtered
        # ----------------------------------------------------

        # wrapped error between desired and measured direction
        error = self._wrap_angle(self.target_wall_angle - wall_angle_used)

        twist = Twist()
        rospy.loginfo("wall_angle=%.3f, filt=%.3f, target=%.3f, err=%.3f",
                      wall_angle, wall_angle_used, self.target_wall_angle, error)

        if abs(error) < self.angle_tolerance:
            # close enough, stop
            self.publish_zero()
            return
        else:
            # ********** MAIN FIX: sign of control **********
            # stable choice: ang_z = -k_p * error
            ang_z = - self.k_p * error
            # ************************************************
            ang_z = max(-self.max_ang_vel, min(self.max_ang_vel, ang_z))
            twist.angular.z = ang_z
            twist.linear.x = 0.0   # rotate in place
            self.cmd_pub.publish(twist)

    def publish_zero(self):
        twist = Twist()
        self.cmd_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node("corridor_aligner")
    node = CorridorAligner()
    rospy.spin()
