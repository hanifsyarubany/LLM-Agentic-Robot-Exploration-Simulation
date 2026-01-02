#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Int32


class ObjectApproachGraspStoreNode:
    """
    Subscribes:
      - /yolo_rs/depth_xyz (geometry_msgs/PointStamped)
      - /grasping_success_mark (std_msgs/Int32) : success when data==1
      - /storing_success_mark  (std_msgs/Int32) : success when data==1

    Publishes:
      - /cmd_vel (geometry_msgs/Twist) : linear.x and linear.y only, angular.z = 0
      - /activate_object_grasping (std_msgs/Int32) : publish 1 ONCE to start grasp
      - /activate_object_storing  (std_msgs/Int32) : publish 1 ONCE to start store
    """

    APPROACH = 0
    GRASP_CMD = 1
    WAIT_GRASP = 2
    WAIT_AFTER_GRASP = 3
    STORE_CMD = 4
    WAIT_STORE = 5
    DONE = 6

    def __init__(self):
        rospy.init_node("object_approach_grasp_store_node")

        # --- Topics (approach)
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo_rs/depth_xyz")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # --- Topics (grasp)
        self.grasp_cmd_topic = rospy.get_param("~grasp_cmd_topic", "/activate_object_grasping")
        self.grasp_success_topic = rospy.get_param("~grasp_success_topic", "/grasping_success_mark")

        # --- Topics (store)
        self.store_cmd_topic = rospy.get_param("~store_cmd_topic", "/activate_object_storing")
        self.store_success_topic = rospy.get_param("~store_success_topic", "/storing_success_mark")

        # --- Target and tolerances
        self.target_depth = rospy.get_param("~target_depth", 0.24)   # meters
        self.x_tol = rospy.get_param("~x_tolerance", 0.005)          # meters
        self.z_tol = rospy.get_param("~z_tolerance", 0.01)           # meters

        # --- Controller gains
        self.kp_x = rospy.get_param("~kp_x", 1.5)
        self.kp_z = rospy.get_param("~kp_z", 1.2)

        # --- Velocity limits
        self.max_vx = rospy.get_param("~max_vx", 0.20)  # m/s
        self.max_vy = rospy.get_param("~max_vy", 0.20)  # m/s

        # --- Minimum speeds to overcome deadband/static friction
        self.min_vx = rospy.get_param("~min_vx", 0.03)  # m/s
        self.min_vy = rospy.get_param("~min_vy", 0.03)  # m/s

        # --- Filtering
        self.use_ema = rospy.get_param("~use_ema", True)
        self.ema_alpha = rospy.get_param("~ema_alpha", 0.3)  # 0..1

        # --- Robustness (approach)
        self.control_rate = rospy.get_param("~rate_hz", 20.0)
        self.msg_timeout = rospy.get_param("~msg_timeout", 0.5)  # seconds
        self.stable_cycles_required = rospy.get_param("~stable_cycles_required", 5)

        # --- Waiting timeouts
        self.grasp_wait_timeout = rospy.get_param("~grasp_wait_timeout", 8.0)  # seconds
        self.store_wait_timeout = rospy.get_param("~store_wait_timeout", 8.0)  # seconds

        # --- Mandatory delay between GRASP success and STORE command
        self.post_grasp_delay = rospy.get_param("~post_grasp_delay", 1.0)  # seconds

        # --- Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.grasp_pub = rospy.Publisher(self.grasp_cmd_topic, Int32, queue_size=10)
        self.store_pub = rospy.Publisher(self.store_cmd_topic, Int32, queue_size=10)

        # --- Subscribers
        self.sub_yolo = rospy.Subscriber(self.yolo_topic, PointStamped, self.cb_point, queue_size=1)
        self.sub_grasp_ok = rospy.Subscriber(self.grasp_success_topic, Int32, self.cb_grasp_success, queue_size=10)
        self.sub_store_ok = rospy.Subscriber(self.store_success_topic, Int32, self.cb_store_success, queue_size=10)

        # --- Latest target data
        self.last_stamp = None
        self.fx = None
        self.fz = None

        # --- State machine
        self.state = self.APPROACH
        self.stable_count = 0

        self.grasp_success = False
        self.store_success = False

        self.state_enter_time = rospy.Time.now()

        rospy.loginfo("[ApproachGraspStore] Node started.")
        rospy.loginfo(f"[ApproachGraspStore] post_grasp_delay={self.post_grasp_delay:.2f}s")

    # ---------------- Utilities ----------------
    @staticmethod
    def clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def publish_stop(self):
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)

    def transition(self, new_state):
        self.state = new_state
        self.state_enter_time = rospy.Time.now()

    def deadband_compensated(self, kp, err, tol, vmin, vmax):
        if abs(err) <= tol:
            return 0.0
        v = kp * err
        if abs(v) < vmin:
            v = vmin * (1.0 if err > 0 else -1.0)
        return self.clamp(v, -vmax, vmax)

    # ---------------- Callbacks ----------------
    def cb_point(self, msg: PointStamped):
        self.last_stamp = rospy.Time.now()
        x = msg.point.x
        z = msg.point.z

        if not self.use_ema:
            self.fx, self.fz = x, z
            return

        if self.fx is None or self.fz is None:
            self.fx, self.fz = x, z
        else:
            a = self.ema_alpha
            self.fx = a * x + (1.0 - a) * self.fx
            self.fz = a * z + (1.0 - a) * self.fz

    def cb_grasp_success(self, msg: Int32):
        if msg.data == 1:
            self.grasp_success = True

    def cb_store_success(self, msg: Int32):
        if msg.data == 1:
            self.store_success = True

    # ---------------- State Steps ----------------
    def step_approach(self):
        if self.last_stamp is None or self.fx is None or self.fz is None:
            self.stable_count = 0
            self.publish_stop()
            rospy.logwarn_throttle(1.0, "[Approach] No target yet. Stopping.")
            return

        age = (rospy.Time.now() - self.last_stamp).to_sec()
        if age > self.msg_timeout:
            self.stable_count = 0
            self.publish_stop()
            rospy.logwarn_throttle(1.0, f"[Approach] Target stale (age={age:.2f}s). Stopping.")
            return

        x_meas = self.fx
        z_meas = self.fz

        ex = 0.0 - x_meas
        ez = self.target_depth - z_meas

        in_x = abs(ex) <= self.x_tol
        in_z = abs(ez) <= self.z_tol

        self.stable_count = self.stable_count + 1 if (in_x and in_z) else 0

        # Keep your sign choice for vx (you used -ez)
        vx = self.deadband_compensated(self.kp_z, -ez, self.z_tol, self.min_vx, self.max_vx)
        vy = self.deadband_compensated(self.kp_x,  ex, self.x_tol, self.min_vy, self.max_vy)

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        rospy.loginfo_throttle(
            0.25,
            f"[Approach] x={x_meas:.3f}, z={z_meas:.3f} | ex={ex:.3f}, ez={ez:.3f} | "
            f"vx={vx:.3f}, vy={vy:.3f} | stable={self.stable_count}/{self.stable_cycles_required}"
        )

        if self.stable_count >= self.stable_cycles_required:
            self.publish_stop()
            self.grasp_success = False
            self.store_success = False
            self.transition(self.GRASP_CMD)

    def step_grasp_cmd(self):
        self.publish_stop()
        self.grasp_pub.publish(Int32(data=1))
        rospy.loginfo("[Grasp] Published /activate_object_grasping: 1 (one-shot)")
        self.transition(self.WAIT_GRASP)

    def step_wait_grasp(self):
        self.publish_stop()

        if self.grasp_success:
            rospy.loginfo("[Grasp] Success=1 received. Waiting 1s before STORE...")
            self.transition(self.WAIT_AFTER_GRASP)
            return

        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed > self.grasp_wait_timeout:
            rospy.logerr(f"[Grasp] Timeout waiting for success (>{self.grasp_wait_timeout:.1f}s).")
            self.transition(self.DONE)
            return

        rospy.loginfo_throttle(0.5, f"[Grasp] Waiting success... elapsed={elapsed:.1f}s")

    def step_wait_after_grasp(self):
        self.publish_stop()
        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()

        if elapsed >= self.post_grasp_delay:
            self.transition(self.STORE_CMD)
            return

        rospy.loginfo_throttle(0.5, f"[Between] Post-grasp delay... {elapsed:.2f}/{self.post_grasp_delay:.2f}s")

    def step_store_cmd(self):
        self.publish_stop()
        self.store_pub.publish(Int32(data=1))
        rospy.loginfo("[Store] Published /activate_object_storing: 1 (one-shot)")
        self.transition(self.WAIT_STORE)

    def step_wait_store(self):
        self.publish_stop()

        if self.store_success:
            rospy.loginfo("[Store] Success=1 received. DONE.")
            self.transition(self.DONE)
            return

        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed > self.store_wait_timeout:
            rospy.logerr(f"[Store] Timeout waiting for success (>{self.store_wait_timeout:.1f}s).")
            self.transition(self.DONE)
            return

        rospy.loginfo_throttle(0.5, f"[Store] Waiting success... elapsed={elapsed:.1f}s")

    # ---------------- Main Loop ----------------
    def spin(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self.state == self.APPROACH:
                self.step_approach()
            elif self.state == self.GRASP_CMD:
                self.step_grasp_cmd()
            elif self.state == self.WAIT_GRASP:
                self.step_wait_grasp()
            elif self.state == self.WAIT_AFTER_GRASP:
                self.step_wait_after_grasp()
            elif self.state == self.STORE_CMD:
                self.step_store_cmd()
            elif self.state == self.WAIT_STORE:
                self.step_wait_store()
            else:
                self.publish_stop()

            rate.sleep()


if __name__ == "__main__":
    try:
        node = ObjectApproachGraspStoreNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass

