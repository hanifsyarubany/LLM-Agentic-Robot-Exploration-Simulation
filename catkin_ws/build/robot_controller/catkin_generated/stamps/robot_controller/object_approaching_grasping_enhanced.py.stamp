#!/usr/bin/env python3
import rospy
import json
import os
import rospkg
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Int32


class MultiObjectGraspStoreWithTrackingID:
    WAIT_ACTIVATION = 0
    SELECT_INSTANCE = 1
    APPROACH = 2
    GRASP_CMD = 3
    WAIT_GRASP = 4
    WAIT_AFTER_GRASP = 5
    STORE_CMD = 6
    WAIT_STORE = 7
    VERIFY_REMOVED = 8
    SEARCH_BACKUP = 9
    FINISH = 10
    VERIFY_STALE_BACKUP = 11  # NEW

    def __init__(self):
        rospy.init_node("multi_object_grasp_store_trackingid_node")

        # ---------------- Topics ----------------
        self.det_topic = rospy.get_param("~det_topic", "/yolo_rs/depth_xyz")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.grasp_cmd_topic = rospy.get_param("~grasp_cmd_topic", "/activate_object_grasping")
        self.grasp_ok_topic = rospy.get_param("~grasp_ok_topic", "/grasping_success_mark")

        self.store_cmd_topic = rospy.get_param("~store_cmd_topic", "/activate_object_storing")
        self.store_ok_topic = rospy.get_param("~store_ok_topic", "/storing_success_mark")

        self.activate_topic = rospy.get_param("~activate_topic", "/activate_object_approaching_grasping_node")
        self.success_topic = rospy.get_param("~success_topic", "/multi_object_task_success")

        # ---------------- Approach Control ----------------
        self.target_depth = rospy.get_param("~target_depth", 0.24)    # meters
        self.x_tol = rospy.get_param("~x_tolerance", 0.005)           # meters
        self.z_tol = rospy.get_param("~z_tolerance", 0.01)            # meters

        self.kp_x = rospy.get_param("~kp_x", 1.5)
        self.kp_z = rospy.get_param("~kp_z", 1.2)

        self.max_vx = rospy.get_param("~max_vx", 0.20)
        self.max_vy = rospy.get_param("~max_vy", 0.20)
        self.min_vx = rospy.get_param("~min_vx", 0.03)
        self.min_vy = rospy.get_param("~min_vy", 0.03)

        # Sign toggles
        self.vx_uses_negative_ez = rospy.get_param("~vx_uses_negative_ez", True)
        self.invert_strafe = rospy.get_param("~invert_strafe", False)

        # ---------------- Filtering / freshness ----------------
        self.use_ema = rospy.get_param("~use_ema", True)
        self.ema_alpha = rospy.get_param("~ema_alpha", 0.3)
        self.detection_timeout = rospy.get_param("~detection_timeout", 0.5)  # seconds

        # ---------------- Timings ----------------
        self.rate_hz = rospy.get_param("~rate_hz", 20.0)
        self.stable_cycles_required = rospy.get_param("~stable_cycles_required", 5)

        self.grasp_wait_timeout = rospy.get_param("~grasp_wait_timeout", 8.0)
        self.store_wait_timeout = rospy.get_param("~store_wait_timeout", 8.0)
        self.post_grasp_delay = rospy.get_param("~post_grasp_delay", 1.0)

        # ---------------- Verify removal ----------------
        self.verify_absent_cycles = rospy.get_param("~verify_absent_cycles", 8)
        self.verify_timeout = rospy.get_param("~verify_timeout", 2.0)

        self.removal_region_x = rospy.get_param("~removal_region_x", 0.06)  # meters
        self.removal_region_z = rospy.get_param("~removal_region_z", 0.06)  # meters

        # ---------------- Search backup ----------------
        self.search_backup_dist = rospy.get_param("~search_backup_dist", 1.0)   # meters
        self.search_backup_speed = rospy.get_param("~search_backup_speed", 0.10)  # m/s magnitude
        self.search_backup_time = abs(self.search_backup_dist / max(self.search_backup_speed, 1e-6))
        self.finish_if_missing_after_backup = rospy.get_param("~finish_if_missing_after_backup", True)

        # ---------------- NEW: stale-detection verify backup ----------------
        self.verify_stale_backup_time = rospy.get_param("~verify_stale_backup_time", 5.0)
        self.verify_stale_backup_speed = rospy.get_param("~verify_stale_backup_speed", self.search_backup_speed)

        # ---------------- Publishers ----------------
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.grasp_pub = rospy.Publisher(self.grasp_cmd_topic, Int32, queue_size=10)
        self.store_pub = rospy.Publisher(self.store_cmd_topic, Int32, queue_size=10)
        self.success_pub = rospy.Publisher(self.success_topic, Int32, queue_size=10, latch=True)

        # ---------------- Detection DB ----------------
        self.instances = {}
        self.last_det_stamp = None

        # ---------------- Subscribers ----------------
        self.det_sub = rospy.Subscriber(self.det_topic, PointStamped, self.cb_det, queue_size=200)
        self.grasp_ok_sub = rospy.Subscriber(self.grasp_ok_topic, Int32, self.cb_grasp_ok, queue_size=10)
        self.store_ok_sub = rospy.Subscriber(self.store_ok_topic, Int32, self.cb_store_ok, queue_size=10)
        self.activate_sub = rospy.Subscriber(self.activate_topic, Int32, self.cb_activate, queue_size=10)

        # ---------------- Targets from file ----------------
        self.target_json_path = self._resolve_target_json_path()
        self.targets, self.target_order = self._load_targets_from_file(self.target_json_path)

        # ---------------- Activation gate ----------------
        self.active = False

        # ---------------- Current lock ----------------
        self.lock_label = None
        self.lock_tid = None
        self.lock_last_x = None
        self.lock_last_z = None

        # ---------------- Grasp/store flags ----------------
        self.grasp_success = False
        self.store_success = False

        # ---------------- Verify removal bookkeeping ----------------
        self.verify_absent_count = 0
        self.last_grasp_label = None
        self.last_grasp_tid = None
        self.last_grasp_x = None
        self.last_grasp_z = None

        # --- Vision snapshot BEFORE grasp ---
        self.pre_label = None
        self.pre_label_count = None
        self.pre_label_ids = set()

        # ---------------- State machine ----------------
        self.state = self.WAIT_ACTIVATION
        self.state_enter_time = rospy.Time.now()
        self.stable_count = 0
        self.finished_published = False

        # cmd_vel stop one-shot
        self._last_cmd_was_zero = True

        rospy.loginfo("[MultiObjectTrackingID] Node started.")
        rospy.loginfo(f"[MultiObjectTrackingID] Targets file: {self.target_json_path}")
        rospy.loginfo(f"[MultiObjectTrackingID] Loaded targets: {self.targets}")
        rospy.loginfo(f"[MultiObjectTrackingID] Waiting activation on: {self.activate_topic}")

    # ---------------- File I/O ----------------
    def _resolve_target_json_path(self):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("semantic_mapping")
        return os.path.join(pkg_path, "maps", "target_object_grasping.json")

    def _load_targets_from_file(self, path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("JSON must be a dict like {\"hamburger\":3, \"cup\":2}")

            targets = {}
            order = []
            for k, v in data.items():
                k = str(k)
                try:
                    iv = int(v)
                except Exception:
                    iv = 0
                if iv < 0:
                    iv = 0
                targets[k] = iv
                order.append(k)

            return targets, order
        except Exception as e:
            rospy.logerr(f"[Targets] Failed to read '{path}': {e}. Using empty targets.")
            return {}, []

    def _write_targets_to_file(self, path, targets_dict):
        tmp_path = path + ".tmp"
        with open(tmp_path, "w") as f:
            json.dump(targets_dict, f, indent=2, sort_keys=False)
        os.replace(tmp_path, path)

    # ---------------- Utilities ----------------
    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def publish_stop(self, force=False):
        if (not force) and self._last_cmd_was_zero:
            return
        t = Twist()
        t.linear.x = 0.0
        t.linear.y = 0.0
        t.angular.z = 0.0
        self.cmd_pub.publish(t)
        self._last_cmd_was_zero = True

    def publish_cmd(self, vx, vy):
        if abs(vx) < 1e-6 and abs(vy) < 1e-6:
            self.publish_stop(force=False)
            return
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self._last_cmd_was_zero = False

    def transition(self, s):
        self.state = s
        self.state_enter_time = rospy.Time.now()

    def deadband_compensated(self, kp, err, tol, vmin, vmax):
        if abs(err) <= tol:
            return 0.0
        v = kp * err
        if abs(v) < vmin:
            v = vmin * (1.0 if err > 0 else -1.0)
        return self.clamp(v, -vmax, vmax)

    def parse_frame_id(self, frame_id):
        s = (frame_id or "").strip()
        if ":" not in s:
            return None, None
        label, tid_str = s.rsplit(":", 1)
        label = label.strip()
        try:
            tid = int(tid_str.strip())
        except Exception:
            return None, None
        if label == "":
            return None, None
        return label, tid

    def targets_done(self):
        return bool(self.targets) and all(v <= 0 for v in self.targets.values())

    def any_remaining(self):
        return bool(self.targets) and any(v > 0 for v in self.targets.values())

    def is_fresh_instance(self, label, tid):
        k = (label, tid)
        if k not in self.instances:
            return False
        age = (rospy.Time.now() - self.instances[k]["stamp"]).to_sec()
        return age <= self.detection_timeout

    def visible_instances_for_label(self, label):
        out = []
        now = rospy.Time.now()
        for (lb, tid), d in self.instances.items():
            if lb != label:
                continue
            if (now - d["stamp"]).to_sec() <= self.detection_timeout:
                out.append((tid, d["x"], d["z"]))
        return out

    def any_visible_matching_targets(self):
        for label, cnt in self.targets.items():
            if cnt <= 0:
                continue
            if len(self.visible_instances_for_label(label)) > 0:
                return True
        return False

    def reset_run_state(self):
        self.publish_stop(force=True)
        self.lock_label = None
        self.lock_tid = None
        self.lock_last_x = None
        self.lock_last_z = None

        self.grasp_success = False
        self.store_success = False

        self.verify_absent_count = 0
        self.last_grasp_label = None
        self.last_grasp_tid = None
        self.last_grasp_x = None
        self.last_grasp_z = None

        self.pre_label = None
        self.pre_label_count = None
        self.pre_label_ids = set()

        self.stable_count = 0
        self.finished_published = False
        self.transition(self.WAIT_ACTIVATION)

    # ---------------- Callbacks ----------------
    def cb_activate(self, msg: Int32):
        if msg.data == 1:
            rospy.loginfo("[Activate] 1 -> START/RESTART mission.")
            self.active = True
            self.targets, self.target_order = self._load_targets_from_file(self.target_json_path)

            self.publish_stop(force=True)
            self.lock_label = None
            self.lock_tid = None
            self.grasp_success = False
            self.store_success = False
            self.verify_absent_count = 0
            self.stable_count = 0
            self.finished_published = False

            self.pre_label = None
            self.pre_label_count = None
            self.pre_label_ids = set()

            if self.any_remaining() and not self.targets_done():
                self.transition(self.SELECT_INSTANCE)
            else:
                self.transition(self.FINISH)
        else:
            rospy.logwarn("[Activate] 0 -> STOP immediately + reset.")
            self.active = False
            self.reset_run_state()

    def cb_det(self, msg: PointStamped):
        now = rospy.Time.now()
        self.last_det_stamp = now

        label, tid = self.parse_frame_id(msg.header.frame_id)
        if label is None:
            return

        x = float(msg.point.x)
        z = float(msg.point.z)
        k = (label, tid)

        if k in self.instances and self.use_ema:
            a = self.ema_alpha
            fx = a * x + (1.0 - a) * self.instances[k]["x"]
            fz = a * z + (1.0 - a) * self.instances[k]["z"]
        else:
            fx, fz = x, z

        self.instances[k] = {"x": fx, "z": fz, "rawx": x, "rawz": z, "stamp": now}

    def cb_grasp_ok(self, msg: Int32):
        if msg.data == 1:
            self.grasp_success = True

    def cb_store_ok(self, msg: Int32):
        if msg.data == 1:
            self.store_success = True

    # ---------------- State Steps ----------------
    def step_wait_activation(self):
        self.publish_stop(force=False)

    def step_select_instance(self):
        self.publish_stop(force=False)

        if self.targets_done():
            self.transition(self.FINISH)
            return

        chosen_label = None
        chosen_tid = None
        chosen_x = None
        chosen_z = None

        for label in self.target_order:
            if self.targets.get(label, 0) <= 0:
                continue

            candidates = self.visible_instances_for_label(label)
            if not candidates:
                continue

            candidates.sort(key=lambda t: t[2])
            tid, x, z = candidates[0]
            chosen_label, chosen_tid, chosen_x, chosen_z = label, tid, x, z
            break

        if chosen_label is None:
            self.transition(self.SEARCH_BACKUP)
            return

        self.lock_label = chosen_label
        self.lock_tid = chosen_tid
        self.lock_last_x = chosen_x
        self.lock_last_z = chosen_z
        self.stable_count = 0

        rospy.loginfo(f"[Select] Locked: {self.lock_label}:{self.lock_tid} (remaining={self.targets.get(self.lock_label,0)})")
        self.transition(self.APPROACH)

    def step_approach(self):
        if self.lock_label is None or self.lock_tid is None:
            self.transition(self.SELECT_INSTANCE)
            return

        k = (self.lock_label, self.lock_tid)
        if k not in self.instances:
            self.publish_stop(force=False)
            rospy.logwarn_throttle(1.0, f"[Approach] Missing {self.lock_label}:{self.lock_tid}. Reselect.")
            self.transition(self.SELECT_INSTANCE)
            return

        if not self.is_fresh_instance(self.lock_label, self.lock_tid):
            self.publish_stop(force=False)
            rospy.logwarn_throttle(1.0, f"[Approach] Stale {self.lock_label}:{self.lock_tid}. Reselect.")
            self.transition(self.SELECT_INSTANCE)
            return

        x_meas = self.instances[k]["x"]
        z_meas = self.instances[k]["z"]

        self.lock_last_x = x_meas
        self.lock_last_z = z_meas

        ex = 0.0 - x_meas
        ez = self.target_depth - z_meas

        in_x = abs(ex) <= self.x_tol
        in_z = abs(ez) <= self.z_tol
        self.stable_count = self.stable_count + 1 if (in_x and in_z) else 0

        ez_used = -ez if self.vx_uses_negative_ez else ez
        vx = self.deadband_compensated(self.kp_z, ez_used, self.z_tol, self.min_vx, self.max_vx)

        ex_used = -ex if self.invert_strafe else ex
        vy = self.deadband_compensated(self.kp_x, ex_used, self.x_tol, self.min_vy, self.max_vy)

        self.publish_cmd(vx, vy)

        rospy.loginfo_throttle(
            0.25,
            f"[Approach {self.lock_label}:{self.lock_tid}] x={x_meas:.3f} z={z_meas:.3f} "
            f"ex={ex:.3f} ez={ez:.3f} vx={vx:.3f} vy={vy:.3f} stable={self.stable_count}/{self.stable_cycles_required}"
        )

        if self.stable_count >= self.stable_cycles_required:
            self.publish_stop(force=False)

            self.last_grasp_label = self.lock_label
            self.last_grasp_tid = self.lock_tid
            self.last_grasp_x = x_meas
            self.last_grasp_z = z_meas
            self.verify_absent_count = 0

            pre = self.visible_instances_for_label(self.lock_label)
            self.pre_label = self.lock_label
            self.pre_label_count = len(pre)
            self.pre_label_ids = set(tid for (tid, _, _) in pre)
            rospy.loginfo(f"[PreCount {self.pre_label}] count={self.pre_label_count} ids={sorted(list(self.pre_label_ids))}")

            self.grasp_success = False
            self.store_success = False
            self.transition(self.GRASP_CMD)

    def step_grasp_cmd(self):
        self.publish_stop(force=False)
        self.grasp_pub.publish(Int32(data=1))
        rospy.loginfo(f"[Grasp {self.lock_label}:{self.lock_tid}] Published grasp cmd=1 (one-shot)")
        self.transition(self.WAIT_GRASP)

    def step_wait_grasp(self):
        self.publish_stop(force=False)

        if self.grasp_success:
            rospy.loginfo(f"[Grasp] Success=1. Wait {self.post_grasp_delay:.1f}s then STORE.")
            self.transition(self.WAIT_AFTER_GRASP)
            return

        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed > self.grasp_wait_timeout:
            rospy.logerr(f"[Grasp] Timeout >{self.grasp_wait_timeout:.1f}s. Reselect.")
            self.transition(self.SELECT_INSTANCE)

    def step_wait_after_grasp(self):
        self.publish_stop(force=False)
        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed >= self.post_grasp_delay:
            self.transition(self.STORE_CMD)

    def step_store_cmd(self):
        self.publish_stop(force=False)
        self.store_pub.publish(Int32(data=1))
        rospy.loginfo(f"[Store {self.lock_label}:{self.lock_tid}] Published store cmd=1 (one-shot)")
        self.transition(self.WAIT_STORE)

    def step_wait_store(self):
        self.publish_stop(force=False)

        if self.store_success:
            rospy.loginfo("[Store] Success=1. Verify removed (with counting)...")
            self.transition(self.VERIFY_REMOVED)
            return

        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed > self.store_wait_timeout:
            rospy.logerr(f"[Store] Timeout >{self.store_wait_timeout:.1f}s. Reselect.")
            self.transition(self.SELECT_INSTANCE)

    def step_verify_removed(self):
        self.publish_stop(force=False)

        label = self.last_grasp_label
        tid = self.last_grasp_tid
        if label is None or tid is None:
            self.transition(self.SELECT_INSTANCE)
            return

        now = rospy.Time.now()

        # --- CHANGED: if detection stream is stale, back up 5 seconds to re-check ---
        if self.last_det_stamp is None or (now - self.last_det_stamp).to_sec() > self.detection_timeout:
            rospy.logwarn_throttle(1.0, "[VerifyRemoved] Detection stream stale; backing up to re-check...")
            self.transition(self.VERIFY_STALE_BACKUP)
            return

        candidates = self.visible_instances_for_label(label)
        current_ids = set(t for (t, _, _) in candidates)
        current_count = len(candidates)

        pose_occupied = False
        for (_, x, z) in candidates:
            if (abs(x - self.last_grasp_x) <= self.removal_region_x) and (abs(z - self.last_grasp_z) <= self.removal_region_z):
                pose_occupied = True
                break

        locked_absent = (tid not in current_ids)

        count_decreased = False
        if self.pre_label == label and self.pre_label_count is not None:
            count_decreased = (current_count <= max(0, self.pre_label_count - 1))

        removed_condition = (not pose_occupied) and (locked_absent or count_decreased)

        if removed_condition:
            self.verify_absent_count += 1
        else:
            self.verify_absent_count = 0

        elapsed = (now - self.state_enter_time).to_sec()
        rospy.loginfo_throttle(
            0.25,
            f"[VerifyRemoved {label}:{tid}] pose_occupied={pose_occupied} locked_absent={locked_absent} "
            f"count={current_count} pre={self.pre_label_count} decreased={count_decreased} "
            f"confirm={removed_condition} absent={self.verify_absent_count}/{self.verify_absent_cycles} elapsed={elapsed:.2f}s"
        )

        if self.verify_absent_count >= self.verify_absent_cycles:
            if self.targets.get(label, 0) > 0:
                self.targets[label] -= 1
                if self.targets[label] < 0:
                    self.targets[label] = 0

            rospy.loginfo(f"[Targets] Vision-confirmed stored. Updated: {self.targets}")

            self.lock_label = None
            self.lock_tid = None
            self.pre_label = None
            self.pre_label_count = None
            self.pre_label_ids = set()

            self.transition(self.SELECT_INSTANCE)
            return

        if elapsed > self.verify_timeout:
            rospy.logwarn(f"[VerifyRemoved] Timeout >{self.verify_timeout:.1f}s. Not decrementing. Reselect.")
            self.lock_label = None
            self.lock_tid = None
            self.pre_label = None
            self.pre_label_count = None
            self.pre_label_ids = set()
            self.transition(self.SELECT_INSTANCE)

    def step_verify_stale_backup(self):
        """
        NEW:
        When YOLO publishes nothing (no detections => no messages),
        back up for verify_stale_backup_time seconds to re-check.
        If detections return -> go back to VERIFY_REMOVED.
        If still no detections at all -> assume nothing visible and accept removal once.
        """
        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed < self.verify_stale_backup_time:
            self.publish_cmd(-abs(self.verify_stale_backup_speed), 0.0)
            rospy.loginfo_throttle(0.5, f"[VerifyStaleBackup] backing up... {elapsed:.1f}/{self.verify_stale_backup_time:.1f}s")
            return

        self.publish_stop(force=False)

        # If detections came back, go verify normally
        now = rospy.Time.now()
        if self.last_det_stamp is not None and (now - self.last_det_stamp).to_sec() <= self.detection_timeout:
            rospy.loginfo("[VerifyStaleBackup] detections resumed -> back to VERIFY_REMOVED")
            self.transition(self.VERIFY_REMOVED)
            return

        # Still no detections at all after backing up: treat as "nothing left visible".
        label = self.last_grasp_label
        if label is not None and self.targets.get(label, 0) > 0:
            self.targets[label] -= 1
            if self.targets[label] < 0:
                self.targets[label] = 0
            rospy.logwarn(f"[VerifyStaleBackup] still no detections -> assume removed; decrement {label}. targets={self.targets}")

        self.lock_label = None
        self.lock_tid = None
        self.pre_label = None
        self.pre_label_count = None
        self.pre_label_ids = set()
        self.verify_absent_count = 0

        self.transition(self.SELECT_INSTANCE)

    def step_search_backup(self):
        if self.targets_done():
            self.transition(self.FINISH)
            return

        if self.any_visible_matching_targets():
            self.transition(self.SELECT_INSTANCE)
            return

        elapsed = (rospy.Time.now() - self.state_enter_time).to_sec()
        if elapsed < self.search_backup_time:
            self.publish_cmd(-abs(self.search_backup_speed), 0.0)
            rospy.loginfo_throttle(0.5, f"[SearchBackup] backing up... {elapsed:.1f}/{self.search_backup_time:.1f}s")
            return

        self.publish_stop(force=False)

        if self.any_visible_matching_targets():
            rospy.loginfo("[SearchBackup] target found after backup. Reselect.")
            self.transition(self.SELECT_INSTANCE)
            return

        if self.finish_if_missing_after_backup:
            rospy.logwarn("[SearchBackup] no matching targets after backup. Finishing.")
            self.transition(self.FINISH)
        else:
            self.transition(self.SELECT_INSTANCE)

    def step_finish(self):
        self.publish_stop(force=False)
        if not self.finished_published:
            self.success_pub.publish(Int32(data=1))
            try:
                self._write_targets_to_file(self.target_json_path, self.targets)
                rospy.loginfo(f"[FINISH] Wrote updated targets to file: {self.target_json_path}")
            except Exception as e:
                rospy.logerr(f"[FINISH] Failed writing targets file: {e}")

            self.finished_published = True
            rospy.loginfo(f"[FINISH] success=1, remaining_targets={self.targets}")

        self.active = False
        self.reset_run_state()

    # ---------------- Main Loop ----------------
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not self.active:
                self.step_wait_activation()
                r.sleep()
                continue

            if self.state == self.WAIT_ACTIVATION:
                if self.any_remaining() and not self.targets_done():
                    self.transition(self.SELECT_INSTANCE)
                else:
                    self.transition(self.FINISH)

            elif self.state == self.SELECT_INSTANCE:
                self.step_select_instance()
            elif self.state == self.APPROACH:
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
            elif self.state == self.VERIFY_REMOVED:
                self.step_verify_removed()
            elif self.state == self.VERIFY_STALE_BACKUP:
                self.step_verify_stale_backup()
            elif self.state == self.SEARCH_BACKUP:
                self.step_search_backup()
            elif self.state == self.FINISH:
                self.step_finish()
            else:
                self.publish_stop(force=False)

            r.sleep()


if __name__ == "__main__":
    try:
        node = MultiObjectGraspStoreWithTrackingID()
        node.spin()
    except rospy.ROSInterruptException:
        pass

