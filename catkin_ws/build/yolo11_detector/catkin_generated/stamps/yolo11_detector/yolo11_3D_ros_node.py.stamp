#!/usr/bin/env python3
import rospy
import rospkg
import os
import time
import numpy as np
import cv2

from ultralytics import YOLO

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge, CvBridgeError
import message_filters


class Yolo11DepthNode:
    def __init__(self):
        # ======================
        # Parameters
        # ======================
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("yolo11_detector")
        default_model_path = os.path.join(pkg_path, "models", "model.pt")

        # Model + device
        self.model_path = rospy.get_param("~model_path", default_model_path)
        self.device = rospy.get_param("~device", "cuda:0")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.imgsz = rospy.get_param("~imgsz", 640)

        # Topics
        self.color_topic = rospy.get_param("~color_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/depth/camera_info")

        # Frame id for published data (RViz)
        self.camera_frame = rospy.get_param("~camera_frame", "yolo_camera")

        # Depth scaling:
        # - If depth image is uint16 (16UC1), we multiply by depth_scale (default 0.001 -> mm->m)
        # - If depth image is float32 (32FC1), we assume it’s already in meters.
        self.depth_scale = rospy.get_param("~depth_scale", 0.001)

        # Window / timing
        self.show_window = rospy.get_param("~show_window", False)
        self.print_timing = rospy.get_param("~print_timing", True)

        rospy.loginfo(f"[YOLO11] Loading model from {self.model_path} on {self.device}")
        self.model = YOLO(self.model_path)
        rospy.loginfo("[YOLO11] Model loaded.")

        self.bridge = CvBridge()

        # Camera intrinsics (from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ======================
        # Publishers (same as your example)
        # ======================
        self.depth_pub = rospy.Publisher(
            "/yolo_rs/depth_xyz", PointStamped, queue_size=10
        )
        self.flag_pub = rospy.Publisher(
            "/yolo_rs/object_detected", Int32, queue_size=10
        )
        self.image_pub = rospy.Publisher(
            "/yolo_rs/image_annotated", Image, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            "/yolo_rs/markers", Marker, queue_size=10
        )
        self.xy_pub = rospy.Publisher(
            "/yolo_rs/xy", PointStamped, queue_size=10
        )

        # ======================
        # Subscribers (sync color + depth + camera_info)
        # ======================
        color_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        info_sub = message_filters.Subscriber(self.camera_info_topic, CameraInfo)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub, info_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.callback)

        rospy.loginfo(f"[YOLO11] Subscribing color: {self.color_topic}")
        rospy.loginfo(f"[YOLO11] Subscribing depth: {self.depth_topic}")
        rospy.loginfo(f"[YOLO11] Subscribing camera_info: {self.camera_info_topic}")
        rospy.loginfo("[YOLO11] Depth + 3D node initialized.")

    # ------------------------------------------------------------------
    # CameraInfo -> intrinsics
    # ------------------------------------------------------------------
    def _update_intrinsics(self, cam_info):
        # Standard pinhole camera model
        # K = [fx 0 cx  0 fy cy  0 0 1]
        self.fx = cam_info.K[0]
        self.fy = cam_info.K[4]
        self.cx = cam_info.K[2]
        self.cy = cam_info.K[5]

    # ------------------------------------------------------------------
    # Main callback (color + depth + camera_info)
    # ------------------------------------------------------------------
    def callback(self, color_msg, depth_msg, cam_info_msg):
        t0 = time.perf_counter()

        if self.fx is None:
            self._update_intrinsics(cam_info_msg)
            rospy.loginfo(
                f"[YOLO11] Intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, "
                f"cx={self.cx:.2f}, cy={self.cy:.2f}"
            )

        # Convert color image
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"[YOLO11] CvBridge color error: {e}")
            return

        # Convert depth image (keep original dtype)
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"[YOLO11] CvBridge depth error: {e}")
            return

        h, w = color_image.shape[:2]

        # YOLO inference
        t1 = time.perf_counter()
        results = self.model(
            color_image,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            device=self.device,
            verbose=False,
        )
        t2 = time.perf_counter()

        r = results[0]
        annotated = color_image.copy()
        detection_present = False

        # ======================
        # Handle detections
        # ======================
        if r.boxes is not None and len(r.boxes) > 0:
            detection_present = True

            boxes = r.boxes
            xyxy = boxes.xyxy.cpu().numpy()
            conf = boxes.conf.cpu().numpy()
            cls = boxes.cls.cpu().numpy().astype(int)

            for i_det in range(len(xyxy)):
                class_id = int(cls[i_det])
                class_name = r.names[class_id] if r.names else str(class_id)

                x1, y1, x2, y2 = xyxy[i_det]
                cx_pix = int((x1 + x2) / 2.0)
                cy_pix = int((y1 + y2) / 2.0)

                # Clamp to image bounds
                cx_pix = max(0, min(w - 1, cx_pix))
                cy_pix = max(0, min(h - 1, cy_pix))

                # NEW: publish 2D center (pixels) regardless of depth
                xy_msg = PointStamped()
                xy_msg.header.stamp = color_msg.header.stamp
                # you can also use self.camera_frame if you prefer
                xy_msg.header.frame_id = class_name
                xy_msg.point.x = float(cx_pix)
                xy_msg.point.y = float(cy_pix)
                xy_msg.point.z = 0.0
                self.xy_pub.publish(xy_msg)

                # ---- Robust depth: median in small patch (similar to your code) ----
                patch_radius = 3
                x_min = max(0, cx_pix - patch_radius)
                x_max = min(w, cx_pix + patch_radius + 1)
                y_min = max(0, cy_pix - patch_radius)
                y_max = min(h, cy_pix + patch_radius + 1)

                depth_patch = depth_image[y_min:y_max, x_min:x_max]

                # Convert to meters depending on encoding / dtype
                if depth_patch.dtype == np.uint16:
                    depth_patch_m = depth_patch.astype(np.float32) * self.depth_scale
                else:
                    # Assume float (meters)
                    depth_patch_m = depth_patch.astype(np.float32)

                valid = depth_patch_m[depth_patch_m > 0]
                if valid.size > 0:
                    depth_m = float(np.median(valid))
                else:
                    depth_m = 0.0

                # ---- Deproject to 3D (camera frame) ----
                if depth_m > 0 and self.fx not in (None, 0) and self.fy not in (None, 0):
                    Z = depth_m
                    u = float(cx_pix)
                    v = float(cy_pix)
                    X = (u - self.cx) * Z / self.fx
                    Y = (v - self.cy) * Z / self.fy

                    depth_text = f"{Z:.2f}m"
                    xyz_text = f"X={X:.2f} Y={Y:.2f} Z={Z:.2f}"
                else:
                    X = Y = Z = 0.0
                    depth_text = "?m"
                    xyz_text = "X=? Y=? Z=?"

                conf_i = float(conf[i_det])

                # ======================
                # Draw on image
                # ======================
                label = f"{class_name} {conf_i:.2f} {depth_text}"
                p1 = (int(x1), int(y1))
                p2 = (int(x2), int(y2))

                cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
                text_org = (int(x1), max(0, int(y1) - 10))
                cv2.putText(
                    annotated,
                    label,
                    text_org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                if depth_m > 0:
                    text_org2 = (int(x1), min(h - 5, int(y2) + 15))
                    cv2.putText(
                        annotated,
                        xyz_text,
                        text_org2,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                    # ======================
                    # Publish PointStamped
                    # ======================
                    pt_msg = PointStamped()
                    pt_msg.header.stamp = color_msg.header.stamp
                    pt_msg.header.frame_id = class_name
                    pt_msg.point.x = X
                    pt_msg.point.y = Y
                    pt_msg.point.z = Z
                    self.depth_pub.publish(pt_msg)

                    # ======================
                    # Publish Marker
                    # ======================
                    marker = Marker()
                    marker.header.stamp = color_msg.header.stamp
                    marker.header.frame_id = self.camera_frame
                    marker.ns = "yolo_rs"
                    marker.id = i_det
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD

                    marker.pose.position.x = X
                    marker.pose.position.y = Y
                    marker.pose.position.z = Z
                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = 0.0
                    marker.pose.orientation.w = 1.0

                    marker.scale.x = 0.05
                    marker.scale.y = 0.05
                    marker.scale.z = 0.05

                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.8

                    marker.lifetime = rospy.Duration(0.2)
                    self.marker_pub.publish(marker)

                    rospy.logdebug(
                        f"[PUB] class={class_id} XYZ=({X:.3f}, {Y:.3f}, {Z:.3f}) depth={depth_m:.3f}"
                    )
                else:
                    rospy.logdebug(f"[DEPTH] {class_name}: no valid depth in patch")

        # ======================
        # Publish 0/1 flag per frame
        # ======================
        flag_value = 1 if detection_present else 0
        self.flag_pub.publish(Int32(data=flag_value))

        # ======================
        # Publish annotated image
        # ======================
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img_msg.header.stamp = color_msg.header.stamp
            img_msg.header.frame_id = self.camera_frame
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            rospy.logwarn(f"[YOLO11] CvBridge annotated error: {e}")

        # Optional local window
        if self.show_window:
            cv2.imshow("YOLO + Depth", annotated)
            cv2.waitKey(1)

        # Timing
        if self.print_timing:
            infer_ms = (t2 - t1) * 1000.0
            total_ms = (t2 - t0) * 1000.0
            fps = 1000.0 / max(total_ms, 1e-3)
            rospy.loginfo(
                f"[YOLO11] infer={infer_ms:.1f} ms  total={total_ms:.1f} ms  FPS≈{fps:.1f}"
            )


def main():
    rospy.init_node("yolo11_depth_node")
    node = Yolo11DepthNode()
    rospy.loginfo("[YOLO11] Depth + 3D node started.")
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
