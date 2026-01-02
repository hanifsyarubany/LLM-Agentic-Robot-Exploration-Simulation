#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
import rospkg


class Yolo11Node:
    def __init__(self):
        # Resolve default model path inside this package
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("yolo11_detector")
        default_model_path = os.path.join(pkg_path, "models", "model.pt")

        self.model_path = rospy.get_param("~model_path", default_model_path)
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.conf_thres = rospy.get_param("~conf_thres", 0.25)
        self.device = rospy.get_param("~device", "cuda:0")

        rospy.loginfo(f"[YOLO11] Loading model from {self.model_path} on {self.device}")
        self.model = YOLO(self.model_path)

        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )

        self.pub_debug = rospy.Publisher("~debug_image", Image, queue_size=1)

        rospy.loginfo(f"[YOLO11] Subscribing to {self.image_topic}")
        rospy.loginfo("[YOLO11] Publishing annotated image on ~debug_image")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"[YOLO11] cv_bridge error: {e}")
            return

        results = self.model.predict(
            source=frame,
            conf=self.conf_thres,
            verbose=False,
            device=self.device,
        )

        if len(results) > 0:
            annotated = results[0].plot()
        else:
            annotated = frame

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub_debug.publish(out_msg)


def main():
    rospy.init_node("yolo11_node")
    node = Yolo11Node()
    rospy.loginfo("[YOLO11] Node started.")
    rospy.spin()


if __name__ == "__main__":
    main()
