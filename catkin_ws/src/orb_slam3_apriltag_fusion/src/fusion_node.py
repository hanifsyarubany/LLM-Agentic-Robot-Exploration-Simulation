#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf.transformations import quaternion_multiply, quaternion_inverse

class ORBTagFusion:
    def __init__(self):

        rospy.loginfo("ORB-SLAM3 + AprilTag fusion node started.")

        # Parameters
        # global_frame MUST match what apriltag_localization uses (world_frame_name)
        self.global_frame = rospy.get_param("~global_frame", "world")
        # This is the topic where apriltag_localization publishes world -> camera pose
        self.tag_pose_topic = rospy.get_param("~tag_pose_topic", "/camera_link")

        # Latest poses
        self.last_orb_pose = None      # orb_world -> camera
        self.last_tag_pose = None      # global_frame -> camera

        # TF broadcaster for global_frame -> orb_world
        self.br = tf2_ros.TransformBroadcaster()

        # Correction transform T_WO: global_frame -> orb_world
        self.T_WO = TransformStamped()
        self.T_WO.header.frame_id = self.global_frame
        self.T_WO.child_frame_id  = "orb_world"
        self.T_WO.transform.translation.x = 0.0
        self.T_WO.transform.translation.y = 0.0
        self.T_WO.transform.translation.z = 0.0
        self.T_WO.transform.rotation.x = 0.0
        self.T_WO.transform.rotation.y = 0.0
        self.T_WO.transform.rotation.z = 0.0
        self.T_WO.transform.rotation.w = 1.0   # identity rotation

        # Publisher for corrected pose in global_frame
        self.pub_corrected = rospy.Publisher(
            "/orb_slam3/pose_corrected",
            PoseStamped,
            queue_size=10
        )

        # Subscribers
        rospy.Subscriber("/orb_slam3/pose", PoseStamped, self.orb_callback)
        rospy.Subscriber(self.tag_pose_topic, PoseStamped, self.tag_callback)

        rospy.loginfo("ORBTagFusion initialized.")
        rospy.loginfo("  global_frame    = %s", self.global_frame)
        rospy.loginfo("  tag_pose_topic  = %s", self.tag_pose_topic)
        rospy.loginfo("Waiting for /orb_slam3/pose and %s ...", self.tag_pose_topic)

    # --------------------------------------------------
    # ORB-SLAM3 pose callback (orb_world -> camera_link)
    # --------------------------------------------------
    def orb_callback(self, msg):
        self.last_orb_pose = msg

        # Convert ORB pose (orb_world -> camera) to transform
        T_OC_orb = self.pose_to_tf(msg.pose)

        # Compute corrected global -> camera:
        # T_WC_corr = T_WO * T_OC_orb
        T_WC_corr = self.multiply_tf(self.T_WO.transform, T_OC_orb)

        # Build corrected PoseStamped in global frame
        corrected = PoseStamped()
        corrected.header.stamp = msg.header.stamp
        corrected.header.frame_id = self.global_frame
        corrected.pose = self.tf_to_pose(T_WC_corr)

        # Publish corrected pose
        self.pub_corrected.publish(corrected)

        # Always broadcast current global -> orb_world TF
        self.T_WO.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.T_WO)

    # --------------------------------------------------
    # AprilTag camera pose callback (global_frame -> camera_link)
    # --------------------------------------------------
    def tag_callback(self, msg):
        # msg is already global_frame -> camera pose from apriltag_localization
        self.last_tag_pose = msg

        # CASE 1: ORB is NOT available yet
        # -> directly use AprilTag camera pose as the corrected pose
        if self.last_orb_pose is None:
            corrected = PoseStamped()
            corrected.header = msg.header      # same stamp & frame_id (global_frame)
            corrected.pose   = msg.pose        # global camera pose
            self.pub_corrected.publish(corrected)
            return

        # CASE 2: ORB is available -> update correction transform T_WO
        dt = abs((msg.header.stamp - self.last_orb_pose.header.stamp).to_sec())
        if dt > 0.15:
            rospy.logwarn("ORB pose too old for correction (dt=%.3f)", dt)
            return

        # T_WC_tag: global_frame -> camera  (from AprilTag)
        # T_OC_orb: orb_world    -> camera  (from ORB-SLAM3)
        T_WC_tag = self.pose_to_tf(msg.pose)
        T_OC_orb = self.pose_to_tf(self.last_orb_pose.pose)

        # Compute global_frame -> orb_world transform:
        # T_WO = T_WC_tag * (T_OC_orb)^(-1)
        T_WO = self.multiply_tf(T_WC_tag, self.inverse_tf(T_OC_orb))

        # Update internal correction transform
        self.T_WO.header.stamp = rospy.Time.now()
        self.T_WO.transform = T_WO

        # Broadcast updated correction once
        self.br.sendTransform(self.T_WO)

        rospy.loginfo(">>> Updated %s -> orb_world correction using AprilTag!",
                      self.global_frame)

    # --------------------------------------------------
    # Helpers: Conversions + math
    # --------------------------------------------------
    def pose_to_tf(self, pose):
        T = TransformStamped().transform
        T.translation.x = pose.position.x
        T.translation.y = pose.position.y
        T.translation.z = pose.position.z
        T.rotation = pose.orientation
        return T

    def tf_to_pose(self, T):
        pose = Pose()
        pose.position.x = T.translation.x
        pose.position.y = T.translation.y
        pose.position.z = T.translation.z
        pose.orientation = T.rotation
        return pose

    def inverse_tf(self, T):
        inv = TransformStamped().transform

        # Inverse rotation
        q = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
        q_inv = quaternion_inverse(q)

        # Inverse translation
        t = [T.translation.x, T.translation.y, T.translation.z]
        t_inv = tf.transformations.quaternion_matrix(q_inv).dot(
            [-t[0], -t[1], -t[2], 1.0]
        )[:3]

        inv.translation.x = t_inv[0]
        inv.translation.y = t_inv[1]
        inv.translation.z = t_inv[2]

        inv.rotation.x = q_inv[0]
        inv.rotation.y = q_inv[1]
        inv.rotation.z = q_inv[2]
        inv.rotation.w = q_inv[3]

        return inv

    def multiply_tf(self, A, B):
        out = TransformStamped().transform

        ta = [A.translation.x, A.translation.y, A.translation.z]
        tb = [B.translation.x, B.translation.y, B.translation.z]

        qa = [A.rotation.x, A.rotation.y, A.rotation.z, A.rotation.w]
        qb = [B.rotation.x, B.rotation.y, B.rotation.z, B.rotation.w]

        # Rotate tb by qa
        tb_rot = tf.transformations.quaternion_matrix(qa).dot(
            [tb[0], tb[1], tb[2], 1.0]
        )[:3]

        out.translation.x = ta[0] + tb_rot[0]
        out.translation.y = ta[1] + tb_rot[1]
        out.translation.z = ta[2] + tb_rot[2]

        # Combine rotations
        q_out = quaternion_multiply(qa, qb)
        out.rotation.x = q_out[0]
        out.rotation.y = q_out[1]
        out.rotation.z = q_out[2]
        out.rotation.w = q_out[3]

        return out


if __name__ == "__main__":
    import tf2_ros  # imported here so it exists before use
    from tf.transformations import quaternion_multiply, quaternion_inverse

    rospy.init_node("orb_slam3_apriltag_fusion_node")
    ORBTagFusion()
    rospy.spin()
