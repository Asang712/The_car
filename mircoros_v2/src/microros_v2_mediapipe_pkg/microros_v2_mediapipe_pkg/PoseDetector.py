#!/usr/bin/env python3
# encoding: utf-8

import os

# 在导入 mediapipe 前设置，避免 Ubuntu 虚拟机中误用 TF/GPU
os.environ["MEDIAPIPE_DISABLE_TF"] = "1"
os.environ["MEDIAPIPE_DISABLE_GPU"] = "1"

import cv2 as cv
import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class PoseDetector(Node):
    def __init__(self):
        super().__init__("pose_detector_node")

        # =========================
        # 参数声明
        # =========================
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("static_image_mode", False)
        self.declare_parameter("smooth_landmarks", True)
        self.declare_parameter("detection_con", 0.5)
        self.declare_parameter("track_con", 0.5)
        self.declare_parameter("draw_on_source", False)
        self.declare_parameter("window_name", "pose_detector")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        static_image_mode = bool(self.get_parameter("static_image_mode").value)
        smooth_landmarks = bool(self.get_parameter("smooth_landmarks").value)
        detection_con = float(self.get_parameter("detection_con").value)
        track_con = float(self.get_parameter("track_con").value)
        self.draw_on_source = bool(self.get_parameter("draw_on_source").value)
        self.window_name = str(self.get_parameter("window_name").value)

        # 摄像头图像建议使用 BEST_EFFORT，减少图像堆积延迟
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =========================
        # MediaPipe Pose 初始化
        # =========================
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils

        self.pose = self.mp_pose.Pose(
            static_image_mode=static_image_mode,
            smooth_landmarks=smooth_landmarks,
            min_detection_confidence=detection_con,
            min_tracking_confidence=track_con,
        )

        self.lm_draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 0, 255),
            thickness=-1,
            circle_radius=6,
        )

        self.draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 255, 0),
            thickness=2,
            circle_radius=2,
        )

        self.rgb_bridge = CvBridge()

        self.sub_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.get_rgb_image_callback,
            self.qos_profile,
        )

        self.get_logger().info("PoseDetector node started.")
        self.get_logger().info(f"Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        frame, pose_img = self.pub_pose_point(rgb_image, draw=self.draw_on_source)
        dist = self.frame_combine(frame, pose_img)

        cv.imshow(self.window_name, dist)

        # 保持 OpenCV 窗口刷新
        # 按 q 可关闭节点
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down PoseDetector node.")
            os._exit(0)  # pylint: disable=protected-access

    def pub_pose_point(self, frame, draw=True):
        pose_img = np.zeros(frame.shape, np.uint8)

        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.pose.process(img_rgb)

        if results.pose_landmarks:
            if draw:
                self.mp_draw.draw_landmarks(
                    frame,
                    results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    self.lm_draw_spec,
                    self.draw_spec,
                )

            self.mp_draw.draw_landmarks(
                pose_img,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                self.lm_draw_spec,
                self.draw_spec,
            )

        return frame, pose_img

    def frame_combine(self, frame, src):
        if len(frame.shape) == 3:
            frame_h, frame_w = frame.shape[:2]
            src_h, src_w = src.shape[:2]

            dst = np.zeros(
                (max(frame_h, src_h), frame_w + src_w, 3),
                np.uint8,
            )

            dst[:, :frame_w] = frame[:, :]
            dst[:, frame_w:] = src[:, :]
        else:
            src = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

            frame_h, frame_w = frame.shape[:2]
            img_h, img_w = src.shape[:2]

            dst = np.zeros((frame_h, frame_w + img_w), np.uint8)
            dst[:, :frame_w] = frame[:, :]
            dst[:, frame_w:] = src[:, :]

        return dst

    def destroy_node(self):
        try:
            self.pose.close()
        except Exception:
            pass

        try:
            cv.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    print("start pose detector")

    rclpy.init(args=args)

    node = PoseDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == "__main__":
    main()
