#!/usr/bin/env python3
# encoding: utf-8

import os

# 在导入 mediapipe 前设置，适合 Ubuntu 虚拟机 CPU-only 环境
# 这里的 TF 是 TensorFlow，不是 ROS 的 tf/tf2
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


class FaceMesh(Node):
    def __init__(self):
        super().__init__("face_mesh_node")

        # =========================
        # 参数声明
        # =========================
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("static_image_mode", False)
        self.declare_parameter("max_faces", 2)
        self.declare_parameter("detection_con", 0.5)
        self.declare_parameter("tracking_con", 0.5)
        self.declare_parameter("draw_on_source", False)
        self.declare_parameter("window_name", "face_mesh")

        self.rgb_topic = self.get_parameter("rgb_topic").value
        static_image_mode = bool(self.get_parameter("static_image_mode").value)
        max_faces = int(self.get_parameter("max_faces").value)
        detection_con = float(self.get_parameter("detection_con").value)
        tracking_con = float(self.get_parameter("tracking_con").value)
        self.draw_on_source = bool(self.get_parameter("draw_on_source").value)
        self.window_name = str(self.get_parameter("window_name").value)

        # 摄像头图像建议使用 BEST_EFFORT，避免图像堆积导致延迟
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # =========================
        # MediaPipe FaceMesh 初始化
        # =========================
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_face_mesh = mp.solutions.face_mesh

        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=static_image_mode,
            max_num_faces=max_faces,
            min_detection_confidence=detection_con,
            min_tracking_confidence=tracking_con,
        )

        self.lm_draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 0, 255),
            thickness=-1,
            circle_radius=3,
        )

        self.draw_spec = self.mp_draw.DrawingSpec(
            color=(0, 255, 0),
            thickness=1,
            circle_radius=1,
        )

        self.rgb_bridge = CvBridge()

        self.sub_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.get_rgb_image_callback,
            self.qos_profile,
        )

        self.get_logger().info("FaceMesh node started.")
        self.get_logger().info(f"Subscribe image topic: {self.rgb_topic}")

    def get_rgb_image_callback(self, msg: Image):
        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        frame, face_mesh_img = self.pub_face_mesh_point(
            rgb_image,
            draw=self.draw_on_source,
        )

        dist = self.frame_combine(frame, face_mesh_img)

        cv.imshow(self.window_name, dist)

        # 保持 OpenCV 窗口刷新
        # 按 q 可关闭节点
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down FaceMesh node.")
            os._exit(0)  # pylint: disable=protected-access

    def pub_face_mesh_point(self, frame, draw=True):
        face_mesh_img = np.zeros(frame.shape, np.uint8)

        img_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.face_mesh.process(img_rgb)

        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                if draw:
                    self.mp_draw.draw_landmarks(
                        frame,
                        face_landmarks,
                        self.mp_face_mesh.FACEMESH_CONTOURS,
                        self.lm_draw_spec,
                        self.draw_spec,
                    )

                self.mp_draw.draw_landmarks(
                    face_mesh_img,
                    face_landmarks,
                    self.mp_face_mesh.FACEMESH_CONTOURS,
                    self.lm_draw_spec,
                    self.draw_spec,
                )

        return frame, face_mesh_img

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
            self.face_mesh.close()
        except Exception:
            pass

        try:
            cv.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    print("start face mesh detector")

    rclpy.init(args=args)

    node = FaceMesh()

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
