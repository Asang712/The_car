#!/usr/bin/env python3
# encoding: utf-8

import os

# 在导入 mediapipe / media_library 前设置，适合 Ubuntu 22 虚拟机 CPU-only 环境
# 这里的 TF 是 TensorFlow，不是 ROS 的 tf/tf2
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import time

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from .media_library import HandDetector


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


def to_int(value, default=0):
    try:
        return int(value)
    except Exception:
        return default


def to_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


class MediapipeGestureNode(Node):
    def __init__(self):
        super().__init__("mediapipe_gesture_node")

        # =========================
        # 基础参数
        # =========================
        self.declare_parameter("rgb_topic", "/camera/image_raw")

        # MediaPipe Hands 参数
        self.declare_parameter("static_image_mode", False)
        self.declare_parameter("max_hands", 1)
        self.declare_parameter("detector_con", 0.7)
        self.declare_parameter("track_con", 0.7)

        # 显示参数
        self.declare_parameter("window_name", "mediapipe_gesture")
        self.declare_parameter("resize_display", True)
        self.declare_parameter("keep_aspect_ratio", True)
        self.declare_parameter("display_width", 640)
        self.declare_parameter("display_height", 480)

        # 调试输出参数
        self.declare_parameter("print_gesture", False)
        self.declare_parameter("print_interval", 0.5)

        # 镜像显示参数
        self.declare_parameter("flip_image", False)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)

        static_image_mode = to_bool(self.get_parameter("static_image_mode").value)
        max_hands = to_int(self.get_parameter("max_hands").value, 1)
        detector_con = to_float(self.get_parameter("detector_con").value, 0.7)
        track_con = to_float(self.get_parameter("track_con").value, 0.7)

        self.window_name = str(self.get_parameter("window_name").value)
        self.resize_display = to_bool(self.get_parameter("resize_display").value)
        self.keep_aspect_ratio = to_bool(self.get_parameter("keep_aspect_ratio").value)
        self.display_width = to_int(self.get_parameter("display_width").value, 640)
        self.display_height = to_int(self.get_parameter("display_height").value, 480)

        self.print_gesture = to_bool(self.get_parameter("print_gesture").value)
        self.print_interval = to_float(self.get_parameter("print_interval").value, 0.5)
        self.flip_image = to_bool(self.get_parameter("flip_image").value)

        # =========================
        # OpenCV 窗口
        # WINDOW_NORMAL 表示窗口可以手动拖拽调整大小
        # =========================
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        if self.display_width > 0 and self.display_height > 0:
            cv.resizeWindow(self.window_name, self.display_width, self.display_height)

        # =========================
        # 摄像头 QoS
        # BEST_EFFORT + depth=1 可以降低视频延迟
        # =========================
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.rgb_bridge = CvBridge()

        # 使用普通订阅即可，单个 RGB 图像话题不需要 message_filters 同步器
        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            self.qos_profile,
        )

        # =========================
        # 手势识别器
        # 使用 media_library.py 中的 HandDetector
        # =========================
        self.hand_detector = HandDetector(
            mode=static_image_mode,
            maxHands=max_hands,
            detectorCon=detector_con,
            trackCon=track_con,
        )

        self.p_time = time.time()
        self.last_print_time = 0.0
        self.current_gesture = "none"

        self.get_logger().info("Mediapipe gesture node started.")
        self.get_logger().info(f"Subscribe image topic: {self.rgb_topic}")
        self.get_logger().info(
            f"HandDetector: static_image_mode={static_image_mode}, "
            f"max_hands={max_hands}, "
            f"detector_con={detector_con}, "
            f"track_con={track_con}"
        )
        self.get_logger().info(
            f"Display: window_name={self.window_name}, "
            f"resize_display={self.resize_display}, "
            f"keep_aspect_ratio={self.keep_aspect_ratio}, "
            f"display_width={self.display_width}, "
            f"display_height={self.display_height}"
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.rgb_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        if frame is None or frame.size == 0:
            self.get_logger().warn("Received empty image frame.")
            return

        if self.flip_image:
            frame = cv.flip(frame, 1)

        frame = self.process(frame)

        display_frame = self.prepare_display_image(frame)

        cv.imshow(self.window_name, display_frame)

        key = cv.waitKey(1) & 0xFF
        if key == ord("q") or key == ord("Q"):
            self.get_logger().info("Pressed q, shutting down mediapipe_gesture_node.")
            os._exit(0)  # pylint: disable=protected-access

    def process(self, frame):
        # media_library.HandDetector.findHands 返回:
        # frame, lmList, bbox
        frame, lm_list, bbox = self.hand_detector.findHands(frame)

        if len(lm_list) != 0:
            self.current_gesture = self.hand_detector.get_gesture(lm_list)
        else:
            self.current_gesture = "none"

        now = time.time()

        # 限制终端打印频率，避免刷屏
        if (
            self.print_gesture
            and now - self.last_print_time >= self.print_interval
        ):
            self.get_logger().info(f"gesture: {self.current_gesture}")
            self.last_print_time = now

        fps = self.compute_fps(now)

        self.draw_status(frame, fps, self.current_gesture, bbox)

        return frame

    def compute_fps(self, now_time):
        dt = now_time - self.p_time
        self.p_time = now_time

        if dt <= 0.0:
            return 0.0

        return 1.0 / dt

    def draw_status(self, frame, fps, gesture, bbox):
        img_h, img_w = frame.shape[:2]

        # 顶部半透明背景，避免文字看不清
        overlay = frame.copy()
        cv.rectangle(
            overlay,
            (0, 0),
            (img_w, 90),
            (0, 0, 0),
            cv.FILLED,
        )
        cv.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)

        cv.putText(
            frame,
            f"Gesture: {gesture}",
            (20, 35),
            cv.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 255),
            2,
        )

        cv.putText(
            frame,
            f"FPS: {int(fps)}",
            (20, 70),
            cv.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )

        cv.putText(
            frame,
            "q: quit",
            (img_w - 120, 35),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        # bbox 由 media_library.HandDetector 输出，格式通常为 (xmin, ymin, xmax, ymax)
        try:
            if bbox and len(bbox) == 4:
                x_min, y_min, x_max, y_max = bbox
                if x_max > x_min and y_max > y_min:
                    cv.rectangle(
                        frame,
                        (int(x_min), int(y_min)),
                        (int(x_max), int(y_max)),
                        (0, 255, 0),
                        2,
                    )
        except Exception:
            pass

    def prepare_display_image(self, image):
        """
        显示前处理：
        - resize_display=False：显示原始图像
        - keep_aspect_ratio=True：等比例缩放并补黑边，保证完整显示不裁剪、不变形
        - keep_aspect_ratio=False：直接缩放到 display_width x display_height
        """

        if (
            not self.resize_display
            or self.display_width <= 0
            or self.display_height <= 0
        ):
            return image

        if self.keep_aspect_ratio:
            return self.resize_with_padding(
                image,
                self.display_width,
                self.display_height,
            )

        return cv.resize(
            image,
            (self.display_width, self.display_height),
            interpolation=cv.INTER_AREA,
        )

    def resize_with_padding(self, image, target_w, target_h):
        """
        等比例缩放到目标尺寸内，并用黑边填充。
        这样不会裁剪图像，保证画面完整显示。
        """

        src_h, src_w = image.shape[:2]

        if src_w <= 0 or src_h <= 0:
            return image

        scale = min(target_w / src_w, target_h / src_h)

        new_w = max(1, int(src_w * scale))
        new_h = max(1, int(src_h * scale))

        resized = cv.resize(
            image,
            (new_w, new_h),
            interpolation=cv.INTER_AREA,
        )

        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)

        x_offset = (target_w - new_w) // 2
        y_offset = (target_h - new_h) // 2

        canvas[
            y_offset:y_offset + new_h,
            x_offset:x_offset + new_w,
        ] = resized

        return canvas

    def destroy_node(self):
        try:
            if hasattr(self.hand_detector, "hands"):
                self.hand_detector.hands.close()
        except Exception:
            pass

        try:
            cv.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MediapipeGestureNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
