#!/usr/bin/env python3
# encoding: utf-8

import os
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import enum
import gc
import math
import time

import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


def distance(point_1, point_2):
    if len(point_1) != len(point_2):
        raise ValueError("两点的维度不一致")
    return math.sqrt(sum([(point_2[i] - point_1[i]) ** 2 for i in range(len(point_1))]))


def vector_2d_angle(v1, v2):
    norm_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)

    if norm_v1_v2 == 0:
        return 0.0

    cos = v1.dot(v2) / norm_v1_v2
    cos = float(np.clip(cos, -1.0, 1.0))

    sin = np.cross(v1, v2) / norm_v1_v2
    angle = np.degrees(np.arctan2(sin, cos))

    return angle


def get_area_max_contour(contours, threshold=100):
    contour_area = zip(
        contours,
        tuple(map(lambda c: math.fabs(cv.contourArea(c)), contours))
    )

    contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))

    if len(contour_area) > 0:
        return max(contour_area, key=lambda c_a: c_a[1])

    return None


def get_hand_landmarks(img, landmarks):
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)


def hand_angle(landmarks):
    angle_list = []

    angle_list.append(vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16]))
    angle_list.append(vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20]))

    return [abs(a) for a in angle_list]


def h_gesture(angle_list):
    """
    five: 五指张开，用于结束轨迹
    one : 食指伸出，用于准备开始和绘制轨迹
    none: 其他手势
    """
    thr_angle = 65.0
    thr_angle_s = 49.0

    if all(a < thr_angle_s for a in angle_list):
        return "five"

    if (
        (angle_list[0] > 5)
        and (angle_list[1] < thr_angle_s)
        and (angle_list[2] > thr_angle)
        and (angle_list[3] > thr_angle)
        and (angle_list[4] > thr_angle)
    ):
        return "one"

    return "none"


def parse_bool(value):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")

    return bool(value)


def _normalize_color(img, color):
    if img.ndim == 2:
        return int(color[0]) if isinstance(color, tuple) else int(color)

    if img.ndim == 3 and img.shape[2] == 1:
        return int(color[0]) if isinstance(color, tuple) else int(color)

    return color


def draw_points(img, points, thickness=4, color=(255, 0, 0)):
    points = np.array(points).astype(dtype=np.int32)

    if len(points) < 2:
        return

    draw_color = _normalize_color(img, color)

    for i, p in enumerate(points):
        if i + 1 >= len(points):
            break

        cv.line(img, tuple(p), tuple(points[i + 1]), draw_color, thickness)


def get_track_img(points):
    points = np.array(points).astype(dtype=np.int32)

    x_min, y_min = np.min(points, axis=0).tolist()
    x_max, y_max = np.max(points, axis=0).tolist()

    width = max(x_max - x_min + 100, 120)
    height = max(y_max - y_min + 100, 120)

    track_img = np.full((height, width), 0, dtype=np.uint8)

    points = points - [x_min, y_min]
    points = points + [50, 50]

    draw_points(track_img, points, 1, 255)

    return track_img


def put_text_with_background(
    img,
    text,
    org,
    font_scale=0.45,
    text_color=(255, 255, 255),
    bg_color=(0, 0, 0),
    thickness=1,
    alpha=0.35,
    pad_x=4,
    pad_y=3,
):
    """
    小面积半透明文字提示，尽量减少对画面的遮挡。
    """
    x, y = org
    font = cv.FONT_HERSHEY_SIMPLEX

    (tw, th), baseline = cv.getTextSize(text, font, font_scale, thickness)

    x1 = max(x - pad_x, 0)
    y1 = max(y - th - pad_y, 0)
    x2 = min(x + tw + pad_x, img.shape[1] - 1)
    y2 = min(y + baseline + pad_y, img.shape[0] - 1)

    overlay = img.copy()
    cv.rectangle(overlay, (x1, y1), (x2, y2), bg_color, -1)
    cv.addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img)

    cv.putText(
        img,
        text,
        (x, y),
        font,
        font_scale,
        text_color,
        thickness,
        cv.LINE_AA,
    )


def put_center_text(
    img,
    text,
    y_ratio=0.88,
    font_scale=0.55,
    text_color=(255, 255, 255),
    bg_color=(0, 120, 0),
    thickness=1,
):
    """
    中心提示改为靠下小提示，避免遮挡手部和绘制区域。
    """
    font = cv.FONT_HERSHEY_SIMPLEX
    (tw, _), _ = cv.getTextSize(text, font, font_scale, thickness)

    x = max((img.shape[1] - tw) // 2, 0)
    y = int(img.shape[0] * y_ratio)

    put_text_with_background(
        img,
        text,
        (x, y),
        font_scale=font_scale,
        text_color=text_color,
        bg_color=bg_color,
        thickness=thickness,
        alpha=0.40,
        pad_x=6,
        pad_y=4,
    )


class State(enum.Enum):
    NULL = 0
    TRACKING = 1


class FingerTrajectory(Node):
    def __init__(self):
        super().__init__("finger_trajectory_node")

        # 5~10 FPS 摄像头推荐默认值
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("detection_con", 0.5)
        self.declare_parameter("tracking_con", 0.05)
        self.declare_parameter("start_count_threshold", 5)
        self.declare_parameter("lost_timeout", 3.0)

        # OpenCV 窗口名称
        self.declare_parameter("window_name", "finger_trajectory")
        self.declare_parameter("track_window_name", "finger_track")

        # 显示提示相关参数
        self.declare_parameter("show_fps", True)
        self.declare_parameter("start_prompt_duration", 1.2)

        # 小字体提示参数
        self.declare_parameter("overlay_font_scale", 0.45)
        self.declare_parameter("overlay_thickness", 1)
        self.declare_parameter("overlay_alpha", 0.35)
        self.declare_parameter("center_prompt_font_scale", 0.55)

        # 显示窗口放大相关参数
        self.declare_parameter("display_scale", 1.8)
        self.declare_parameter("track_display_scale", 2.2)
        self.declare_parameter("display_width", 0)
        self.declare_parameter("display_height", 0)
        self.declare_parameter("track_display_width", 0)
        self.declare_parameter("track_display_height", 0)

        # 轨迹采样与绘制参数
        self.declare_parameter("min_point_distance", 5.0)
        self.declare_parameter("smooth_alpha", 0.55)
        self.declare_parameter("track_line_thickness", 3)

        # 图形识别相关参数
        self.declare_parameter("contour_area_threshold", 200.0)
        self.declare_parameter("morph_iterations", 2)
        self.declare_parameter("approx_epsilon_ratio", 0.026)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)

        detection_con = float(self.get_parameter("detection_con").value)
        tracking_con = float(self.get_parameter("tracking_con").value)

        self.start_count_threshold = int(self.get_parameter("start_count_threshold").value)
        self.lost_timeout = float(self.get_parameter("lost_timeout").value)

        self.window_name = str(self.get_parameter("window_name").value)
        self.track_window_name = str(self.get_parameter("track_window_name").value)

        self.show_fps = parse_bool(self.get_parameter("show_fps").value)
        self.start_prompt_duration = float(self.get_parameter("start_prompt_duration").value)

        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_thickness = int(self.get_parameter("overlay_thickness").value)
        self.overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        self.center_prompt_font_scale = float(self.get_parameter("center_prompt_font_scale").value)

        self.display_scale = float(self.get_parameter("display_scale").value)
        self.track_display_scale = float(self.get_parameter("track_display_scale").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.track_display_width = int(self.get_parameter("track_display_width").value)
        self.track_display_height = int(self.get_parameter("track_display_height").value)

        self.min_point_distance = float(self.get_parameter("min_point_distance").value)
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.track_line_thickness = int(self.get_parameter("track_line_thickness").value)

        self.contour_area_threshold = float(self.get_parameter("contour_area_threshold").value)
        self.morph_iterations = int(self.get_parameter("morph_iterations").value)
        self.approx_epsilon_ratio = float(self.get_parameter("approx_epsilon_ratio").value)

        self.start_count_threshold = max(self.start_count_threshold, 1)
        self.lost_timeout = max(self.lost_timeout, 0.5)

        self.overlay_font_scale = max(self.overlay_font_scale, 0.25)
        self.overlay_thickness = max(self.overlay_thickness, 1)
        self.overlay_alpha = float(np.clip(self.overlay_alpha, 0.0, 1.0))
        self.center_prompt_font_scale = max(self.center_prompt_font_scale, 0.3)

        self.display_scale = max(self.display_scale, 0.1)
        self.track_display_scale = max(self.track_display_scale, 0.1)
        self.display_width = max(self.display_width, 0)
        self.display_height = max(self.display_height, 0)
        self.track_display_width = max(self.track_display_width, 0)
        self.track_display_height = max(self.track_display_height, 0)

        self.min_point_distance = max(self.min_point_distance, 0.0)
        self.smooth_alpha = float(np.clip(self.smooth_alpha, 0.0, 1.0))
        self.track_line_thickness = max(self.track_line_thickness, 1)

        self.contour_area_threshold = max(self.contour_area_threshold, 1.0)
        self.morph_iterations = max(self.morph_iterations, 0)
        self.approx_epsilon_ratio = max(self.approx_epsilon_ratio, 0.001)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.drawing = mp.solutions.drawing_utils

        self.state = State.NULL
        self.points = []
        self.smoothed_tip = None

        self.start_count = 0
        self.no_finger_timestamp = time.time()
        self.tracking_start_timestamp = 0.0

        self.last_frame_timestamp = None
        self.fps = 0.0

        self.gc_stamp = time.time()
        self.last_result_text = ""
        self.last_result_until = 0.0

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=tracking_con,
            min_detection_confidence=detection_con,
        )

        self.rgb_bridge = CvBridge()

        self.sub_rgb = self.create_subscription(
            Image,
            self.rgb_topic,
            self.get_rgb_image_callback,
            self.qos_profile,
        )

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.namedWindow(self.track_window_name, cv.WINDOW_NORMAL)

        self.get_logger().info(
            "FingerTrajectory node started. "
            f"rgb_topic={self.rgb_topic}, "
            f"detection_con={detection_con}, "
            f"tracking_con={tracking_con}, "
            f"start_count_threshold={self.start_count_threshold}, "
            f"lost_timeout={self.lost_timeout}, "
            f"display_scale={self.display_scale}, "
            f"track_display_scale={self.track_display_scale}, "
            f"overlay_font_scale={self.overlay_font_scale}"
        )

    def resize_for_display(self, img, scale, target_width=0, target_height=0):
        """
        只放大显示画面，不改变识别逻辑使用的原始图像坐标。
        display_width/display_height 同时大于 0 时，优先固定显示尺寸。
        否则使用 display_scale 等比例放大。
        """
        if img is None:
            return img

        if target_width > 0 and target_height > 0:
            return cv.resize(
                img,
                (target_width, target_height),
                interpolation=cv.INTER_LINEAR,
            )

        if abs(scale - 1.0) > 0.01:
            return cv.resize(
                img,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv.INTER_LINEAR,
            )

        return img

    def update_fps(self):
        now = time.time()

        if self.last_frame_timestamp is None:
            self.last_frame_timestamp = now
            return

        dt = now - self.last_frame_timestamp
        self.last_frame_timestamp = now

        if dt <= 0.0001:
            return

        instant_fps = 1.0 / dt

        if self.fps <= 0.01:
            self.fps = instant_fps
        else:
            self.fps = 0.85 * self.fps + 0.15 * instant_fps

    def smooth_point(self, point):
        point = np.array(point, dtype=np.float32)

        if self.smoothed_tip is None:
            self.smoothed_tip = point
        else:
            self.smoothed_tip = (
                self.smooth_alpha * point
                + (1.0 - self.smooth_alpha) * self.smoothed_tip
            )

        return self.smoothed_tip.tolist()

    def reset_tracking(self):
        self.state = State.NULL
        self.points = []
        self.smoothed_tip = None
        self.start_count = 0
        self.tracking_start_timestamp = 0.0

    def start_tracking(self):
        self.state = State.TRACKING
        self.points = []
        self.smoothed_tip = None
        self.start_count = 0
        self.tracking_start_timestamp = time.time()
        self.no_finger_timestamp = time.time()

        self.get_logger().info("Start tracking finger trajectory.")

    def draw_overlay_text(self, img, text, org, bg_color=(0, 0, 0)):
        put_text_with_background(
            img,
            text,
            org,
            font_scale=self.overlay_font_scale,
            bg_color=bg_color,
            thickness=self.overlay_thickness,
            alpha=self.overlay_alpha,
            pad_x=4,
            pad_y=3,
        )

    def draw_status_overlay(self, img, gesture, hand_detected):
        """
        小面积状态提示：
        - 统一放在左上角
        - 字体小
        - 背景透明度低
        - 不再使用大面积提示框
        """
        x = 8
        y = 20
        line_gap = int(22 * max(self.overlay_font_scale / 0.45, 0.8))

        if self.show_fps:
            self.draw_overlay_text(
                img,
                f"FPS:{self.fps:.1f}",
                (x, y),
                bg_color=(50, 50, 50),
            )
            y += line_gap

        self.draw_overlay_text(
            img,
            f"G:{gesture}",
            (x, y),
            bg_color=(50, 50, 50),
        )
        y += line_gap

        if self.state == State.NULL:
            if not hand_detected:
                self.draw_overlay_text(
                    img,
                    "ONE finger to start",
                    (x, y),
                    bg_color=(0, 0, 100),
                )

            elif gesture == "one":
                progress = min(self.start_count, self.start_count_threshold)
                ratio = progress / max(self.start_count_threshold, 1)

                self.draw_overlay_text(
                    img,
                    f"Hold:{progress}/{self.start_count_threshold}",
                    (x, y),
                    bg_color=(0, 100, 100),
                )

                # 小型进度条，避免大面积遮挡
                bar_x = x
                bar_y = y + 6
                bar_w = 100
                bar_h = 6

                cv.rectangle(
                    img,
                    (bar_x, bar_y),
                    (bar_x + bar_w, bar_y + bar_h),
                    (80, 80, 80),
                    1,
                )

                cv.rectangle(
                    img,
                    (bar_x, bar_y),
                    (bar_x + int(bar_w * ratio), bar_y + bar_h),
                    (0, 220, 220),
                    -1,
                )

            else:
                self.draw_overlay_text(
                    img,
                    "Use ONE finger",
                    (x, y),
                    bg_color=(0, 0, 100),
                )

            if time.time() < self.last_result_until and self.last_result_text:
                put_center_text(
                    img,
                    self.last_result_text,
                    y_ratio=0.92,
                    font_scale=self.center_prompt_font_scale,
                    bg_color=(0, 80, 0),
                    thickness=1,
                )

        elif self.state == State.TRACKING:
            if hand_detected:
                self.draw_overlay_text(
                    img,
                    "Drawing, open palm to finish",
                    (x, y),
                    bg_color=(0, 100, 0),
                )
            else:
                remain = max(
                    self.lost_timeout - (time.time() - self.no_finger_timestamp),
                    0.0,
                )

                self.draw_overlay_text(
                    img,
                    f"Lost:{remain:.1f}s",
                    (x, y),
                    bg_color=(0, 0, 120),
                )

            if time.time() - self.tracking_start_timestamp <= self.start_prompt_duration:
                put_center_text(
                    img,
                    "START DRAWING",
                    y_ratio=0.92,
                    font_scale=self.center_prompt_font_scale,
                    bg_color=(0, 120, 0),
                    thickness=1,
                )

    def get_rgb_image_callback(self, msg: Image):
        self.update_fps()

        try:
            rgb_image = self.rgb_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS Image to OpenCV image: {e}")
            return

        rgb_image = cv.flip(rgb_image, 1)
        result_image = np.copy(rgb_image)

        gesture = "none"
        hand_detected = False
        index_finger_tip = None

        try:
            results = self.hand_detector.process(
                cv.cvtColor(rgb_image, cv.COLOR_BGR2RGB)
            )

            if results is not None and results.multi_hand_landmarks:
                hand_detected = True
                self.no_finger_timestamp = time.time()

                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        result_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS,
                    )

                    landmarks = get_hand_landmarks(
                        rgb_image,
                        hand_landmarks.landmark,
                    )

                    gesture = h_gesture(hand_angle(landmarks))
                    index_finger_tip = landmarks[8].tolist()

                if self.state == State.NULL:
                    if gesture == "one":
                        self.start_count += 1

                        if self.start_count >= self.start_count_threshold:
                            self.start_tracking()
                    else:
                        self.start_count = 0

                elif self.state == State.TRACKING:
                    if gesture == "five":
                        self.finish_tracking()

                    elif index_finger_tip is not None:
                        tip = self.smooth_point(index_finger_tip)

                        if len(self.points) > 0:
                            if distance(self.points[-1], tip) >= self.min_point_distance:
                                self.points.append(tip)
                        else:
                            self.points.append(tip)

                        cv.circle(
                            result_image,
                            tuple(np.array(tip).astype(np.int32)),
                            5,
                            (0, 255, 255),
                            -1,
                        )

            else:
                if self.state == State.NULL:
                    self.start_count = 0

                if self.state == State.TRACKING:
                    if time.time() - self.no_finger_timestamp > self.lost_timeout:
                        self.get_logger().warn("Finger lost timeout. Tracking canceled.")
                        self.reset_tracking()

            if self.state == State.TRACKING and len(self.points) > 1:
                draw_points(
                    result_image,
                    self.points,
                    thickness=self.track_line_thickness,
                    color=(255, 0, 0),
                )

        except BaseException as e:
            self.get_logger().warn(f"FingerTrajectory process warning: {e}")

        self.draw_status_overlay(result_image, gesture, hand_detected)

        display_image = self.resize_for_display(
            result_image,
            self.display_scale,
            self.display_width,
            self.display_height,
        )

        cv.imshow(self.window_name, display_image)

        key = cv.waitKey(1) & 0xFF

        if key == ord(" "):
            self.points = []
            self.smoothed_tip = None
            self.get_logger().info("Tracked points cleared.")

        if key == ord("q"):
            self.get_logger().info("Pressed q, shutting down FingerTrajectory node.")
            os._exit(0)  # pylint: disable=protected-access

        if time.time() > self.gc_stamp:
            self.gc_stamp = time.time() + 1
            gc.collect()

    def finish_tracking(self):
        points = list(self.points)
        self.reset_tracking()

        if len(points) < 3:
            self.last_result_text = "Too few points"
            self.last_result_until = time.time() + 2.0
            return

        track_img = get_track_img(points)

        contours = cv.findContours(
            track_img,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_NONE,
        )[-2]

        contour = get_area_max_contour(
            contours,
            self.contour_area_threshold,
        )

        if contour is None:
            self.last_result_text = "No valid contour"
            self.last_result_until = time.time() + 2.0
            return

        contour = contour[0]
        track_img = cv.fillPoly(track_img, [contour], 255)

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))

        for _ in range(self.morph_iterations):
            track_img = cv.erode(track_img, kernel)
            track_img = cv.dilate(track_img, kernel)

        contours = cv.findContours(
            track_img,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_NONE,
        )[-2]

        contour = get_area_max_contour(
            contours,
            self.contour_area_threshold,
        )

        if contour is None:
            self.last_result_text = "No contour after morph"
            self.last_result_until = time.time() + 2.0
            return

        contour = contour[0]

        h, w = track_img.shape[:2]
        track_img_show = np.full((h, w, 3), 0, dtype=np.uint8)

        track_img_show = cv.drawContours(
            track_img_show,
            [contour],
            -1,
            (0, 255, 0),
            2,
        )

        epsilon = self.approx_epsilon_ratio * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)

        track_img_show = cv.drawContours(
            track_img_show,
            [approx],
            -1,
            (0, 0, 255),
            2,
        )

        graph_name = "unknown"

        if len(approx) == 3:
            graph_name = "Triangle"
        elif len(approx) in (4, 5):
            graph_name = "Square"
        elif 5 < len(approx) < 10:
            graph_name = "Circle"
        elif len(approx) == 10:
            graph_name = "Star"

        cv.putText(
            track_img_show,
            graph_name,
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 0),
            1,
            cv.LINE_AA,
        )

        cv.putText(
            track_img_show,
            f"p:{len(points)} v:{len(approx)}",
            (10, 55),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv.LINE_AA,
        )

        track_display_image = self.resize_for_display(
            track_img_show,
            self.track_display_scale,
            self.track_display_width,
            self.track_display_height,
        )

        cv.imshow(self.track_window_name, track_display_image)

        self.last_result_text = f"Result: {graph_name}"
        self.last_result_until = time.time() + 2.5

        self.get_logger().info(
            f"Trajectory recognized as: {graph_name}, "
            f"points={len(points)}, "
            f"approx_vertices={len(approx)}"
        )

    def destroy_node(self):
        try:
            self.hand_detector.close()
        except Exception:
            pass

        try:
            cv.destroyAllWindows()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = FingerTrajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
