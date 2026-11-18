#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR code PTZ tracker.

识别二维码后，只调整云台角度，让二维码保持在画面中心。
不发布底盘速度，不解析二维码为运动命令。
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr

try:
    import pyzbar.pyzbar as pyzbar_module
except Exception:  # pylint: disable=broad-except
    pyzbar_module = None


DEFAULT_NODE_NAME = "qr_tracker_ptz_node"

STATE_IDENTIFY = "identify"
STATE_TRACKING = "tracking"
STATE_LOST = "lost"
STATE_PAUSED = "paused"

KEY_SPACE = 32
KEY_ESCAPE = 27
MIN_TIMER_RATE_HZ = 1.0
QR_CANDIDATE_COLOR = (0, 255, 0)
QR_PRIMARY_COLOR = (255, 0, 255)


@dataclass
class QrTarget:
    found: bool
    payload: str = ""
    rect: Tuple[int, int, int, int] = (0, 0, 0, 0)
    polygon: Tuple[Tuple[int, int], ...] = tuple()
    center: Tuple[float, float] = (0.0, 0.0)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


class QrControlPtzNode(Node):
    """二维码云台跟随节点。"""

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)

        self.image_lock = threading.Lock()
        self.target_lock = threading.Lock()
        self.latest_image_msg: Optional[Image] = None
        self.latest_image_seq = 0
        self.processed_image_seq = 0

        self.qr_detector = cv2.QRCodeDetector()
        self.latest_display_frame: Optional[np.ndarray] = None
        self.debug_window_ready = False
        self.running = True

        self.qr_target = QrTarget(False)
        self.last_target_time = 0.0
        self.image_width = 0
        self.image_height = 0
        self.measured_fps = 0.0
        self.fps_frame_count = 0
        self.fps_last_time = time.perf_counter()
        self.last_debug_log_time = 0.0
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

        self._declare_qr_tracker_parameters()
        self.load_parameters()

        self.current_horizontal_deg = clamp(
            self.initial_horizontal_deg,
            self.ptz_x_min,
            self.ptz_x_max,
        )
        self.current_pitch_deg = clamp(
            self.initial_pitch_deg,
            self.ptz_y_min,
            self.ptz_y_max,
        )
        self.control_enabled = self.auto_start_control
        self.state = STATE_TRACKING if self.control_enabled else STATE_IDENTIFY

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )
        self.ptz_publisher = self.create_publisher(Vector3, self.ptz_command_topic, 10)

        self.image_timer = self.create_timer(
            1.0 / max(self.image_processing_rate_hz, MIN_TIMER_RATE_HZ),
            self.process_latest_image,
        )
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, MIN_TIMER_RATE_HZ),
            self.update_ptz_tracking,
        )

        self.initial_publish_count_left = self.initial_publish_count
        self.initial_timer = None
        if self.publish_initial_ptz_command:
            self.initial_timer = self.create_timer(
                max(self.initial_publish_interval_sec, 0.02),
                self.publish_initial_ptz,
            )

        self.parameter_callback = self.add_on_set_parameters_callback(
            self.on_parameter_update
        )

        self.get_logger().info(
            "QR PTZ tracker started. image_topic=%s, ptz_command_topic=%s"
            % (self.image_topic, self.ptz_command_topic)
        )

    def _declare_qr_tracker_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "qr_tracker_ptz")

        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.45)
        self.declare_parameter("overlay_line_height", 18)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 20)
        self.declare_parameter("overlay_show_background", False)

        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("qr_detector_backend", "pyzbar")
        self.declare_parameter("qr_select_strategy", "largest")

        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)
        self.declare_parameter("auto_start_control", True)
        self.declare_parameter("lost_timeout", 0.5)

        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)

        self.declare_parameter("ptz_x_min", -90.0)
        self.declare_parameter("ptz_x_max", 90.0)
        self.declare_parameter("ptz_y_min", -90.0)
        self.declare_parameter("ptz_y_max", 20.0)
        self.declare_parameter("ptz_x_reverse", True)
        self.declare_parameter("ptz_y_reverse", True)
        self.declare_parameter("deadband_x", 18.0)
        self.declare_parameter("deadband_y", 18.0)
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("debug_log_interval_sec", 2.0)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def load_parameters(self) -> None:
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.ptz_command_topic = str(self.get_parameter("ptz_command_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.show_debug_window = as_bool(self.get_parameter("show_debug_window").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = as_bool(self.get_parameter("overlay_show_background").value)

        self.enable_horizontal_flip = as_bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = as_bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)
        self.qr_detector_backend = str(self.get_parameter("qr_detector_backend").value).lower()
        if self.qr_detector_backend not in {"pyzbar", "opencv", "auto"}:
            self.qr_detector_backend = "pyzbar"
        self.qr_select_strategy = str(self.get_parameter("qr_select_strategy").value).lower()
        if self.qr_select_strategy not in {"largest", "nearest_center", "first"}:
            self.qr_select_strategy = "largest"

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), MIN_TIMER_RATE_HZ)
        self.image_processing_rate_hz = max(
            float(self.get_parameter("image_processing_rate_hz").value),
            MIN_TIMER_RATE_HZ,
        )
        self.auto_start_control = as_bool(self.get_parameter("auto_start_control").value)
        self.lost_timeout = max(float(self.get_parameter("lost_timeout").value), 0.05)

        self.publish_initial_ptz_command = as_bool(self.get_parameter("publish_initial_ptz_command").value)
        self.initial_horizontal_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self.initial_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)

        self.ptz_x_min = float(self.get_parameter("ptz_x_min").value)
        self.ptz_x_max = float(self.get_parameter("ptz_x_max").value)
        self.ptz_y_min = float(self.get_parameter("ptz_y_min").value)
        self.ptz_y_max = float(self.get_parameter("ptz_y_max").value)
        self.ptz_x_reverse = as_bool(self.get_parameter("ptz_x_reverse").value)
        self.ptz_y_reverse = as_bool(self.get_parameter("ptz_y_reverse").value)
        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.pd_kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self.pd_kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self.ptz_step_smooth_alpha = clamp(
            float(self.get_parameter("ptz_step_smooth_alpha").value),
            0.0,
            1.0,
        )
        self.max_ptz_step = float(self.get_parameter("max_ptz_step").value)
        self.debug_log_interval_sec = max(float(self.get_parameter("debug_log_interval_sec").value), 0.1)
        self.enable_rqt_dynamic_parameters = as_bool(
            self.get_parameter("enable_rqt_dynamic_parameters").value
        )

    def image_callback(self, image_msg: Image) -> None:
        with self.image_lock:
            self.latest_image_msg = image_msg
            self.latest_image_seq += 1

    def publish_initial_ptz(self) -> None:
        if self.initial_publish_count_left <= 0:
            if self.initial_timer is not None:
                self.destroy_timer(self.initial_timer)
                self.initial_timer = None
            return
        self.publish_ptz_angle(self.current_horizontal_deg, self.current_pitch_deg)
        self.initial_publish_count_left -= 1

    def publish_ptz_angle(self, horizontal_deg: float, pitch_deg: float) -> None:
        msg = Vector3()
        msg.x = float(horizontal_deg)
        msg.y = float(pitch_deg)
        msg.z = 0.0
        self.ptz_publisher.publish(msg)

    def process_latest_image(self) -> None:
        with self.image_lock:
            if self.latest_image_msg is None:
                return
            image_msg = self.latest_image_msg
            image_seq = self.latest_image_seq

        if image_seq == self.processed_image_seq:
            self.update_debug_window()
            return
        self.processed_image_seq = image_seq

        frame = self.decode_image(image_msg)
        if frame is None:
            return
        frame = self.prepare_frame(frame)
        self.image_height, self.image_width = frame.shape[:2]

        candidates = self.detect_qr(frame)
        primary_index = self.select_target_index(candidates)
        primary_target = (
            candidates[primary_index]
            if primary_index is not None
            else QrTarget(False)
        )
        self.update_target(primary_target)

        display_frame = frame.copy()
        self.draw_center_cross(display_frame)
        self.draw_qr_targets(display_frame, candidates, primary_index)
        self.draw_overlay(display_frame, len(candidates))
        self.latest_display_frame = display_frame

        self.update_fps()
        self.update_debug_window()

    def decode_image(self, image_msg: Image) -> Optional[np.ndarray]:
        try:
            return image_to_bgr(image_msg)
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().warn("QR image decode failed: %s" % exception)
            return None

    def prepare_frame(self, frame: np.ndarray) -> np.ndarray:
        frame = cv2.resize(frame, (640, 480))
        if self.enable_horizontal_flip:
            frame = cv2.flip(frame, 1)
        if self.enable_vertical_flip:
            frame = cv2.flip(frame, 0)
        if self.rotate_image_code == "90_clockwise":
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code == "90_counterclockwise":
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code == "180":
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def detect_qr(self, frame: np.ndarray) -> List[QrTarget]:
        backend = self.qr_detector_backend
        if backend not in {"pyzbar", "opencv", "auto"}:
            backend = "pyzbar"

        candidates: List[QrTarget] = []
        if backend in {"pyzbar", "auto"}:
            candidates.extend(self.detect_qr_with_pyzbar(frame))
            if candidates:
                return candidates

        if backend == "pyzbar":
            return []

        candidates.extend(self.detect_qr_with_opencv(frame))
        return candidates

    def detect_qr_with_pyzbar(self, frame: np.ndarray) -> List[QrTarget]:
        if pyzbar_module is None:
            self.throttled_debug_log(
                "pyzbar is unavailable. Install libzbar0/python3-pyzbar or set qr_detector_backend:=opencv."
            )
            return []

        try:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            decoded_barcodes = pyzbar_module.decode(gray)
        except Exception as exception:  # pylint: disable=broad-except
            self.throttled_debug_log("pyzbar QR detection failed: %s" % exception)
            return []

        targets: List[QrTarget] = []
        for decoded_barcode in decoded_barcodes:
            payload = decoded_barcode.data.decode("utf-8", errors="ignore").strip()
            if not payload:
                continue
            x, y, width, height = decoded_barcode.rect
            polygon = tuple((int(point.x), int(point.y)) for point in decoded_barcode.polygon)
            if not polygon:
                polygon = (
                    (int(x), int(y)),
                    (int(x + width), int(y)),
                    (int(x + width), int(y + height)),
                    (int(x), int(y + height)),
                )
            targets.append(
                QrTarget(
                    True,
                    payload,
                    (int(x), int(y), int(width), int(height)),
                    polygon,
                    (float(x + width * 0.5), float(y + height * 0.5)),
                )
            )
        return targets

    def detect_qr_with_opencv(self, frame: np.ndarray) -> List[QrTarget]:
        targets: List[QrTarget] = []
        try:
            if hasattr(self.qr_detector, "detectAndDecodeMulti"):
                ok, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
                if ok and points is not None:
                    for payload, point_array in zip(decoded_info, points):
                        payload = str(payload).strip()
                        if payload:
                            targets.append(self.target_from_points(payload, point_array))
            if targets:
                return targets

            payload, points, _ = self.qr_detector.detectAndDecode(frame)
            payload = str(payload).strip()
            if payload and points is not None:
                targets.append(self.target_from_points(payload, points))
        except Exception as exception:  # pylint: disable=broad-except
            self.throttled_debug_log("OpenCV QR detection failed: %s" % exception)
        return targets

    def select_target_index(self, candidates: List[QrTarget]) -> Optional[int]:
        if not candidates:
            return None
        if self.qr_select_strategy == "nearest_center":
            cx = self.image_width * 0.5
            cy = self.image_height * 0.5
            return min(
                range(len(candidates)),
                key=lambda index: math.hypot(
                    candidates[index].center[0] - cx,
                    candidates[index].center[1] - cy,
                ),
            )
        if self.qr_select_strategy == "first":
            return 0
        return max(
            range(len(candidates)),
            key=lambda index: candidates[index].rect[2] * candidates[index].rect[3],
        )

    def target_from_points(self, payload: str, point_array: np.ndarray) -> QrTarget:
        points = np.array(point_array, dtype=np.float32).reshape(-1, 2)
        polygon = tuple((int(point[0]), int(point[1])) for point in points)
        x, y, width, height = cv2.boundingRect(np.array(polygon, dtype=np.int32))
        center = (float(np.mean(points[:, 0])), float(np.mean(points[:, 1])))
        return QrTarget(True, payload, (int(x), int(y), int(width), int(height)), polygon, center)

    def throttled_debug_log(self, text: str) -> None:
        now = time.perf_counter()
        if now - self.last_debug_log_time >= self.debug_log_interval_sec:
            self.get_logger().warn(text)
            self.last_debug_log_time = now

    def update_target(self, target: QrTarget) -> None:
        now = time.perf_counter()
        with self.target_lock:
            if target.found:
                self.qr_target = target
                self.last_target_time = now
                self.state = STATE_TRACKING if self.control_enabled else STATE_IDENTIFY
            elif now - self.last_target_time > self.lost_timeout:
                self.qr_target = QrTarget(False)
                self.reset_pd_state()
                if self.control_enabled:
                    self.state = STATE_LOST

    def update_ptz_tracking(self) -> None:
        if not self.running or not self.control_enabled:
            return

        with self.target_lock:
            target = self.qr_target
            target_age = time.perf_counter() - self.last_target_time

        if not target.found or target_age > self.lost_timeout or self.image_width <= 0 or self.image_height <= 0:
            self.state = STATE_LOST
            self.reset_pd_state()
            return

        now = time.perf_counter()
        dt = now - self.previous_control_time if self.previous_control_time > 0.0 else 1.0 / self.control_rate_hz
        dt = max(dt, 1.0e-3)
        image_center_x = self.image_width * 0.5
        image_center_y = self.image_height * 0.5
        error_x = target.center[0] - image_center_x
        error_y = target.center[1] - image_center_y
        if abs(error_x) < self.deadband_x:
            error_x = 0.0
        if abs(error_y) < self.deadband_y:
            error_y = 0.0

        normalized_error_x = error_x / image_center_x
        normalized_error_y = error_y / image_center_y
        derivative_x = (normalized_error_x - self.previous_error_x) / dt
        derivative_y = (normalized_error_y - self.previous_error_y) / dt
        raw_step_x = (self.pd_kp * normalized_error_x + self.pd_kd * derivative_x) * self.max_ptz_step
        raw_step_y = (self.pd_kp * normalized_error_y + self.pd_kd * derivative_y) * self.max_ptz_step
        raw_step_x = clamp(raw_step_x, -self.max_ptz_step, self.max_ptz_step)
        raw_step_y = clamp(raw_step_y, -self.max_ptz_step, self.max_ptz_step)
        alpha = self.ptz_step_smooth_alpha
        step_x = (1.0 - alpha) * self.last_step_x + alpha * raw_step_x
        step_y = (1.0 - alpha) * self.last_step_y + alpha * raw_step_y

        self.previous_error_x = normalized_error_x
        self.previous_error_y = normalized_error_y
        self.previous_control_time = now
        self.last_step_x = step_x
        self.last_step_y = step_y

        self.current_horizontal_deg += step_x if self.ptz_x_reverse else -step_x
        self.current_pitch_deg += -step_y if self.ptz_y_reverse else step_y
        self.current_horizontal_deg = clamp(self.current_horizontal_deg, self.ptz_x_min, self.ptz_x_max)
        self.current_pitch_deg = clamp(self.current_pitch_deg, self.ptz_y_min, self.ptz_y_max)
        self.publish_ptz_angle(self.current_horizontal_deg, self.current_pitch_deg)

    def reset_pd_state(self) -> None:
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

    def draw_center_cross(self, frame: np.ndarray) -> None:
        cx = int(frame.shape[1] * 0.5)
        cy = int(frame.shape[0] * 0.5)
        cv2.line(frame, (cx - 18, cy), (cx + 18, cy), (255, 255, 0), 2)
        cv2.line(frame, (cx, cy - 18), (cx, cy + 18), (255, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 4, (255, 255, 0), -1)

    def draw_qr_targets(
        self,
        frame: np.ndarray,
        candidates: List[QrTarget],
        primary_index: Optional[int],
    ) -> None:
        for index, target in enumerate(candidates):
            self.draw_qr_annotation(frame, target, index, False)

        if primary_index is not None and 0 <= primary_index < len(candidates):
            self.draw_qr_annotation(
                frame,
                candidates[primary_index],
                primary_index,
                True,
            )

    def draw_qr_annotation(
        self,
        frame: np.ndarray,
        target: QrTarget,
        index: int,
        is_primary: bool,
    ) -> None:
        if not target.found:
            return
        x, y, width, height = target.rect
        color = QR_PRIMARY_COLOR if is_primary else QR_CANDIDATE_COLOR
        thickness = 3 if is_primary else 2
        if target.polygon:
            polygon = np.array(target.polygon, dtype=np.int32).reshape(
                (-1, 1, 2)
            )
            cv2.polylines(frame, [polygon], True, color, thickness)
        else:
            cv2.rectangle(
                frame,
                (x, y),
                (x + width, y + height),
                color,
                thickness,
            )
        cv2.circle(
            frame,
            (int(target.center[0]), int(target.center[1])),
            5,
            color,
            -1,
        )

        payload = (
            target.payload[:32] + "..."
            if len(target.payload) > 32
            else target.payload
        )
        prefix = "PRIMARY " if is_primary else ""
        label = "%sQR[%d]: %s" % (prefix, index + 1, payload)
        (label_width, _), _ = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            thickness,
        )
        text_x = max(0, min(x, frame.shape[1] - label_width - 1))
        text_y = max(22, y - 8)
        cv2.putText(
            frame,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            thickness,
            cv2.LINE_AA,
        )

    def draw_overlay(self, frame: np.ndarray, qr_count: int) -> None:
        with self.target_lock:
            payload = self.qr_target.payload if self.qr_target.found else "None"
            target_age = (
                time.perf_counter() - self.last_target_time
                if self.last_target_time > 0.0
                else 999.0
            )

        payload = payload[:36] + "..." if len(payload) > 36 else payload
        lines = [
            "State: %s | FPS: %.1f" % (self.state, self.measured_fps),
            "QR count: %d | Primary: %s | age: %.2fs"
            % (qr_count, payload, target_age),
            "PTZ: x=%.1f y=%.1f step=%.1f" % (
                self.current_horizontal_deg,
                self.current_pitch_deg,
                self.max_ptz_step,
            ),
            "PD: kp=%.2f kd=%.2f smooth=%.2f" % (
                self.pd_kp,
                self.pd_kd,
                self.ptz_step_smooth_alpha,
            ),
            "Keys: SPACE track/pause, i identify, r reset, q quit",
        ]
        if self.overlay_show_background:
            height = self.overlay_line_height * len(lines) + 8
            cv2.rectangle(frame, (0, 0), (min(frame.shape[1], 760), height), (0, 0, 0), -1)
        for index, text in enumerate(lines):
            origin = (
                self.overlay_margin_x,
                self.overlay_margin_y + index * self.overlay_line_height,
            )
            cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, self.overlay_font_scale, (0, 255, 255), 1, cv2.LINE_AA)

    def update_debug_window(self) -> None:
        if not self.show_debug_window:
            if self.debug_window_ready:
                cv2.destroyWindow(self.window_name)
                self.debug_window_ready = False
            return
        if self.latest_display_frame is None:
            return
        if not self.debug_window_ready:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
            self.debug_window_ready = True
        cv2.imshow(self.window_name, self.latest_display_frame)
        self.handle_key(cv2.waitKey(1) & 0xFF)

    def handle_key(self, key: int) -> None:
        if key in (255, -1):
            return
        if key in (ord("q"), ord("Q"), KEY_ESCAPE):
            self.stop_from_keyboard()
        elif key in (ord("r"), ord("R")):
            self.reset_target()
        elif key in (ord("i"), ord("I")):
            self.control_enabled = False
            self.state = STATE_IDENTIFY
        elif key == KEY_SPACE:
            self.control_enabled = not self.control_enabled
            self.state = STATE_TRACKING if self.control_enabled else STATE_PAUSED

    def reset_target(self) -> None:
        with self.target_lock:
            self.qr_target = QrTarget(False)
            self.last_target_time = 0.0
        self.reset_pd_state()
        self.control_enabled = False
        self.state = STATE_IDENTIFY

    def update_fps(self) -> None:
        self.fps_frame_count += 1
        now = time.perf_counter()
        elapsed = now - self.fps_last_time
        if elapsed >= 1.0:
            self.measured_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_last_time = now

    def on_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        if not self.enable_rqt_dynamic_parameters:
            return SetParametersResult(successful=False, reason="Dynamic parameters are disabled.")
        try:
            recreate_image_timer = False
            recreate_control_timer = False
            for parameter in parameters:
                name = parameter.name
                value = parameter.value
                if name in {"image_topic", "ptz_command_topic", "window_name"}:
                    return SetParametersResult(successful=False, reason="'%s' requires node restart." % name)
                if name == "show_debug_window":
                    self.show_debug_window = as_bool(value)
                elif name in {"window_width", "window_height", "overlay_line_height", "overlay_margin_x", "overlay_margin_y"}:
                    setattr(self, name, int(value))
                elif name == "overlay_font_scale":
                    self.overlay_font_scale = float(value)
                elif name in {"overlay_show_background", "enable_horizontal_flip", "enable_vertical_flip", "ptz_x_reverse", "ptz_y_reverse"}:
                    setattr(self, name, as_bool(value))
                elif name in {"rotate_image_code", "qr_select_strategy", "qr_detector_backend"}:
                    setattr(self, name, str(value).lower())
                elif name == "control_rate_hz":
                    self.control_rate_hz = max(float(value), MIN_TIMER_RATE_HZ)
                    recreate_control_timer = True
                elif name == "image_processing_rate_hz":
                    self.image_processing_rate_hz = max(float(value), MIN_TIMER_RATE_HZ)
                    recreate_image_timer = True
                elif name == "auto_start_control":
                    self.control_enabled = as_bool(value)
                    self.auto_start_control = self.control_enabled
                    self.state = STATE_TRACKING if self.control_enabled else STATE_IDENTIFY
                elif name in {
                    "lost_timeout",
                    "initial_horizontal_deg",
                    "initial_pitch_deg",
                    "initial_publish_interval_sec",
                    "ptz_x_min",
                    "ptz_x_max",
                    "ptz_y_min",
                    "ptz_y_max",
                    "deadband_x",
                    "deadband_y",
                    "pd_kp",
                    "pd_kd",
                    "ptz_step_smooth_alpha",
                    "max_ptz_step",
                    "debug_log_interval_sec",
                }:
                    if name == "ptz_step_smooth_alpha":
                        setattr(self, name, clamp(float(value), 0.0, 1.0))
                    elif name in {"pd_kp", "pd_kd"}:
                        setattr(self, name, max(float(value), 0.0))
                    else:
                        setattr(self, name, float(value))
                elif name == "initial_publish_count":
                    self.initial_publish_count = int(value)
                elif name in {"publish_initial_ptz_command", "enable_rqt_dynamic_parameters"}:
                    setattr(self, name, as_bool(value))

            self.current_horizontal_deg = clamp(self.current_horizontal_deg, self.ptz_x_min, self.ptz_x_max)
            self.current_pitch_deg = clamp(self.current_pitch_deg, self.ptz_y_min, self.ptz_y_max)
            if recreate_control_timer:
                self.destroy_timer(self.control_timer)
                self.control_timer = self.create_timer(1.0 / self.control_rate_hz, self.update_ptz_tracking)
            if recreate_image_timer:
                self.destroy_timer(self.image_timer)
                self.image_timer = self.create_timer(1.0 / self.image_processing_rate_hz, self.process_latest_image)
            if not self.show_debug_window and self.debug_window_ready:
                cv2.destroyWindow(self.window_name)
                self.debug_window_ready = False
            return SetParametersResult(successful=True)
        except Exception as exception:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception))

    def stop_from_keyboard(self) -> None:
        self.running = False
        os._exit(0)  # pylint: disable=protected-access

    def destroy_node(self) -> bool:
        self.running = False
        if self.show_debug_window or self.debug_window_ready:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = QrControlPtzNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
