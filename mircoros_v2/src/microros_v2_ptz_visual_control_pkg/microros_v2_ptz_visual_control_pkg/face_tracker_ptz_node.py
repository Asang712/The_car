#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
face_tracker_ptz_node_os_exit_quit.py

ROS2 Humble 人脸识别云台控制节点。

本版本针对“按 q 退出卡死”做了结构调整：
1. 图像订阅回调只解码并缓存最新图像。
2. 云台控制仍然使用 ROS2 timer 固定频率发布。
3. OpenCV imshow/waitKey 不再放在 ROS2 timer 回调里，而是放在 main() 主循环执行。
4. q/Q/ESC 不调用 rclpy.shutdown()。
5. q/Q/ESC 不调用 cv2.destroyAllWindows()。
6. q/Q/ESC 直接 os._exit(0) 结束当前节点进程。
7. main 使用 spin_once 循环，不使用阻塞式 rclpy.spin()。
8. 不通过 cv2.getWindowProperty 自动判断窗口关闭，避免启动时误退出。
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr


VISUAL_CONTROL_PACKAGE_NAME = "microros_v2_visual_control_pkg"
DEFAULT_NODE_NAME = "face_tracker_ptz_node"

FACE_TRACKING_STATE_SEARCHING = "searching"
FACE_TRACKING_STATE_TRACKING = "tracking"
FACE_TRACKING_STATE_LOST = "lost"
FACE_TRACKING_STATE_IDENTIFY = "identify"
FACE_TRACKING_STATE_PAUSED = "paused"

KEY_CODE_SPACE = 32
KEY_CODE_ESCAPE = 27

MINIMUM_TIMER_RATE_HZ = 1.0
MINIMUM_ALPHA_VALUE = 0.01
MAXIMUM_ALPHA_VALUE = 1.0
MINIMUM_TARGET_LOST_TIMEOUT_SEC = 0.05
MINIMUM_FACE_DETECTION_SCALE = 0.10
MAXIMUM_FACE_DETECTION_SCALE = 1.00

DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "window_width",
    "window_height",
    "overlay_font_scale",
    "overlay_line_height",
    "overlay_margin_x",
    "overlay_margin_y",
    "overlay_show_background",
    "img_flip",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "horizontal_min_deg",
    "horizontal_max_deg",
    "pitch_min_deg",
    "pitch_max_deg",
    "deadband_x",
    "deadband_y",
    "pd_kp",
    "pd_kd",
    "ptz_step_smooth_alpha",
    "max_ptz_step",
    "reverse_horizontal_direction",
    "reverse_pitch_direction",
    "target_smooth_alpha",
    "ptz_smooth_alpha",
    "control_rate_hz",
    "image_processing_rate_hz",
    "face_cascade_file",
    "face_detection_scale_factor",
    "face_min_neighbors",
    "face_min_width",
    "face_min_height",
    "face_detection_scale",
    "face_select_strategy",
    "enable_gray_equalize",
    "target_lost_timeout_sec",
    "lost_target_stop",
    "auto_start_tracking",
    "auto_reacquire_face",
    "publish_initial_ptz_command",
    "initial_publish_count",
    "initial_publish_interval_sec",
    "debug_log_interval_sec",
    "print_ptz_angle",
    "print_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
    "enable_rqt_dynamic_parameters",
    "hard_exit_on_quit",
})

RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "ptz_command_topic",
    "window_name",
})


@dataclass
class FaceDetectionResult:
    """人脸识别结果 / Face detection result."""

    is_face_detected: bool
    face_center_x: float = 0.0
    face_center_y: float = 0.0
    face_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)
    all_face_rectangles: Tuple[Tuple[int, int, int, int], ...] = ()


@dataclass
class FaceTargetStateSnapshot:
    """人脸目标状态快照 / Face target state snapshot."""

    target_is_valid: bool
    target_age_sec: float
    filtered_target_center_x: Optional[float]
    filtered_target_center_y: Optional[float]
    image_width: int
    image_height: int


def clamp_numeric_value(current_value: float, minimum_value: float, maximum_value: float) -> float:
    """数值限幅 / Clamp numeric value."""
    return max(float(minimum_value), min(float(maximum_value), float(current_value)))


def convert_value_to_boolean(input_value: Any, default_boolean_value: bool) -> bool:
    """安全转换布尔参数 / Safely convert value to boolean."""
    if isinstance(input_value, bool):
        return input_value

    if isinstance(input_value, str):
        normalized_text_value = input_value.strip().lower()
        if normalized_text_value in ("true", "1", "yes", "y", "on"):
            return True
        if normalized_text_value in ("false", "0", "no", "n", "off"):
            return False

    if isinstance(input_value, (int, float)):
        return bool(input_value)

    return default_boolean_value


def get_default_face_cascade_file() -> str:
    """获取默认人脸级联分类器路径 / Get default face cascade classifier file path."""
    try:
        package_share_directory = get_package_share_directory(VISUAL_CONTROL_PACKAGE_NAME)
        return os.path.join(package_share_directory, "config", "haarcascade_frontalface_default.xml")
    except Exception:  # pylint: disable=broad-except
        return os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")


class FaceTrackerPtzNode(Node):
    """ROS2 人脸识别云台控制节点 / ROS2 face tracking PTZ control node."""

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)

        self.latest_image_lock = threading.Lock()
        self.target_state_lock = threading.Lock()

        self.latest_image_frame: Optional[np.ndarray] = None
        self.latest_image_timestamp_sec = 0.0
        self.last_processed_image_timestamp_sec = 0.0

        self.face_cascade_classifier: Optional[cv2.CascadeClassifier] = None

        self.tracking_state = FACE_TRACKING_STATE_SEARCHING

        self.horizontal_angle_deg = 0.0
        self.pitch_angle_deg = 0.0
        self.last_control_time_sec = time.monotonic()
        self.last_debug_log_time_sec = 0.0
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

        self.is_shutdown_requested = False
        self.cleanup_has_run = False

        self.raw_face_rectangle = (0, 0, 0, 0)
        self.all_raw_face_rectangles: Tuple[Tuple[int, int, int, int], ...] = ()

        self.raw_target_center_x: Optional[float] = None
        self.raw_target_center_y: Optional[float] = None
        self.filtered_target_center_x: Optional[float] = None
        self.filtered_target_center_y: Optional[float] = None

        self.target_is_valid = False
        self.last_valid_target_time_sec = time.monotonic()

        self.latest_tracking_image_width = 0
        self.latest_tracking_image_height = 0

        self.last_detection_result_for_display = FaceDetectionResult(False)

        self.remaining_initial_publish_count = 0
        self.initial_publish_timer = None
        self.image_processing_timer = None
        self.ptz_control_timer = None

        self.last_main_loop_image_process_time_sec = 0.0

        self.previous_header_stamp_sec = 0.0
        self.header_based_frame_rate = 0.0
        self.receive_based_frame_rate = 0.0
        self.previous_receive_time_sec = 0.0
        self.image_stamp_delay_ms = 0.0

        self.last_ptz_print_time_sec = 0.0

        self._declare_ros_parameters()
        self._load_ros_parameters()
        self._reload_face_cascade_classifier()

        self.horizontal_angle_deg = self.initial_horizontal_deg
        self.pitch_angle_deg = self.initial_pitch_deg

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        self.ptz_angle_publisher = self.create_publisher(Vector3, self.ptz_command_topic_name, 10)

        # 图像处理不再使用 ROS2 timer，改为 main() 主循环调用 process_latest_image_from_main_loop()。
        self._recreate_image_processing_timer(False)

        # 云台控制仍然使用 ROS2 timer。
        self._recreate_ptz_control_timer(False)

        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        self._initialize_debug_window()

        if self.publish_initial_ptz_command:
            self._start_initial_ptz_publish_timer()

        self.get_logger().info(
            "Face tracker PTZ node started. "
            "SPACE: track/pause, i: identify only, r: reset target, q/ESC: quit. "
            f"image_topic={self.image_topic_name}, ptz_topic={self.ptz_command_topic_name}, "
            f"face_cascade={self.face_cascade_file}, control_rate={self.control_rate_hz:.1f}Hz, "
            f"image_processing_rate={self.image_processing_rate_hz:.1f}Hz"
        )

    def _declare_ros_parameters(self) -> None:
        """声明 ROS 参数 / Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "face_tracker_ptz")

        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.45)
        self.declare_parameter("overlay_line_height", 18)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 18)
        self.declare_parameter("overlay_show_background", True)

        self.declare_parameter("img_flip", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        self.declare_parameter("face_cascade_file", get_default_face_cascade_file())
        self.declare_parameter("face_detection_scale_factor", 1.1)
        self.declare_parameter("face_min_neighbors", 5)
        self.declare_parameter("face_min_width", 80)
        self.declare_parameter("face_min_height", 80)
        self.declare_parameter("face_detection_scale", 0.75)
        self.declare_parameter("face_select_strategy", "largest")
        self.declare_parameter("enable_gray_equalize", True)

        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)

        self.declare_parameter("reverse_horizontal_direction", False)
        self.declare_parameter("reverse_pitch_direction", True)
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("target_smooth_alpha", 0.55)
        self.declare_parameter("ptz_smooth_alpha", 0.75)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)

        self.declare_parameter("target_lost_timeout_sec", 1.0)
        self.declare_parameter("lost_target_stop", True)
        self.declare_parameter("auto_start_tracking", True)
        self.declare_parameter("auto_reacquire_face", True)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("debug_log_interval_sec", 1.0)
        self.declare_parameter("print_ptz_angle", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)
        self.declare_parameter("hard_exit_on_quit", True)

    def _load_ros_parameters(self) -> None:
        """读取 ROS 参数 / Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
        self.ptz_command_topic_name = str(self.get_parameter("ptz_command_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.show_debug_window = convert_value_to_boolean(
            self.get_parameter("show_debug_window").value,
            True,
        )
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = convert_value_to_boolean(
            self.get_parameter("overlay_show_background").value,
            True,
        )

        self.img_flip = convert_value_to_boolean(self.get_parameter("img_flip").value, False)
        self.enable_horizontal_flip = convert_value_to_boolean(
            self.get_parameter("enable_horizontal_flip").value,
            False,
        )
        self.enable_vertical_flip = convert_value_to_boolean(
            self.get_parameter("enable_vertical_flip").value,
            False,
        )
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value).lower()

        self.face_cascade_file = str(self.get_parameter("face_cascade_file").value)
        self.face_detection_scale_factor = float(self.get_parameter("face_detection_scale_factor").value)
        self.face_min_neighbors = int(self.get_parameter("face_min_neighbors").value)
        self.face_min_width = int(self.get_parameter("face_min_width").value)
        self.face_min_height = int(self.get_parameter("face_min_height").value)
        self.face_detection_scale = clamp_numeric_value(
            float(self.get_parameter("face_detection_scale").value),
            MINIMUM_FACE_DETECTION_SCALE,
            MAXIMUM_FACE_DETECTION_SCALE,
        )
        self.face_select_strategy = str(self.get_parameter("face_select_strategy").value).lower()
        self.enable_gray_equalize = convert_value_to_boolean(
            self.get_parameter("enable_gray_equalize").value,
            True,
        )

        self.horizontal_min_deg = float(self.get_parameter("horizontal_min_deg").value)
        self.horizontal_max_deg = float(self.get_parameter("horizontal_max_deg").value)
        self.pitch_min_deg = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max_deg = float(self.get_parameter("pitch_max_deg").value)
        self.initial_horizontal_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self.initial_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)

        self.reverse_horizontal_direction = convert_value_to_boolean(
            self.get_parameter("reverse_horizontal_direction").value,
            False,
        )
        self.reverse_pitch_direction = convert_value_to_boolean(
            self.get_parameter("reverse_pitch_direction").value,
            True,
        )
        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.deadband_y = float(self.get_parameter("deadband_y").value)
        self.pd_kp = max(float(self.get_parameter("pd_kp").value), 0.0)
        self.pd_kd = max(float(self.get_parameter("pd_kd").value), 0.0)
        self.ptz_step_smooth_alpha = clamp_numeric_value(
            float(self.get_parameter("ptz_step_smooth_alpha").value),
            0.0,
            1.0,
        )
        self.max_ptz_step = float(self.get_parameter("max_ptz_step").value)
        self.target_smooth_alpha = clamp_numeric_value(
            float(self.get_parameter("target_smooth_alpha").value),
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
        )
        self.ptz_smooth_alpha = clamp_numeric_value(
            float(self.get_parameter("ptz_smooth_alpha").value),
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
        )
        self.control_rate_hz = max(
            float(self.get_parameter("control_rate_hz").value),
            MINIMUM_TIMER_RATE_HZ,
        )
        self.image_processing_rate_hz = max(
            float(self.get_parameter("image_processing_rate_hz").value),
            MINIMUM_TIMER_RATE_HZ,
        )

        self.target_lost_timeout_sec = max(
            float(self.get_parameter("target_lost_timeout_sec").value),
            MINIMUM_TARGET_LOST_TIMEOUT_SEC,
        )
        self.lost_target_stop = convert_value_to_boolean(
            self.get_parameter("lost_target_stop").value,
            True,
        )
        self.auto_start_tracking = convert_value_to_boolean(
            self.get_parameter("auto_start_tracking").value,
            True,
        )
        self.auto_reacquire_face = convert_value_to_boolean(
            self.get_parameter("auto_reacquire_face").value,
            True,
        )
        self.publish_initial_ptz_command = convert_value_to_boolean(
            self.get_parameter("publish_initial_ptz_command").value,
            True,
        )
        self.initial_publish_count = max(int(self.get_parameter("initial_publish_count").value), 0)
        self.initial_publish_interval_sec = max(
            float(self.get_parameter("initial_publish_interval_sec").value),
            0.02,
        )
        self.debug_log_interval_sec = float(self.get_parameter("debug_log_interval_sec").value)
        self.print_ptz_angle = convert_value_to_boolean(
            self.get_parameter("print_ptz_angle").value,
            False,
        )
        self.print_interval_sec = max(float(self.get_parameter("print_interval_sec").value), 0.05)
        self.use_header_stamp_for_fps = convert_value_to_boolean(
            self.get_parameter("use_header_stamp_for_fps").value,
            True,
        )
        self.show_image_stamp_delay = convert_value_to_boolean(
            self.get_parameter("show_image_stamp_delay").value,
            True,
        )
        self.enable_rqt_dynamic_parameters = convert_value_to_boolean(
            self.get_parameter("enable_rqt_dynamic_parameters").value,
            True,
        )
        self.hard_exit_on_quit = convert_value_to_boolean(
            self.get_parameter("hard_exit_on_quit").value,
            True,
        )

    def _reload_face_cascade_classifier(self) -> bool:
        """重新加载人脸级联分类器 / Reload face cascade classifier."""
        resolved_face_cascade_file = os.path.expanduser(self.face_cascade_file)
        face_cascade_classifier = cv2.CascadeClassifier(resolved_face_cascade_file)

        if face_cascade_classifier.empty():
            self.get_logger().error(f"Failed to load face cascade file: {resolved_face_cascade_file}")
            return False

        self.face_cascade_file = resolved_face_cascade_file
        self.face_cascade_classifier = face_cascade_classifier
        self.get_logger().info(f"Face cascade loaded: {self.face_cascade_file}")
        return True

    def _initialize_debug_window(self) -> None:
        """初始化 OpenCV 调试窗口 / Initialize OpenCV debug window."""
        if not self.show_debug_window:
            return

        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Initialize debug window failed: {exception_error}")

    def _update_debug_window(self) -> None:
        """更新 OpenCV 调试窗口 / Update OpenCV debug window."""
        try:
            if not self.show_debug_window:
                cv2.destroyWindow(self.window_name)
                return

            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Update debug window failed: {exception_error}")

    def _start_initial_ptz_publish_timer(self) -> None:
        """启动初始云台角度发布定时器 / Start initial PTZ angle publish timer."""
        self.remaining_initial_publish_count = self.initial_publish_count
        if self.remaining_initial_publish_count <= 0:
            return

        self.initial_publish_timer = self.create_timer(
            self.initial_publish_interval_sec,
            self._initial_ptz_publish_timer_callback,
        )

    def _initial_ptz_publish_timer_callback(self) -> None:
        """初始云台角度发布回调 / Initial PTZ angle publish callback."""
        if self.is_shutdown_requested:
            return

        self.publish_ptz_angle_command()
        self.remaining_initial_publish_count -= 1

        if self.remaining_initial_publish_count <= 0 and self.initial_publish_timer is not None:
            try:
                self.initial_publish_timer.cancel()
            except Exception:  # pylint: disable=broad-except
                pass

    def _recreate_image_processing_timer(self, should_log_update: bool) -> None:
        """
        重建图像处理定时器 / Recreate image processing timer.

        本版本禁用 image_processing_timer。
        OpenCV imshow/waitKey 必须在 main() 主循环执行，避免在 ROS2 timer 回调里退出卡死。
        """
        if self.image_processing_timer is not None:
            try:
                self.destroy_timer(self.image_processing_timer)
            except Exception as exception_error:  # pylint: disable=broad-except
                self.get_logger().warn(f"Destroy old image processing timer failed: {exception_error}")
            self.image_processing_timer = None

        if should_log_update:
            self.get_logger().info(
                "Image processing timer disabled. "
                f"Main loop processing rate={self.image_processing_rate_hz:.2f}Hz"
            )

    def _recreate_ptz_control_timer(self, should_log_update: bool) -> None:
        """重建云台控制定时器 / Recreate PTZ control timer."""
        if self.ptz_control_timer is not None:
            try:
                self.destroy_timer(self.ptz_control_timer)
            except Exception as exception_error:  # pylint: disable=broad-except
                self.get_logger().warn(f"Destroy old PTZ timer failed: {exception_error}")
            self.ptz_control_timer = None

        ptz_control_timer_period_sec = 1.0 / max(self.control_rate_hz, MINIMUM_TIMER_RATE_HZ)
        self.ptz_control_timer = self.create_timer(
            ptz_control_timer_period_sec,
            self.ptz_control_timer_callback,
        )

        if should_log_update:
            self.get_logger().info(f"PTZ control timer updated: rate={self.control_rate_hz:.2f}Hz")

    def image_callback(self, image_msg: Image) -> None:
        """接收并转换图像 / Receive and convert image."""
        if self.is_shutdown_requested:
            return

        try:
            decoded_image_frame = image_to_bgr(image_msg)

            decoded_image_frame = self._apply_image_orientation(decoded_image_frame)
            self._update_image_timing_statistics(image_msg)

            with self.latest_image_lock:
                self.latest_image_frame = decoded_image_frame
                self.latest_image_timestamp_sec = time.monotonic()

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def _apply_image_orientation(self, image_frame: np.ndarray) -> np.ndarray:
        """应用图像方向修正 / Apply image orientation correction."""
        corrected_image_frame = image_frame

        if self.img_flip or self.enable_horizontal_flip:
            corrected_image_frame = cv2.flip(corrected_image_frame, 1)

        if self.enable_vertical_flip:
            corrected_image_frame = cv2.flip(corrected_image_frame, 0)

        if self.rotate_image_code == "90_clockwise":
            corrected_image_frame = cv2.rotate(corrected_image_frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code == "90_counterclockwise":
            corrected_image_frame = cv2.rotate(corrected_image_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code == "180":
            corrected_image_frame = cv2.rotate(corrected_image_frame, cv2.ROTATE_180)

        return corrected_image_frame

    def _update_image_timing_statistics(self, image_msg: Image) -> None:
        """更新图像帧率和延迟统计 / Update image FPS and delay statistics."""
        current_receive_time_sec = time.monotonic()

        if self.previous_receive_time_sec > 0.0:
            receive_interval_sec = current_receive_time_sec - self.previous_receive_time_sec
            if receive_interval_sec > 0.0:
                self.receive_based_frame_rate = 1.0 / receive_interval_sec

        self.previous_receive_time_sec = current_receive_time_sec

        header_stamp_sec = (
            float(image_msg.header.stamp.sec)
            + float(image_msg.header.stamp.nanosec) * 1e-9
        )

        if header_stamp_sec > 0.0:
            if self.previous_header_stamp_sec > 0.0:
                header_interval_sec = header_stamp_sec - self.previous_header_stamp_sec
                if header_interval_sec > 0.0:
                    self.header_based_frame_rate = 1.0 / header_interval_sec

            self.previous_header_stamp_sec = header_stamp_sec

            node_now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.image_stamp_delay_ms = max((node_now_sec - header_stamp_sec) * 1000.0, 0.0)

    def process_latest_image_from_main_loop(self) -> None:
        """
        在 main() 主循环中按固定频率处理图像。

        OpenCV 的 imshow/waitKey 放在主线程/主循环执行，避免在 ROS2 timer 回调里按 q 退出卡死。
        """
        if self.is_shutdown_requested:
            return

        current_time_sec = time.monotonic()
        image_processing_period_sec = 1.0 / max(self.image_processing_rate_hz, MINIMUM_TIMER_RATE_HZ)

        if current_time_sec - self.last_main_loop_image_process_time_sec < image_processing_period_sec:
            return

        self.last_main_loop_image_process_time_sec = current_time_sec
        self.process_latest_image()

    def process_latest_image(self) -> None:
        """处理最新图像并更新人脸目标状态 / Process latest image and update face target state."""
        if self.is_shutdown_requested:
            return

        with self.latest_image_lock:
            if self.latest_image_frame is None:
                return

            image_frame = self.latest_image_frame.copy()
            image_timestamp_sec = self.latest_image_timestamp_sec

        if image_timestamp_sec <= self.last_processed_image_timestamp_sec:
            if self.show_debug_window:
                self._draw_and_show_debug_image(image_frame, self.last_detection_result_for_display)
            return

        self.last_processed_image_timestamp_sec = image_timestamp_sec

        detection_result = self._detect_main_face(image_frame)
        self._update_target_state_from_detection(detection_result, image_frame)
        self.last_detection_result_for_display = detection_result

        if self.show_debug_window:
            self._draw_and_show_debug_image(image_frame, detection_result)

    def _detect_main_face(self, image_frame: np.ndarray) -> FaceDetectionResult:
        """检测主目标人脸 / Detect main target face."""
        if self.face_cascade_classifier is None:
            return FaceDetectionResult(False)

        image_height, image_width = image_frame.shape[:2]

        detection_scale = clamp_numeric_value(
            self.face_detection_scale,
            MINIMUM_FACE_DETECTION_SCALE,
            MAXIMUM_FACE_DETECTION_SCALE,
        )

        if detection_scale < 0.999:
            detection_image_frame = cv2.resize(
                image_frame,
                (0, 0),
                fx=detection_scale,
                fy=detection_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            detection_image_frame = image_frame

        gray_detection_image = cv2.cvtColor(detection_image_frame, cv2.COLOR_BGR2GRAY)

        if self.enable_gray_equalize:
            gray_detection_image = cv2.equalizeHist(gray_detection_image)

        scaled_min_width = max(int(self.face_min_width * detection_scale), 1)
        scaled_min_height = max(int(self.face_min_height * detection_scale), 1)

        detected_faces = self.face_cascade_classifier.detectMultiScale(
            gray_detection_image,
            scaleFactor=max(self.face_detection_scale_factor, 1.01),
            minNeighbors=max(self.face_min_neighbors, 1),
            minSize=(scaled_min_width, scaled_min_height),
        )

        if len(detected_faces) <= 0:
            return FaceDetectionResult(False)

        mapped_face_rectangles: List[Tuple[int, int, int, int]] = []

        for detected_face in detected_faces:
            detected_x, detected_y, detected_width, detected_height = [int(value) for value in detected_face]

            if detection_scale < 0.999:
                face_x = int(round(detected_x / detection_scale))
                face_y = int(round(detected_y / detection_scale))
                face_width = int(round(detected_width / detection_scale))
                face_height = int(round(detected_height / detection_scale))
            else:
                face_x = detected_x
                face_y = detected_y
                face_width = detected_width
                face_height = detected_height

            face_x = int(clamp_numeric_value(face_x, 0, image_width - 1))
            face_y = int(clamp_numeric_value(face_y, 0, image_height - 1))
            face_width = int(clamp_numeric_value(face_width, 1, image_width - face_x))
            face_height = int(clamp_numeric_value(face_height, 1, image_height - face_y))

            mapped_face_rectangles.append((face_x, face_y, face_width, face_height))

        selected_face_rectangle = self._select_main_face_rectangle(
            mapped_face_rectangles,
            image_width,
            image_height,
        )

        if selected_face_rectangle is None:
            return FaceDetectionResult(False)

        face_x, face_y, face_width, face_height = selected_face_rectangle

        face_center_x = float(face_x) + float(face_width) * 0.5
        face_center_y = float(face_y) + float(face_height) * 0.5

        return FaceDetectionResult(
            True,
            face_center_x,
            face_center_y,
            selected_face_rectangle,
            tuple(mapped_face_rectangles),
        )

    def _select_main_face_rectangle(
        self,
        face_rectangles: Sequence[Tuple[int, int, int, int]],
        image_width: int,
        image_height: int,
    ) -> Optional[Tuple[int, int, int, int]]:
        """从多个人脸中选择主目标 / Select main target from multiple faces."""
        if not face_rectangles:
            return None

        if self.face_select_strategy == "nearest_center":
            image_center_x = float(image_width) * 0.5
            image_center_y = float(image_height) * 0.5

            return min(
                face_rectangles,
                key=lambda rectangle: (
                    (float(rectangle[0]) + float(rectangle[2]) * 0.5 - image_center_x) ** 2
                    + (float(rectangle[1]) + float(rectangle[3]) * 0.5 - image_center_y) ** 2
                ),
            )

        return max(face_rectangles, key=lambda rectangle: int(rectangle[2]) * int(rectangle[3]))

    def _update_target_state_from_detection(
        self,
        detection_result: FaceDetectionResult,
        image_frame: np.ndarray,
    ) -> None:
        """根据人脸检测结果更新目标状态 / Update target state from face detection result."""
        image_height, image_width = image_frame.shape[:2]

        with self.target_state_lock:
            self.latest_tracking_image_width = int(image_width)
            self.latest_tracking_image_height = int(image_height)

            if detection_result.is_face_detected:
                self.raw_target_center_x = detection_result.face_center_x
                self.raw_target_center_y = detection_result.face_center_y
                self.raw_face_rectangle = detection_result.face_rectangle
                self.all_raw_face_rectangles = detection_result.all_face_rectangles
                self.target_is_valid = True
                self.last_valid_target_time_sec = time.monotonic()

                if self.filtered_target_center_x is None or self.filtered_target_center_y is None:
                    self.filtered_target_center_x = detection_result.face_center_x
                    self.filtered_target_center_y = detection_result.face_center_y
                else:
                    self.filtered_target_center_x = (
                        (1.0 - self.target_smooth_alpha) * self.filtered_target_center_x
                        + self.target_smooth_alpha * detection_result.face_center_x
                    )
                    self.filtered_target_center_y = (
                        (1.0 - self.target_smooth_alpha) * self.filtered_target_center_y
                        + self.target_smooth_alpha * detection_result.face_center_y
                    )

                if self.tracking_state in (
                    FACE_TRACKING_STATE_SEARCHING,
                    FACE_TRACKING_STATE_LOST,
                ):
                    if self.auto_start_tracking and (
                        self.auto_reacquire_face or self.tracking_state != FACE_TRACKING_STATE_LOST
                    ):
                        self.tracking_state = FACE_TRACKING_STATE_TRACKING
                    else:
                        self.tracking_state = FACE_TRACKING_STATE_IDENTIFY

                return

            target_age_sec = time.monotonic() - self.last_valid_target_time_sec

            if target_age_sec > self.target_lost_timeout_sec:
                self.target_is_valid = False

                if self.lost_target_stop:
                    self.filtered_target_center_x = None
                    self.filtered_target_center_y = None

                if self.tracking_state != FACE_TRACKING_STATE_PAUSED:
                    self.tracking_state = FACE_TRACKING_STATE_LOST

    def _get_face_target_state_snapshot(self) -> FaceTargetStateSnapshot:
        """获取目标状态快照 / Get target state snapshot."""
        with self.target_state_lock:
            return FaceTargetStateSnapshot(
                self.target_is_valid,
                time.monotonic() - self.last_valid_target_time_sec,
                self.filtered_target_center_x,
                self.filtered_target_center_y,
                self.latest_tracking_image_width,
                self.latest_tracking_image_height,
            )

    def ptz_control_timer_callback(self) -> None:
        """固定频率云台控制回调 / Fixed-rate PTZ control callback."""
        if self.is_shutdown_requested:
            return

        current_time_sec = time.monotonic()
        time_delta_sec = current_time_sec - self.last_control_time_sec
        self.last_control_time_sec = current_time_sec

        if self.tracking_state != FACE_TRACKING_STATE_TRACKING:
            self._reset_pd_control_state()
            return

        target_state_snapshot = self._get_face_target_state_snapshot()

        if not self._is_target_snapshot_usable(target_state_snapshot):
            self._handle_unusable_target_snapshot(target_state_snapshot)
            return

        image_center_x = float(target_state_snapshot.image_width) * 0.5
        image_center_y = float(target_state_snapshot.image_height) * 0.5

        horizontal_error_pixel = float(target_state_snapshot.filtered_target_center_x) - image_center_x
        pitch_error_pixel = float(target_state_snapshot.filtered_target_center_y) - image_center_y

        self.calculate_ptz_angle_command(horizontal_error_pixel, pitch_error_pixel, time_delta_sec)
        self.publish_ptz_angle_command()
        self._log_debug_control_state(horizontal_error_pixel, pitch_error_pixel)
        self._print_ptz_angle_if_needed()

    def _is_target_snapshot_usable(self, target_state_snapshot: FaceTargetStateSnapshot) -> bool:
        """判断目标快照是否可用于控制 / Check whether target snapshot is usable for control."""
        return (
            target_state_snapshot.target_is_valid
            and target_state_snapshot.filtered_target_center_x is not None
            and target_state_snapshot.filtered_target_center_y is not None
            and target_state_snapshot.image_width > 0
            and target_state_snapshot.image_height > 0
            and target_state_snapshot.target_age_sec <= self.target_lost_timeout_sec
        )

    def _handle_unusable_target_snapshot(self, target_state_snapshot: FaceTargetStateSnapshot) -> None:
        """处理不可用目标快照 / Handle unusable target snapshot."""
        self._reset_pd_control_state()

        if self.lost_target_stop and target_state_snapshot.target_age_sec > self.target_lost_timeout_sec:
            with self.target_state_lock:
                self.target_is_valid = False
                self.filtered_target_center_x = None
                self.filtered_target_center_y = None

                if self.tracking_state != FACE_TRACKING_STATE_PAUSED:
                    self.tracking_state = FACE_TRACKING_STATE_LOST

    def calculate_ptz_angle_command(
        self,
        horizontal_error_pixel: float,
        pitch_error_pixel: float,
        time_delta_sec: float,
    ) -> None:
        """根据目标像素误差计算云台角度命令 / Calculate PTZ angle command from pixel error."""
        if self.latest_tracking_image_width <= 0 or self.latest_tracking_image_height <= 0:
            return

        target_is_centered_x = abs(horizontal_error_pixel) <= self.deadband_x
        target_is_centered_y = abs(pitch_error_pixel) <= self.deadband_y
        if target_is_centered_x:
            horizontal_error_pixel = 0.0
        if target_is_centered_y:
            pitch_error_pixel = 0.0

        image_center_x = float(self.latest_tracking_image_width) * 0.5
        image_center_y = float(self.latest_tracking_image_height) * 0.5
        normalized_error_x = horizontal_error_pixel / image_center_x
        normalized_error_y = pitch_error_pixel / image_center_y
        time_delta_sec = max(float(time_delta_sec), 1.0e-3)
        derivative_x = (normalized_error_x - self.previous_error_x) / time_delta_sec
        derivative_y = (normalized_error_y - self.previous_error_y) / time_delta_sec

        raw_step_x = (self.pd_kp * normalized_error_x + self.pd_kd * derivative_x) * self.max_ptz_step
        raw_step_y = (self.pd_kp * normalized_error_y + self.pd_kd * derivative_y) * self.max_ptz_step
        raw_step_x = clamp_numeric_value(raw_step_x, -self.max_ptz_step, self.max_ptz_step)
        raw_step_y = clamp_numeric_value(raw_step_y, -self.max_ptz_step, self.max_ptz_step)

        step_smoothing_alpha = self.ptz_step_smooth_alpha
        step_x = (1.0 - step_smoothing_alpha) * self.last_step_x + step_smoothing_alpha * raw_step_x
        step_y = (1.0 - step_smoothing_alpha) * self.last_step_y + step_smoothing_alpha * raw_step_y
        if target_is_centered_x:
            step_x = 0.0
            normalized_error_x = 0.0
        if target_is_centered_y:
            step_y = 0.0
            normalized_error_y = 0.0

        self.previous_error_x = normalized_error_x
        self.previous_error_y = normalized_error_y
        self.last_step_x = step_x
        self.last_step_y = step_y

        if self.reverse_horizontal_direction:
            step_x = -step_x
        if self.reverse_pitch_direction:
            step_y = -step_y

        desired_horizontal_angle_deg = self.horizontal_angle_deg + step_x
        desired_pitch_angle_deg = self.pitch_angle_deg + step_y

        desired_horizontal_angle_deg = clamp_numeric_value(
            desired_horizontal_angle_deg,
            self.horizontal_min_deg,
            self.horizontal_max_deg,
        )
        desired_pitch_angle_deg = clamp_numeric_value(
            desired_pitch_angle_deg,
            self.pitch_min_deg,
            self.pitch_max_deg,
        )

        self.horizontal_angle_deg = (
            (1.0 - self.ptz_smooth_alpha) * self.horizontal_angle_deg
            + self.ptz_smooth_alpha * desired_horizontal_angle_deg
        )
        self.pitch_angle_deg = (
            (1.0 - self.ptz_smooth_alpha) * self.pitch_angle_deg
            + self.ptz_smooth_alpha * desired_pitch_angle_deg
        )

        self.horizontal_angle_deg = clamp_numeric_value(
            self.horizontal_angle_deg,
            self.horizontal_min_deg,
            self.horizontal_max_deg,
        )
        self.pitch_angle_deg = clamp_numeric_value(
            self.pitch_angle_deg,
            self.pitch_min_deg,
            self.pitch_max_deg,
        )

    def _reset_pd_control_state(self) -> None:
        """重置 PD 控制历史状态 / Reset PD control history."""
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

    def publish_ptz_angle_command(self) -> None:
        """发布云台角度命令 / Publish PTZ angle command."""
        ptz_angle_message = Vector3()
        ptz_angle_message.x = float(self.horizontal_angle_deg)
        ptz_angle_message.y = float(self.pitch_angle_deg)
        ptz_angle_message.z = 0.0
        self.ptz_angle_publisher.publish(ptz_angle_message)

    def _log_debug_control_state(self, horizontal_error_pixel: float, pitch_error_pixel: float) -> None:
        """按固定间隔打印控制调试信息 / Log control debug state periodically."""
        if self.debug_log_interval_sec <= 0.0:
            return

        current_time_sec = time.monotonic()

        if current_time_sec - self.last_debug_log_time_sec < self.debug_log_interval_sec:
            return

        self.last_debug_log_time_sec = current_time_sec

        self.get_logger().info(
            f"state={self.tracking_state}, error=({horizontal_error_pixel:.1f}, {pitch_error_pixel:.1f}), "
            f"ptz=({self.horizontal_angle_deg:.2f}, {self.pitch_angle_deg:.2f}), "
            f"pd=({self.pd_kp:.2f},{self.pd_kd:.2f}), smooth={self.ptz_step_smooth_alpha:.2f}"
        )

    def _print_ptz_angle_if_needed(self) -> None:
        """按需打印云台角度 / Print PTZ angle if needed."""
        if not self.print_ptz_angle:
            return

        current_time_sec = time.monotonic()

        if current_time_sec - self.last_ptz_print_time_sec < self.print_interval_sec:
            return

        self.last_ptz_print_time_sec = current_time_sec

        print(
            f"face_ptz_angle: horizontal={self.horizontal_angle_deg:.2f}, "
            f"pitch={self.pitch_angle_deg:.2f}"
        )

    def _draw_and_show_debug_image(
        self,
        image_frame: np.ndarray,
        detection_result: FaceDetectionResult,
    ) -> None:
        """绘制并显示调试图像 / Draw and show debug image."""
        if self.is_shutdown_requested:
            return

        display_image = image_frame.copy()

        for face_rectangle in detection_result.all_face_rectangles:
            face_x, face_y, face_width, face_height = face_rectangle
            cv2.rectangle(
                display_image,
                (face_x, face_y),
                (face_x + face_width, face_y + face_height),
                (255, 255, 0),
                1,
            )

        if detection_result.is_face_detected:
            face_x, face_y, face_width, face_height = detection_result.face_rectangle

            cv2.rectangle(
                display_image,
                (face_x, face_y),
                (face_x + face_width, face_y + face_height),
                (0, 255, 0),
                2,
            )

            cv2.circle(
                display_image,
                (int(round(detection_result.face_center_x)), int(round(detection_result.face_center_y))),
                4,
                (0, 0, 255),
                -1,
            )

        with self.target_state_lock:
            if self.filtered_target_center_x is not None and self.filtered_target_center_y is not None:
                cv2.circle(
                    display_image,
                    (int(round(self.filtered_target_center_x)), int(round(self.filtered_target_center_y))),
                    5,
                    (255, 0, 255),
                    2,
                )

        self._draw_overlay_text(display_image)

        try:
            cv2.imshow(self.window_name, display_image)
            key_code = cv2.waitKey(1) & 0xFF
            self._handle_key_input(key_code)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"OpenCV show window failed: {exception_error}")

    def _draw_overlay_text(self, display_image: np.ndarray) -> None:
        """绘制叠加文字 / Draw overlay text."""
        frame_rate_value = (
            self.header_based_frame_rate
            if self.use_header_stamp_for_fps
            else self.receive_based_frame_rate
        )

        overlay_lines = [
            f"state: {self.tracking_state}",
            f"fps: {frame_rate_value:.1f}",
            f"ptz: h={self.horizontal_angle_deg:.1f}, p={self.pitch_angle_deg:.1f}",
            f"PD: kp={self.pd_kp:.2f}, kd={self.pd_kd:.2f}, smooth={self.ptz_step_smooth_alpha:.2f}",
            "SPACE track/pause, i identify, r reset, q quit",
        ]

        if self.show_image_stamp_delay:
            overlay_lines.insert(2, f"delay: {self.image_stamp_delay_ms:.1f}ms")

        if self.overlay_show_background:
            background_height = max(self.overlay_line_height * len(overlay_lines) + 8, 1)
            cv2.rectangle(
                display_image,
                (0, 0),
                (min(display_image.shape[1], 470), min(display_image.shape[0], background_height)),
                (0, 0, 0),
                -1,
            )

        for line_index, overlay_text in enumerate(overlay_lines):
            text_y = self.overlay_margin_y + line_index * self.overlay_line_height

            cv2.putText(
                display_image,
                overlay_text,
                (self.overlay_margin_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def _handle_key_input(self, key_code: int) -> None:
        """
        处理键盘输入 / Handle keyboard input.

        重要:
            q/Q/ESC 不调用 rclpy.shutdown()。
            q/Q/ESC 不调用 cv2.destroyAllWindows()。
            q/Q/ESC 直接 os._exit(0) 结束当前节点进程，避免 OpenCV/ROS2 清理阶段卡死。
        """
        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            # 使用 print + flush，避免 os._exit(0) 直接退出导致日志缓存来不及刷新。
            print("[face_tracker_ptz_node] q/Q/ESC pressed, hard exit current process with os._exit(0).", flush=True)

            # 严格按照退出优化要求：
            # 1. 不调用 rclpy.shutdown()。
            # 2. 不调用 cv2.destroyAllWindows()。
            # 3. 不调用 cleanup()。
            # 4. 直接结束当前节点进程。
            os._exit(0)  # pylint: disable=protected-access

        if key_code in (ord("r"), ord("R")):
            self._reset_face_target_state()
            self.tracking_state = FACE_TRACKING_STATE_SEARCHING
            self.get_logger().info("Face target reset. Searching again.")
            return

        if key_code in (ord("i"), ord("I")):
            self.tracking_state = FACE_TRACKING_STATE_IDENTIFY
            self._reset_pd_control_state()
            self.get_logger().info("Switched to identify-only mode.")
            return

        if key_code == KEY_CODE_SPACE:
            if self.tracking_state == FACE_TRACKING_STATE_TRACKING:
                self.tracking_state = FACE_TRACKING_STATE_PAUSED
                self._reset_pd_control_state()
                self.get_logger().info("Face tracking paused.")
            else:
                self.tracking_state = FACE_TRACKING_STATE_TRACKING
                self._reset_pd_control_state()
                self.get_logger().info("Face tracking started.")

    def _reset_face_target_state(self) -> None:
        """重置人脸目标状态 / Reset face target state."""
        with self.target_state_lock:
            self.target_is_valid = False
            self.raw_target_center_x = None
            self.raw_target_center_y = None
            self.filtered_target_center_x = None
            self.filtered_target_center_y = None
            self.raw_face_rectangle = (0, 0, 0, 0)
            self.all_raw_face_rectangles = ()

        self._reset_pd_control_state()

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """动态参数更新回调 / Dynamic parameter update callback."""
        if not self.enable_rqt_dynamic_parameters:
            return SetParametersResult(successful=False, reason="Dynamic parameters are disabled.")

        parameter_names = {parameter.name for parameter in parameters}

        for parameter_name in parameter_names:
            if parameter_name in RESTART_REQUIRED_PARAMETER_NAMES:
                return SetParametersResult(
                    successful=False,
                    reason=f"Parameter '{parameter_name}' requires node restart.",
                )

            if parameter_name not in DYNAMIC_PARAMETER_NAMES:
                return SetParametersResult(
                    successful=False,
                    reason=f"Parameter '{parameter_name}' is not dynamically tunable.",
                )

        previous_control_rate_hz = self.control_rate_hz
        previous_image_processing_rate_hz = self.image_processing_rate_hz
        previous_face_cascade_file = self.face_cascade_file
        previous_face_cascade_classifier = self.face_cascade_classifier

        try:
            self._load_ros_parameters()

            if previous_face_cascade_file != self.face_cascade_file:
                if not self._reload_face_cascade_classifier():
                    self.face_cascade_file = previous_face_cascade_file
                    self.face_cascade_classifier = previous_face_cascade_classifier
                    return SetParametersResult(
                        successful=False,
                        reason="Failed to reload face cascade classifier. Previous classifier restored.",
                    )

            if abs(previous_control_rate_hz - self.control_rate_hz) > 1e-6:
                self._recreate_ptz_control_timer(True)

            if abs(previous_image_processing_rate_hz - self.image_processing_rate_hz) > 1e-6:
                self._recreate_image_processing_timer(True)

            self._update_debug_window()

        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception_error))

        return SetParametersResult(successful=True)

    def cleanup(self) -> None:
        """清理节点资源 / Cleanup node resources."""
        if self.cleanup_has_run:
            return

        self.cleanup_has_run = True
        self.is_shutdown_requested = True

        for timer_handle in (
            self.initial_publish_timer,
            self.image_processing_timer,
            self.ptz_control_timer,
        ):
            try:
                if timer_handle is not None:
                    timer_handle.cancel()
            except Exception:  # pylint: disable=broad-except
                pass

        try:
            self.publish_ptz_angle_command()
        except Exception:  # pylint: disable=broad-except
            pass

        # 不在 cleanup 中调用 cv2.destroyAllWindows()/cv2.waitKey()。
        # 某些 OpenCV GUI 后端在 ROS2 launch 退出阶段会卡在这里，
        # 导致 launch 最终用 SIGKILL 杀进程，也就是 exit code -9。
        # 进程退出时 OpenCV 窗口会由系统自动回收。

    def destroy_node(self) -> bool:
        """销毁节点 / Destroy node."""
        self.cleanup()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    节点入口函数 / Node entry function.

    说明:
        使用 spin_once + 主循环处理 OpenCV 显示。
        q/Q/ESC 由 _handle_key_input() 直接 os._exit(0)，不进入 cleanup/shutdown。
    """
    rclpy.init(args=args)

    face_tracker_ptz_node = FaceTrackerPtzNode()

    try:
        while rclpy.ok() and not face_tracker_ptz_node.is_shutdown_requested:
            rclpy.spin_once(face_tracker_ptz_node, timeout_sec=0.001)
            face_tracker_ptz_node.process_latest_image_from_main_loop()
            time.sleep(0.001)

    except KeyboardInterrupt:
        pass

    finally:
        face_tracker_ptz_node.cleanup()
        face_tracker_ptz_node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
