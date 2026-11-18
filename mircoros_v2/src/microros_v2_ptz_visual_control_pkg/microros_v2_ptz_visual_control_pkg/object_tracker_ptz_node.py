#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
object_tracker_ptz_node.py

中文说明:
    ROS2 Humble 物体追踪云台控制节点。
    本版本按 ptz-tracking-rules SKILL 优化：
    - 使用 PD 控制替代 PID（无积分项，归一化误差，死区内归零强制静止）
    - 清理所有旧版 PID 参数（pid_kp/pid_ki/pid_kd/pid_scale/horizontal_kp 等）
    - q/ESC 使用 os._exit(0) 直接退出
    - 图像处理与云台控制解耦，固定频率控制定时器

English Description:
    ROS2 Humble object tracking PTZ control node.
    Optimized per ptz-tracking-rules SKILL:
    - PD control only (no integral term, normalized error, forced zero inside deadband)
    - Removed all legacy PID parameters
    - q/ESC exits via os._exit(0)
    - Decoupled image processing and fixed-rate PTZ control timer
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

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


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "object_tracker_ptz_node"

# TRACKING_STATE_SELECTING: 选择目标状态 / Selecting target state.
TRACKING_STATE_SELECTING = "selecting"

# TRACKING_STATE_READY: 目标已选择但未追踪状态 / Target selected but not tracking state.
TRACKING_STATE_READY = "ready"

# TRACKING_STATE_TRACKING: 正在追踪状态 / Active tracking state.
TRACKING_STATE_TRACKING = "tracking"

# TRACKING_STATE_LOST: 目标丢失状态 / Target lost state.
TRACKING_STATE_LOST = "lost"

# TRACKING_STATE_IDENTIFY: 识别但不控制状态 / Identify target without PTZ control state.
TRACKING_STATE_IDENTIFY = "identify"

# KEY_CODE_SPACE: 空格键编码 / Space key code.
KEY_CODE_SPACE = 32

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# MINIMUM_TIMER_RATE_HZ: 定时器最小频率 / Minimum timer frequency.
MINIMUM_TIMER_RATE_HZ = 1.0

# MINIMUM_TARGET_LOST_TIMEOUT_SEC: 目标丢失超时时间下限 / Lower bound of target lost timeout.
MINIMUM_TARGET_LOST_TIMEOUT_SEC = 0.05

# MINIMUM_ALPHA_VALUE: 平滑系数最小值 / Minimum smoothing alpha value.
MINIMUM_ALPHA_VALUE = 0.01

# MAXIMUM_ALPHA_VALUE: 平滑系数最大值 / Maximum smoothing alpha value.
MAXIMUM_ALPHA_VALUE = 1.0

# DYNAMIC_PARAMETER_NAMES: 支持 rqt 运行时动态修改的参数 / Runtime dynamic parameters for rqt.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "window_width",
    "window_height",
    "overlay_font_scale",
    "overlay_line_height",
    "overlay_margin_x",
    "overlay_margin_y",
    "overlay_show_background",
    "show_binary_window",
    "img_flip",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "horizontal_min_deg",
    "horizontal_max_deg",
    "pitch_min_deg",
    "pitch_max_deg",
    "reverse_horizontal_direction",
    "reverse_pitch_direction",
    "deadband_x",
    "deadband_y",
    "pd_kp",
    "pd_kd",
    "ptz_step_smooth_alpha",
    "max_ptz_step",
    "target_smooth_alpha",
    "ptz_smooth_alpha",
    "control_rate_hz",
    "image_processing_rate_hz",
    "tracking_min_area",
    "tracking_backend",
    "opencv_tracker_type",
    "hsv_saturation_min",
    "hsv_value_min",
    "hsv_value_max",
    "Hmin",
    "Smin",
    "Vmin",
    "Hmax",
    "Smax",
    "Vmax",
    "h_expand",
    "s_expand",
    "v_expand",
    "morph_kernel_size",
    "min_target_radius",
    "image_width",
    "image_height",
    "hsv_file",
    "target_lost_timeout_sec",
    "lost_target_stop",
    "publish_initial_ptz_command",
    "initial_publish_count",
    "initial_publish_interval_sec",
    "debug_log_interval_sec",
    "print_ptz_angle",
    "print_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
    "enable_rqt_dynamic_parameters",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require node restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "ptz_command_topic",
    "window_name",
})


@dataclass
class TargetTrackingResult:
    """目标追踪结果 / Target tracking result."""

    # is_tracking_successful: 追踪是否成功 / Whether tracking is successful.
    is_tracking_successful: bool

    # target_center_x: 目标中心 x 像素坐标 / Target center x coordinate in pixels.
    target_center_x: float = 0.0

    # target_center_y: 目标中心 y 像素坐标 / Target center y coordinate in pixels.
    target_center_y: float = 0.0

    # target_rectangle: 目标外接矩形 (x, y, width, height) / Target bounding rectangle.
    target_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)

    # target_polygon: 目标旋转框点集 / Target rotated polygon points.
    target_polygon: Optional[np.ndarray] = None


@dataclass
class TargetStateSnapshot:
    """目标状态快照 / Target state snapshot."""

    # target_is_valid: 目标是否有效 / Whether target is valid.
    target_is_valid: bool

    # target_age_sec: 目标状态距离上次有效更新的时间 / Age since last valid target update.
    target_age_sec: float

    # filtered_target_center_x: 滤波后的目标中心 x / Filtered target center x.
    filtered_target_center_x: Optional[float]

    # filtered_target_center_y: 滤波后的目标中心 y / Filtered target center y.
    filtered_target_center_y: Optional[float]

    # image_width: 图像宽度 / Image width.
    image_width: int

    # image_height: 图像高度 / Image height.
    image_height: int


def clamp_numeric_value(current_value: float, minimum_value: float, maximum_value: float) -> float:
    """
    数值限幅 / Clamp numeric value.

    Args:
        current_value: 当前数值 / Current value.
        minimum_value: 最小值 / Minimum value.
        maximum_value: 最大值 / Maximum value.

    Returns:
        限幅后的数值 / Clamped value.
    """
    return max(float(minimum_value), min(float(maximum_value), float(current_value)))


def convert_value_to_boolean(input_value: Any, default_boolean_value: bool) -> bool:
    """
    安全转换布尔参数 / Safely convert value to boolean.

    Args:
        input_value: 输入参数值 / Input parameter value.
        default_boolean_value: 默认布尔值 / Default boolean value.

    Returns:
        转换后的布尔值 / Converted boolean value.
    """
    if isinstance(input_value, bool):
        return input_value
    if isinstance(input_value, str):
        # normalized_text_value: 归一化文本 / Normalized text value.
        normalized_text_value = input_value.strip().lower()
        if normalized_text_value in ("true", "1", "yes", "y", "on"):
            return True
        if normalized_text_value in ("false", "0", "no", "n", "off"):
            return False
    if isinstance(input_value, (int, float)):
        return bool(input_value)
    return default_boolean_value


def read_hsv_threshold_file(hsv_file_path: str) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]:
    """从文件读取 HSV 阈值范围 / Read HSV threshold range from file.

    文件格式 / File format：Hmin, Smin, Vmin, Hmax, Smax, Vmax
    """
    if not hsv_file_path:
        return None
    expanded_hsv_file_path = os.path.expanduser(hsv_file_path)
    if not os.path.exists(expanded_hsv_file_path):
        return None
    try:
        with open(expanded_hsv_file_path, "r", encoding="utf-8") as hsv_file_reader:
            hsv_line_text = hsv_file_reader.readline().strip()
        if not hsv_line_text:
            return None
        hsv_integer_values = [int(text_value.strip()) for text_value in hsv_line_text.split(",")]
        if len(hsv_integer_values) != 6:
            return None
        return (
            (hsv_integer_values[0], hsv_integer_values[1], hsv_integer_values[2]),
            (hsv_integer_values[3], hsv_integer_values[4], hsv_integer_values[5]),
        )
    except Exception:
        return None


def write_hsv_threshold_file(hsv_file_path: str, hsv_threshold_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]]) -> None:
    """将 HSV 阈值范围写入文件 / Write HSV threshold range to file."""
    if not hsv_file_path:
        return
    expanded_hsv_file_path = os.path.expanduser(hsv_file_path)
    hsv_file_parent_directory = os.path.dirname(expanded_hsv_file_path)
    if hsv_file_parent_directory:
        os.makedirs(hsv_file_parent_directory, exist_ok=True)
    hsv_file_content_text = (
        f"{hsv_threshold_range[0][0]}, {hsv_threshold_range[0][1]}, {hsv_threshold_range[0][2]}, "
        f"{hsv_threshold_range[1][0]}, {hsv_threshold_range[1][1]}, {hsv_threshold_range[1][2]}"
    )
    with open(expanded_hsv_file_path, "w", encoding="utf-8") as hsv_file_writer:
        hsv_file_writer.write(hsv_file_content_text)
        hsv_file_writer.flush()


class ObjectTrackerPtzNode(Node):
    """ROS2 物体追踪云台控制节点 / ROS2 object tracking PTZ control node."""

    def __init__(self) -> None:
        """初始化节点 / Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # latest_image_lock: 最新图像互斥锁 / Lock for latest image frame.
        self.latest_image_lock = threading.Lock()

        # target_state_lock: 目标状态互斥锁 / Lock for target state.
        self.target_state_lock = threading.Lock()

        # latest_image_frame: 最新 OpenCV 图像 / Latest OpenCV image frame.
        self.latest_image_frame: Optional[np.ndarray] = None

        # latest_image_timestamp_sec: 最新图像接收时间 / Receive time of latest image.
        self.latest_image_timestamp_sec = 0.0

        # last_processed_image_timestamp_sec: 上一次处理图像时间戳 / Last processed image timestamp.
        self.last_processed_image_timestamp_sec = 0.0

        # selected_region_start_point: 鼠标框选起点 / Mouse selected region start point.
        self.selected_region_start_point: Optional[Tuple[int, int]] = None

        # selected_region_end_point: 鼠标框选终点 / Mouse selected region end point.
        self.selected_region_end_point: Optional[Tuple[int, int]] = None

        # selected_tracking_rectangle: 已选择目标矩形 / Selected target rectangle.
        self.selected_tracking_rectangle: Optional[Tuple[int, int, int, int]] = None

        # is_mouse_dragging: 鼠标是否正在拖拽 / Whether mouse is dragging.
        self.is_mouse_dragging = False

        # tracking_state: 当前追踪状态 / Current tracking state.
        self.tracking_state = TRACKING_STATE_SELECTING

        # camshift_histogram: CamShift HSV 直方图 / CamShift HSV histogram.
        self.camshift_histogram: Optional[np.ndarray] = None

        # current_track_window: 当前追踪窗口 / Current tracking window.
        self.current_track_window: Optional[Tuple[int, int, int, int]] = None

        # opencv_tracker: OpenCV 内置追踪器 / Built-in OpenCV tracker.
        self.opencv_tracker = None

        # is_opencv_tracker_available: OpenCV 追踪器是否可用 / Whether OpenCV tracker is available.
        self.is_opencv_tracker_available = False

        # horizontal_angle_deg: 当前 s1 水平舵机角度 / Current s1 horizontal servo angle.
        self.horizontal_angle_deg = 0.0

        # pitch_angle_deg: 当前 s2 俯仰舵机角度 / Current s2 pitch servo angle.
        self.pitch_angle_deg = 0.0

        # PD 控制状态 / PD control state.
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

        # last_debug_log_time_sec: 上一次调试日志时间 / Last debug log time.
        self.last_debug_log_time_sec = 0.0

        # is_shutdown_requested: 是否请求关闭节点 / Whether shutdown is requested.
        self.is_shutdown_requested = False

        # raw_target_rectangle: 原始目标外接矩形 / Raw target bounding rectangle.
        self.raw_target_rectangle = (0, 0, 0, 0)

        # raw_target_polygon: 原始目标旋转框点集 / Raw target rotated polygon.
        self.raw_target_polygon: Optional[np.ndarray] = None

        # filtered_target_center_x: 滤波后的目标中心 x / Filtered target center x.
        self.filtered_target_center_x: Optional[float] = None

        # filtered_target_center_y: 滤波后的目标中心 y / Filtered target center y.
        self.filtered_target_center_y: Optional[float] = None

        # target_is_valid: 当前目标是否有效 / Whether current target is valid.
        self.target_is_valid = False

        # last_valid_target_time_sec: 上一次有效目标时间 / Last valid target time.
        self.last_valid_target_time_sec = time.monotonic()

        # latest_tracking_image_width: 最近追踪图像宽度 / Latest tracking image width.
        self.latest_tracking_image_width = 0

        # latest_tracking_image_height: 最近追踪图像高度 / Latest tracking image height.
        self.latest_tracking_image_height = 0

        # last_tracking_result_for_display: 用于显示的追踪结果 / Tracking result for display.
        self.last_tracking_result_for_display = TargetTrackingResult(False)

        # latest_binary_mask_image: 最新二值掩码图像 / Latest binary mask image.
        self.latest_binary_mask_image: Optional[np.ndarray] = None

        # remaining_initial_publish_count: 剩余初始角度发布次数 / Remaining initial angle publish count.
        self.remaining_initial_publish_count = 0

        # initial_publish_timer: 初始角度发布定时器 / Initial angle publish timer.
        self.initial_publish_timer = None

        # previous_header_stamp_sec: 上一帧 header 时间戳 / Previous frame header timestamp.
        self.previous_header_stamp_sec = 0.0

        # header_based_frame_rate: 基于 header 时间戳的帧率 / Header stamp based FPS.
        self.header_based_frame_rate = 0.0

        # receive_based_frame_rate: 基于接收时间的帧率 / Receive time based FPS.
        self.receive_based_frame_rate = 0.0

        # previous_receive_time_sec: 上一帧接收时间 / Previous receive time.
        self.previous_receive_time_sec = 0.0

        # image_stamp_delay_ms: 图像 header 到接收时刻的延迟 / Image header-to-receive delay in ms.
        self.image_stamp_delay_ms = 0.0

        # last_ptz_print_time_sec: 上一次角度打印时间 / Last PTZ print time.
        self.last_ptz_print_time_sec = 0.0

        self._declare_ros_parameters()
        self._load_ros_parameters()

        self.horizontal_angle_deg = self.initial_horizontal_deg
        self.pitch_angle_deg = self.initial_pitch_deg

        # 从 HSV 文件加载历史阈值（参考 color_tracker_ptz_node 模式）
        # Load persisted HSV threshold from file (reference color_tracker_ptz_node pattern).
        saved_hsv_threshold_range = read_hsv_threshold_file(self.hsv_file)
        if saved_hsv_threshold_range is not None:
            self._apply_hsv_range_to_ros_parameters(saved_hsv_threshold_range)
            self.get_logger().info(
                f"Loaded HSV from file: {self.hsv_file}, hsv={saved_hsv_threshold_range}"
            )

        # image_subscription: 图像订阅器 / Image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        # ptz_angle_publisher: 云台角度发布器 / PTZ angle publisher.
        self.ptz_angle_publisher = self.create_publisher(
            Vector3,
            self.ptz_command_topic_name,
            10,
        )

        # image_processing_timer: 图像处理定时器 / Image processing timer.
        self.image_processing_timer = None

        # ptz_control_timer: 云台控制定时器 / PTZ control timer.
        self.ptz_control_timer = None

        self._recreate_image_processing_timer(False)
        self._recreate_ptz_control_timer(False)

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        self._initialize_debug_window()

        if self.publish_initial_ptz_command:
            self._start_initial_ptz_publish_timer()

        self.get_logger().info(
            "Object tracker PTZ node started (PD control). "
            "Mouse drag to select target, SPACE to track/pause, r to reselect, q/ESC to quit. "
            f"image_topic={self.image_topic_name}, ptz_topic={self.ptz_command_topic_name}, "
            f"control_rate={self.control_rate_hz:.1f}Hz, "
            f"pd_kp={self.pd_kp:.2f}, pd_kd={self.pd_kd:.2f}, "
            f"deadband=({self.deadband_x:.0f},{self.deadband_y:.0f}), max_step={self.max_ptz_step:.1f}"
        )

    def _declare_ros_parameters(self) -> None:
        """声明 ROS 参数 / Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "object_tracker_ptz")
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.45)
        self.declare_parameter("overlay_line_height", 18)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 18)
        self.declare_parameter("overlay_show_background", True)
        self.declare_parameter("show_binary_window", False)

        self.declare_parameter("img_flip", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", 0.0)

        # PD 控制参数 / PD control parameters.
        self.declare_parameter("reverse_horizontal_direction", True)
        self.declare_parameter("reverse_pitch_direction", True)
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)

        self.declare_parameter("target_smooth_alpha", 0.55)
        self.declare_parameter("ptz_smooth_alpha", 0.85)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)
        self.declare_parameter("min_target_radius", 5.0)
        self.declare_parameter("tracking_min_area", 100.0)

        self.declare_parameter("tracking_backend", "camshift")
        self.declare_parameter("opencv_tracker_type", "CSRT")
        self.declare_parameter("hsv_saturation_min", 30)
        self.declare_parameter("hsv_file", "")
        self.declare_parameter("hsv_value_min", 30)
        self.declare_parameter("hsv_value_max", 255)
        self.declare_parameter("Hmin", 0)
        self.declare_parameter("Smin", 85)
        self.declare_parameter("Vmin", 126)
        self.declare_parameter("Hmax", 9)
        self.declare_parameter("Smax", 253)
        self.declare_parameter("Vmax", 253)
        self.declare_parameter("h_expand", 5)
        self.declare_parameter("s_expand", 20)
        self.declare_parameter("v_expand", 20)
        self.declare_parameter("morph_kernel_size", 5)

        self.declare_parameter("target_lost_timeout_sec", 1.0)
        self.declare_parameter("lost_target_stop", True)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("debug_log_interval_sec", 1.0)
        self.declare_parameter("print_ptz_angle", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_ros_parameters(self) -> None:
        """读取 ROS 参数 / Load ROS parameters."""
        # image_topic_name: 图像话题名称 / Image topic name.
        self.image_topic_name = str(self.get_parameter("image_topic").value)

        # ptz_command_topic_name: 云台控制话题名称 / PTZ command topic name.
        self.ptz_command_topic_name = str(self.get_parameter("ptz_command_topic").value)

        # window_name: OpenCV 窗口名称 / OpenCV window name.
        self.window_name = str(self.get_parameter("window_name").value)

        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)

        # show_debug_window: 是否显示调试窗口 / Whether to show debug window.
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
        self.show_binary_window = convert_value_to_boolean(
            self.get_parameter("show_binary_window").value,
            False,
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

        self.horizontal_min_deg = float(self.get_parameter("horizontal_min_deg").value)
        self.horizontal_max_deg = float(self.get_parameter("horizontal_max_deg").value)
        self.pitch_min_deg = float(self.get_parameter("pitch_min_deg").value)
        self.pitch_max_deg = float(self.get_parameter("pitch_max_deg").value)
        self.initial_horizontal_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self.initial_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)

        # PD 控制参数 / PD control parameters.
        self.reverse_horizontal_direction = convert_value_to_boolean(
            self.get_parameter("reverse_horizontal_direction").value,
            True,
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
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
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
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.image_processing_rate_hz = float(self.get_parameter("image_processing_rate_hz").value)
        self.tracking_min_area = float(self.get_parameter("tracking_min_area").value)
        self.min_target_radius = float(self.get_parameter("min_target_radius").value)
        self.hsv_file = os.path.expanduser(str(self.get_parameter("hsv_file").value))

        self.tracking_backend = str(self.get_parameter("tracking_backend").value).lower()
        self.opencv_tracker_type = str(self.get_parameter("opencv_tracker_type").value).upper()
        self.hsv_saturation_min = int(self.get_parameter("hsv_saturation_min").value)
        self.hsv_value_min = int(self.get_parameter("hsv_value_min").value)
        self.hsv_value_max = int(self.get_parameter("hsv_value_max").value)
        self.hue_minimum = int(self.get_parameter("Hmin").value)
        self.saturation_minimum = int(self.get_parameter("Smin").value)
        self.value_minimum = int(self.get_parameter("Vmin").value)
        self.hue_maximum = int(self.get_parameter("Hmax").value)
        self.saturation_maximum = int(self.get_parameter("Smax").value)
        self.value_maximum = int(self.get_parameter("Vmax").value)
        self.hue_expand = int(self.get_parameter("h_expand").value)
        self.saturation_expand = int(self.get_parameter("s_expand").value)
        self.value_expand = int(self.get_parameter("v_expand").value)
        self.morph_kernel_size = int(self.get_parameter("morph_kernel_size").value)

        self.target_lost_timeout_sec = max(
            float(self.get_parameter("target_lost_timeout_sec").value),
            MINIMUM_TARGET_LOST_TIMEOUT_SEC,
        )
        self.lost_target_stop = convert_value_to_boolean(
            self.get_parameter("lost_target_stop").value,
            True,
        )
        self.publish_initial_ptz_command = convert_value_to_boolean(
            self.get_parameter("publish_initial_ptz_command").value,
            True,
        )
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)
        self.debug_log_interval_sec = float(self.get_parameter("debug_log_interval_sec").value)
        self.print_ptz_angle = convert_value_to_boolean(
            self.get_parameter("print_ptz_angle").value,
            False,
        )
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
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

    def _get_hsv_threshold_range_from_parameters(self) -> Tuple[Tuple[int, int, int], Tuple[int, int, int]]:
        """读取 HSV 阈值范围 / Get HSV threshold range from parameters."""
        # hsv_lower_threshold: HSV 下限阈值 / HSV lower threshold.
        hsv_lower_threshold = (
            int(self.hue_minimum),
            int(self.saturation_minimum),
            int(self.value_minimum),
        )

        # hsv_upper_threshold: HSV 上限阈值 / HSV upper threshold.
        hsv_upper_threshold = (
            int(self.hue_maximum),
            int(self.saturation_maximum),
            int(self.value_maximum),
        )
        return hsv_lower_threshold, hsv_upper_threshold

    def _apply_hsv_range_to_ros_parameters(self, hsv_threshold_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]]) -> None:
        """将 ROI 采样得到的 HSV 写入 ROS 参数 / Write sampled HSV thresholds to ROS parameters."""
        # hsv_parameter_list: HSV 参数列表 / HSV parameter list.
        hsv_parameter_list = [
            Parameter("Hmin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][0])),
            Parameter("Smin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][1])),
            Parameter("Vmin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][2])),
            Parameter("Hmax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][0])),
            Parameter("Smax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][1])),
            Parameter("Vmax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][2])),
        ]
        self.set_parameters(hsv_parameter_list)
        self.hue_minimum, self.saturation_minimum, self.value_minimum = hsv_threshold_range[0]
        self.hue_maximum, self.saturation_maximum, self.value_maximum = hsv_threshold_range[1]

    def _sample_hsv_range_from_selected_region(
        self,
        image_frame: np.ndarray,
        target_rectangle: Tuple[int, int, int, int],
    ) -> Optional[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]:
        """从已框选 ROI 中采样 HSV 阈值 / Sample HSV thresholds from selected ROI."""
        rectangle_x, rectangle_y, rectangle_width, rectangle_height = target_rectangle
        # selected_region_image: 框选区域图像 / Selected ROI image.
        selected_region_image = image_frame[
            rectangle_y : rectangle_y + rectangle_height,
            rectangle_x : rectangle_x + rectangle_width,
        ]
        if selected_region_image.size == 0:
            return None

        # selected_region_hsv: 框选区域 HSV 图像 / HSV image of selected ROI.
        selected_region_hsv = cv2.cvtColor(selected_region_image, cv2.COLOR_BGR2HSV)

        # hsv_channel_minimums: HSV 各通道最小值 / Minimum values of HSV channels.
        hsv_channel_minimums = np.min(selected_region_hsv.reshape(-1, 3), axis=0)

        # hsv_channel_maximums: HSV 各通道最大值 / Maximum values of HSV channels.
        hsv_channel_maximums = np.max(selected_region_hsv.reshape(-1, 3), axis=0)

        # hue_minimum/hue_maximum: 色相范围 / Hue range.
        hue_minimum = int(clamp_numeric_value(hsv_channel_minimums[0] - self.hue_expand, 0, 179))
        hue_maximum = int(clamp_numeric_value(hsv_channel_maximums[0] + self.hue_expand, 0, 179))

        # saturation_minimum/saturation_maximum: 饱和度范围 / Saturation range.
        saturation_minimum = int(clamp_numeric_value(hsv_channel_minimums[1] - self.saturation_expand, 0, 255))
        saturation_maximum = int(clamp_numeric_value(hsv_channel_maximums[1] + self.saturation_expand, 0, 255))

        # value_minimum/value_maximum: 明度范围 / Value range.
        value_minimum = int(clamp_numeric_value(hsv_channel_minimums[2] - self.value_expand, 0, 255))
        value_maximum = int(clamp_numeric_value(hsv_channel_maximums[2] + self.value_expand, 0, 255))

        return (hue_minimum, saturation_minimum, value_minimum), (hue_maximum, saturation_maximum, value_maximum)

    def _apply_image_orientation(self, decoded_image_frame: np.ndarray) -> np.ndarray:
        """根据参数修正图像方向 / Correct image orientation according to parameters."""
        # oriented_image_frame: 修正方向后的图像 / Orientation corrected image.
        oriented_image_frame = decoded_image_frame

        if self.img_flip or self.enable_horizontal_flip:
            oriented_image_frame = cv2.flip(oriented_image_frame, 1)

        if self.enable_vertical_flip:
            oriented_image_frame = cv2.flip(oriented_image_frame, 0)

        if self.rotate_image_code in ("90_clockwise", "cw90"):
            oriented_image_frame = cv2.rotate(oriented_image_frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code in ("90_counterclockwise", "ccw90"):
            oriented_image_frame = cv2.rotate(oriented_image_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code in ("180", "rotate_180"):
            oriented_image_frame = cv2.rotate(oriented_image_frame, cv2.ROTATE_180)

        return oriented_image_frame

    def _start_initial_ptz_publish_timer(self) -> None:
        """启动初始云台角度重复发布定时器 / Start repeated initial PTZ angle publish timer."""
        self.remaining_initial_publish_count = max(0, int(self.initial_publish_count))
        if self.remaining_initial_publish_count <= 0:
            return
        self.publish_ptz_angle_command()
        self.remaining_initial_publish_count -= 1
        if self.remaining_initial_publish_count <= 0:
            return
        # safe_initial_interval_sec: 安全的初始发布间隔 / Safe initial publish interval.
        safe_initial_interval_sec = max(float(self.initial_publish_interval_sec), 0.02)
        self.initial_publish_timer = self.create_timer(
            safe_initial_interval_sec,
            self._publish_initial_ptz_angle_once,
        )

    def _publish_initial_ptz_angle_once(self) -> None:
        """定时发布一次初始云台角度 / Publish one initial PTZ angle on timer."""
        if self.remaining_initial_publish_count <= 0:
            if self.initial_publish_timer is not None:
                self.destroy_timer(self.initial_publish_timer)
                self.initial_publish_timer = None
            return

        self.publish_ptz_angle_command()
        self.remaining_initial_publish_count -= 1

    def _update_image_timing_statistics(self, image_msg: Image) -> None:
        """更新图像帧率和延迟统计 / Update image FPS and delay statistics."""
        # current_receive_time_sec: 当前接收时间 / Current receive time.
        current_receive_time_sec = time.monotonic()
        if self.previous_receive_time_sec > 0.0:
            # receive_interval_sec: 接收间隔 / Receive interval.
            receive_interval_sec = current_receive_time_sec - self.previous_receive_time_sec
            if receive_interval_sec > 1.0e-6:
                self.receive_based_frame_rate = 1.0 / receive_interval_sec
        self.previous_receive_time_sec = current_receive_time_sec

        if not self.use_header_stamp_for_fps:
            return

        # header_stamp_sec: 图像消息 header 时间戳 / Image message header stamp.
        header_stamp_sec = (
            float(image_msg.header.stamp.sec)
            + float(image_msg.header.stamp.nanosec) * 1.0e-9
        )
        if header_stamp_sec <= 0.0:
            return

        if self.previous_header_stamp_sec > 0.0:
            # header_interval_sec: header 时间戳间隔 / Header stamp interval.
            header_interval_sec = header_stamp_sec - self.previous_header_stamp_sec
            if header_interval_sec > 1.0e-6:
                self.header_based_frame_rate = 1.0 / header_interval_sec
        self.previous_header_stamp_sec = header_stamp_sec

        # node_time_sec: 当前 ROS 时钟时间 / Current ROS clock time.
        node_time_sec = float(self.get_clock().now().nanoseconds) * 1.0e-9
        self.image_stamp_delay_ms = max(0.0, (node_time_sec - header_stamp_sec) * 1000.0)

    def _build_display_image_with_optional_binary(self, display_image_frame: np.ndarray) -> np.ndarray:
        """根据开关生成原图或原图+二值图显示 / Build normal or image-plus-binary display frame."""
        if not self.show_binary_window or self.latest_binary_mask_image is None:
            return display_image_frame

        try:
            # binary_mask_bgr_image: BGR 格式二值图 / Binary mask converted to BGR image.
            binary_mask_bgr_image = cv2.cvtColor(self.latest_binary_mask_image, cv2.COLOR_GRAY2BGR)
            if binary_mask_bgr_image.shape[:2] != display_image_frame.shape[:2]:
                binary_mask_bgr_image = cv2.resize(
                    binary_mask_bgr_image,
                    (display_image_frame.shape[1], display_image_frame.shape[0]),
                )
            return np.hstack([display_image_frame, binary_mask_bgr_image])
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Build binary display image failed: {exception_error}")
            return display_image_frame

    def _initialize_debug_window(self) -> None:
        """初始化 OpenCV 调试窗口 / Initialize OpenCV debug window."""
        if not self.show_debug_window:
            return

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        cv2.setMouseCallback(self.window_name, self.mouse_event_callback)

    def _recreate_image_processing_timer(self, should_log_update: bool) -> None:
        """
        重建图像处理定时器 / Recreate image processing timer.

        Args:
            should_log_update: 是否打印更新日志 / Whether to log update.
        """
        # image_processing_timer_period_sec: 图像处理定时器周期 / Image processing timer period.
        image_processing_timer_period_sec = 1.0 / max(
            float(self.image_processing_rate_hz),
            MINIMUM_TIMER_RATE_HZ,
        )

        if self.image_processing_timer is not None:
            self.destroy_timer(self.image_processing_timer)

        self.image_processing_timer = self.create_timer(
            image_processing_timer_period_sec,
            self.process_latest_image,
        )

        if should_log_update:
            self.get_logger().info(
                f"Image processing timer updated: rate={self.image_processing_rate_hz:.2f}Hz"
            )

    def _recreate_ptz_control_timer(self, should_log_update: bool) -> None:
        """
        重建固定频率云台控制定时器 / Recreate fixed-rate PTZ control timer.

        Args:
            should_log_update: 是否打印更新日志 / Whether to log update.
        """
        # ptz_control_timer_period_sec: 云台控制定时器周期 / PTZ control timer period.
        ptz_control_timer_period_sec = 1.0 / max(float(self.control_rate_hz), MINIMUM_TIMER_RATE_HZ)

        if self.ptz_control_timer is not None:
            self.destroy_timer(self.ptz_control_timer)

        self.ptz_control_timer = self.create_timer(
            ptz_control_timer_period_sec,
            self.ptz_control_timer_callback,
        )

        if should_log_update:
            self.get_logger().info(f"PTZ control timer updated: rate={self.control_rate_hz:.2f}Hz")

    def image_callback(self, image_msg: Image) -> None:
        """
        接收并转换图像 / Receive and convert image.

        Args:
            image_msg: 图像消息 / Image message.
        """
        try:
            decoded_image_frame = image_to_bgr(image_msg)

            decoded_image_frame = self._apply_image_orientation(decoded_image_frame)

            # 按参数缩放图像（参考 color_tracker_ptz_node）/ Scale image per parameter.
            if self.image_width > 0 and self.image_height > 0:
                decoded_image_frame = cv2.resize(
                    decoded_image_frame,
                    (self.image_width, self.image_height),
                    interpolation=cv2.INTER_AREA,
                )

            self._update_image_timing_statistics(image_msg)

            with self.latest_image_lock:
                self.latest_image_frame = decoded_image_frame
                self.latest_image_timestamp_sec = time.monotonic()
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def mouse_event_callback(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        """
        处理鼠标框选事件 / Handle mouse selection events.

        Args:
            event: OpenCV 鼠标事件 / OpenCV mouse event.
            x: 鼠标 x 坐标 / Mouse x coordinate.
            y: 鼠标 y 坐标 / Mouse y coordinate.
            flags: OpenCV 鼠标标志 / OpenCV mouse flags.
            param: OpenCV 回调参数 / OpenCV callback parameter.
        """
        del flags, param

        if event == cv2.EVENT_LBUTTONDOWN:
            self.selected_region_start_point = (x, y)
            self.selected_region_end_point = (x, y)
            self.selected_tracking_rectangle = None
            self.is_mouse_dragging = True
            self.tracking_state = TRACKING_STATE_SELECTING
            self._reset_filtered_target_state()

        elif event == cv2.EVENT_MOUSEMOVE and self.is_mouse_dragging:
            self.selected_region_end_point = (x, y)

        elif event == cv2.EVENT_LBUTTONUP:
            self.selected_region_end_point = (x, y)
            self.is_mouse_dragging = False
            self._update_selected_rectangle_from_mouse_points()
            self.initialize_tracker_from_selected_region()

    def _update_selected_rectangle_from_mouse_points(self) -> None:
        """根据鼠标起止点更新目标矩形 / Update target rectangle from mouse points."""
        if self.selected_region_start_point is None or self.selected_region_end_point is None:
            return

        # start_x/start_y: 鼠标起点坐标 / Mouse start coordinates.
        start_x, start_y = self.selected_region_start_point

        # end_x/end_y: 鼠标终点坐标 / Mouse end coordinates.
        end_x, end_y = self.selected_region_end_point

        # rectangle_x/rectangle_y: 目标矩形左上角 / Target rectangle top-left corner.
        rectangle_x = min(start_x, end_x)
        rectangle_y = min(start_y, end_y)

        # rectangle_width/rectangle_height: 目标矩形宽高 / Target rectangle size.
        rectangle_width = abs(end_x - start_x)
        rectangle_height = abs(end_y - start_y)

        if rectangle_width < 10 or rectangle_height < 10:
            self.get_logger().warn("Selected target region is too small.")
            self.selected_tracking_rectangle = None
            self.tracking_state = TRACKING_STATE_SELECTING
            return

        self.selected_tracking_rectangle = (
            int(rectangle_x),
            int(rectangle_y),
            int(rectangle_width),
            int(rectangle_height),
        )

    def initialize_tracker_from_selected_region(self) -> bool:
        """
        根据已选择 ROI 初始化追踪器 / Initialize tracker from selected ROI.

        Returns:
            是否初始化成功 / Whether initialization succeeded.
        """
        if self.selected_tracking_rectangle is None:
            return False

        # current_image_frame: 当前图像帧 / Current image frame.
        current_image_frame = self._get_latest_image_copy()
        if current_image_frame is None:
            self.get_logger().warn("No image frame available when initializing tracker.")
            return False

        # normalized_rectangle: 修正到图像范围内的矩形 / Rectangle clipped inside image.
        normalized_rectangle = self._normalize_rectangle_to_image(
            self.selected_tracking_rectangle,
            current_image_frame.shape[1],
            current_image_frame.shape[0],
        )

        if normalized_rectangle is None:
            self.get_logger().warn("Selected target region is outside the image.")
            return False

        self.selected_tracking_rectangle = normalized_rectangle
        self.current_track_window = normalized_rectangle
        self._reset_filtered_target_state()
        self._reset_pd_control_state()

        if self.tracking_backend == "color_hsv":
            # sampled_hsv_threshold_range: ROI 采样得到的 HSV 阈值 / HSV thresholds sampled from ROI.
            sampled_hsv_threshold_range = self._sample_hsv_range_from_selected_region(
                current_image_frame,
                normalized_rectangle,
            )
            if sampled_hsv_threshold_range is not None:
                self._apply_hsv_range_to_ros_parameters(sampled_hsv_threshold_range)
                write_hsv_threshold_file(self.hsv_file, sampled_hsv_threshold_range)
                self.get_logger().info(
                    f"HSV sampled and saved: {sampled_hsv_threshold_range}, file={self.hsv_file}"
                )

        if self.tracking_backend == "opencv":
            self.is_opencv_tracker_available = self._initialize_opencv_tracker(
                current_image_frame,
                normalized_rectangle,
            )
            if not self.is_opencv_tracker_available:
                self.get_logger().warn("OpenCV tracker is unavailable, fallback to CamShift backend.")
                self.tracking_backend = "camshift"

        if self.tracking_backend != "opencv":
            if not self._initialize_camshift_tracker(current_image_frame, normalized_rectangle):
                return False

        self.tracking_state = TRACKING_STATE_READY
        self.get_logger().info(
            "Target selected. Press SPACE to start tracking. "
            f"ROI={self.selected_tracking_rectangle}"
        )
        return True

    def _initialize_camshift_tracker(
        self,
        image_frame: np.ndarray,
        target_rectangle: Tuple[int, int, int, int],
    ) -> bool:
        """
        初始化 CamShift 追踪器 / Initialize CamShift tracker.

        Args:
            image_frame: 输入图像 / Input image.
            target_rectangle: 目标矩形 / Target rectangle.

        Returns:
            是否初始化成功 / Whether initialization succeeded.
        """
        rectangle_x, rectangle_y, rectangle_width, rectangle_height = target_rectangle

        # selected_region_image: 目标 ROI 图像 / Target ROI image.
        selected_region_image = image_frame[
            rectangle_y : rectangle_y + rectangle_height,
            rectangle_x : rectangle_x + rectangle_width,
        ]

        if selected_region_image.size == 0:
            self.get_logger().warn("Selected target ROI is empty.")
            return False

        # selected_region_hsv: ROI 的 HSV 图像 / HSV image of ROI.
        selected_region_hsv = cv2.cvtColor(selected_region_image, cv2.COLOR_BGR2HSV)

        # selected_region_mask: ROI 颜色有效掩膜 / Valid color mask of ROI.
        selected_region_mask = cv2.inRange(
            selected_region_hsv,
            np.array([0, self.hsv_saturation_min, self.hsv_value_min], dtype=np.uint8),
            np.array([180, 255, self.hsv_value_max], dtype=np.uint8),
        )

        # camshift_histogram: ROI Hue 直方图 / ROI Hue histogram.
        camshift_histogram = cv2.calcHist(
            [selected_region_hsv],
            [0],
            selected_region_mask,
            [180],
            [0, 180],
        )

        if float(cv2.sumElems(camshift_histogram)[0]) <= 0.0:
            self.get_logger().warn(
                "Target histogram is empty. Try selecting a more colorful or brighter region."
            )
            return False

        cv2.normalize(camshift_histogram, camshift_histogram, 0, 255, cv2.NORM_MINMAX)
        self.camshift_histogram = camshift_histogram
        return True

    def _initialize_opencv_tracker(
        self,
        image_frame: np.ndarray,
        target_rectangle: Tuple[int, int, int, int],
    ) -> bool:
        """
        初始化 OpenCV 内置追踪器 / Initialize built-in OpenCV tracker.

        Args:
            image_frame: 输入图像 / Input image.
            target_rectangle: 目标矩形 / Target rectangle.

        Returns:
            是否初始化成功 / Whether initialization succeeded.
        """
        self.opencv_tracker = self._create_opencv_tracker_by_type(self.opencv_tracker_type)
        if self.opencv_tracker is None:
            return False

        try:
            # initialization_result: OpenCV 追踪器初始化结果 / OpenCV tracker initialization result.
            initialization_result = self.opencv_tracker.init(image_frame, target_rectangle)
            return bool(initialization_result) if initialization_result is not None else True
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"OpenCV tracker init failed: {exception_error}")
            self.opencv_tracker = None
            return False

    def _create_opencv_tracker_by_type(self, tracker_type_name: str):
        """
        按类型创建 OpenCV 追踪器 / Create OpenCV tracker by type.

        Args:
            tracker_type_name: 追踪器类型名 / Tracker type name.

        Returns:
            OpenCV 追踪器对象，失败返回 None / OpenCV tracker object or None.
        """
        # tracker_constructors: 追踪器构造函数候选列表 / Candidate tracker constructors.
        tracker_constructors = {
            "CSRT": ["TrackerCSRT_create", "legacy.TrackerCSRT_create"],
            "KCF": ["TrackerKCF_create", "legacy.TrackerKCF_create"],
            "MIL": ["TrackerMIL_create", "legacy.TrackerMIL_create"],
            "BOOSTING": ["TrackerBoosting_create", "legacy.TrackerBoosting_create"],
            "MOSSE": ["legacy.TrackerMOSSE_create"],
        }

        for constructor_path in tracker_constructors.get(tracker_type_name, []):
            # tracker_constructor: 追踪器构造函数 / Tracker constructor.
            tracker_constructor = self._get_nested_cv2_attribute(constructor_path)
            if tracker_constructor is not None:
                try:
                    return tracker_constructor()
                except Exception as exception_error:  # pylint: disable=broad-except
                    self.get_logger().warn(
                        f"Failed to create OpenCV tracker {constructor_path}: {exception_error}"
                    )

        return None

    @staticmethod
    def _get_nested_cv2_attribute(attribute_path: str):
        """
        获取 cv2 嵌套属性 / Get nested cv2 attribute.

        Args:
            attribute_path: 属性路径 / Attribute path.

        Returns:
            属性对象，失败返回 None / Attribute object or None.
        """
        # current_attribute: 当前属性对象 / Current attribute object.
        current_attribute = cv2
        for attribute_name in attribute_path.split("."):
            if not hasattr(current_attribute, attribute_name):
                return None
            current_attribute = getattr(current_attribute, attribute_name)
        return current_attribute

    def process_latest_image(self) -> None:
        """处理最新图像并更新目标状态 / Process latest image and update target state."""
        if self.is_shutdown_requested:
            return

        # current_image_frame: 当前处理图像 / Current image for processing.
        current_image_frame, current_image_timestamp_sec = self._get_latest_image_copy_with_timestamp()
        if current_image_frame is None:
            return

        # display_image_frame: 用于显示的图像 / Image frame for display.
        display_image_frame = current_image_frame.copy()

        # tracking_result: 当前追踪结果 / Current tracking result.
        tracking_result = TargetTrackingResult(False)

        if current_image_timestamp_sec > self.last_processed_image_timestamp_sec:
            self.last_processed_image_timestamp_sec = current_image_timestamp_sec
            tracking_result = self._process_new_image_frame(current_image_frame)
        else:
            tracking_result = self.last_tracking_result_for_display

        if self.show_debug_window:
            self._draw_debug_overlay(display_image_frame, tracking_result)
            # display_frame_with_optional_binary: 可选拼接二值图后的显示图像 / Display image optionally stacked with binary mask.
            display_frame_with_optional_binary = self._build_display_image_with_optional_binary(display_image_frame)
            cv2.imshow(self.window_name, display_frame_with_optional_binary)
            # key_code: OpenCV 按键编码 / OpenCV key code.
            key_code = cv2.waitKey(1) & 0xFF
            if key_code != 255:
                self.process_keyboard_command(key_code)

    def _process_new_image_frame(self, image_frame: np.ndarray) -> TargetTrackingResult:
        """
        处理新图像帧并更新目标滤波状态 / Process new image frame and update filtered target state.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            当前追踪结果 / Current tracking result.
        """
        # tracking_result: 当前追踪结果 / Current tracking result.
        tracking_result = TargetTrackingResult(False)

        if self.tracking_state in (TRACKING_STATE_TRACKING, TRACKING_STATE_IDENTIFY):
            tracking_result = self.update_target_tracking(image_frame)
            if tracking_result.is_tracking_successful:
                self._update_filtered_target_state(
                    tracking_result,
                    image_frame.shape[1],
                    image_frame.shape[0],
                )
            else:
                self._handle_tracking_update_failure(image_frame.shape[1], image_frame.shape[0])
        elif self.tracking_state in (TRACKING_STATE_SELECTING, TRACKING_STATE_READY, TRACKING_STATE_LOST):
            self._refresh_tracking_image_size(image_frame.shape[1], image_frame.shape[0])

        self.last_tracking_result_for_display = tracking_result
        return tracking_result

    def update_target_tracking(self, image_frame: np.ndarray) -> TargetTrackingResult:
        """
        更新目标追踪结果 / Update target tracking result.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            目标追踪结果 / Target tracking result.
        """
        if self.tracking_backend == "opencv" and self.opencv_tracker is not None:
            return self._update_opencv_tracking(image_frame)

        if self.tracking_backend == "color_hsv":
            return self._update_color_hsv_tracking(image_frame)

        return self._update_camshift_tracking(image_frame)

    def _update_opencv_tracking(self, image_frame: np.ndarray) -> TargetTrackingResult:
        """
        更新 OpenCV 内置追踪器 / Update built-in OpenCV tracker.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            目标追踪结果 / Target tracking result.
        """
        try:
            # is_tracking_successful/target_rectangle: OpenCV 更新结果 / OpenCV update result.
            is_tracking_successful, target_rectangle = self.opencv_tracker.update(image_frame)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"OpenCV tracker update failed: {exception_error}")
            return TargetTrackingResult(False)

        if not is_tracking_successful:
            return TargetTrackingResult(False)

        rectangle_x, rectangle_y, rectangle_width, rectangle_height = target_rectangle
        target_area = float(rectangle_width * rectangle_height)
        if target_area < self.tracking_min_area:
            return TargetTrackingResult(False)

        # target_center_x/target_center_y: 目标中心坐标 / Target center coordinates.
        target_center_x = rectangle_x + rectangle_width / 2.0
        target_center_y = rectangle_y + rectangle_height / 2.0

        # normalized_rectangle: 整数化目标矩形 / Integer target rectangle.
        normalized_rectangle = (
            int(rectangle_x),
            int(rectangle_y),
            int(rectangle_width),
            int(rectangle_height),
        )
        self.current_track_window = normalized_rectangle

        return TargetTrackingResult(True, target_center_x, target_center_y, normalized_rectangle, None)

    def _update_color_hsv_tracking(self, image_frame: np.ndarray) -> TargetTrackingResult:
        """使用 HSV 阈值分割更新颜色目标 / Update target with HSV threshold segmentation."""
        # hsv_image_frame: 当前帧 HSV 图像 / HSV image of current frame.
        hsv_image_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

        # hsv_lower_threshold: HSV 下限 / HSV lower threshold.
        hsv_lower_threshold = np.array(
            [self.hue_minimum, self.saturation_minimum, self.value_minimum],
            dtype=np.uint8,
        )

        # hsv_upper_threshold: HSV 上限 / HSV upper threshold.
        hsv_upper_threshold = np.array(
            [self.hue_maximum, self.saturation_maximum, self.value_maximum],
            dtype=np.uint8,
        )

        # binary_mask_image: HSV 分割二值图 / HSV segmentation binary mask.
        binary_mask_image = cv2.inRange(hsv_image_frame, hsv_lower_threshold, hsv_upper_threshold)

        # morphology_kernel_size: 形态学核尺寸 / Morphology kernel size.
        morphology_kernel_size = max(1, int(self.morph_kernel_size))
        if morphology_kernel_size % 2 == 0:
            morphology_kernel_size += 1

        # morphology_kernel: 形态学核 / Morphology kernel.
        morphology_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (morphology_kernel_size, morphology_kernel_size),
        )
        binary_mask_image = cv2.morphologyEx(binary_mask_image, cv2.MORPH_CLOSE, morphology_kernel)
        binary_mask_image = cv2.morphologyEx(binary_mask_image, cv2.MORPH_OPEN, morphology_kernel)
        self.latest_binary_mask_image = binary_mask_image

        # external_contours: 外部轮廓 / External contours.
        external_contours, _ = cv2.findContours(
            binary_mask_image,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not external_contours:
            return TargetTrackingResult(False)

        # largest_target_contour: 最大目标轮廓 / Largest target contour.
        largest_target_contour = max(external_contours, key=cv2.contourArea)

        # largest_target_area: 最大目标面积 / Largest target area.
        largest_target_area = float(cv2.contourArea(largest_target_contour))

        # minimum_area_rectangle: 最小外接旋转矩形 / Minimum area rotated rectangle.
        minimum_area_rectangle = cv2.minAreaRect(largest_target_contour)

        # target_polygon: 旋转矩形点集 / Rotated rectangle points.
        target_polygon = cv2.boxPoints(minimum_area_rectangle)
        target_polygon = np.intp(target_polygon)

        # 面积过滤 / Area filter.
        if largest_target_area < self.tracking_min_area:
            return TargetTrackingResult(False)

        # 半径过滤（参考 color_tracker_ptz_node）/ Radius filter (reference color_tracker_ptz_node).
        (enclosing_center_x, enclosing_center_y), enclosing_radius = cv2.minEnclosingCircle(target_polygon)
        if enclosing_radius < self.min_target_radius:
            return TargetTrackingResult(False)

        # bounding_rectangle: 外接矩形 / Bounding rectangle.
        bounding_rectangle = cv2.boundingRect(target_polygon)
        rectangle_x, rectangle_y, rectangle_width, rectangle_height = bounding_rectangle

        # target_center_x/target_center_y: 目标中心点 / Target center point.
        target_center_x, target_center_y = minimum_area_rectangle[0]
        if not self._is_point_inside_image(
            target_center_x,
            target_center_y,
            image_frame.shape[1],
            image_frame.shape[0],
        ):
            return TargetTrackingResult(False)

        return TargetTrackingResult(
            True,
            float(target_center_x),
            float(target_center_y),
            (int(rectangle_x), int(rectangle_y), int(rectangle_width), int(rectangle_height)),
            target_polygon,
        )

    def _update_camshift_tracking(self, image_frame: np.ndarray) -> TargetTrackingResult:
        """
        更新 CamShift 追踪器 / Update CamShift tracker.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            目标追踪结果 / Target tracking result.
        """
        if self.camshift_histogram is None or self.current_track_window is None:
            return TargetTrackingResult(False)

        # hsv_image_frame: 当前帧 HSV 图像 / HSV image of current frame.
        hsv_image_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

        # hsv_mask: 当前帧 HSV 掩膜 / HSV mask of current frame.
        hsv_mask = cv2.inRange(
            hsv_image_frame,
            np.array([0, self.hsv_saturation_min, self.hsv_value_min], dtype=np.uint8),
            np.array([180, 255, self.hsv_value_max], dtype=np.uint8),
        )

        # back_projection: Hue 反向投影 / Hue back projection.
        back_projection = cv2.calcBackProject(
            [hsv_image_frame],
            [0],
            self.camshift_histogram,
            [0, 180],
            1,
        )
        back_projection = cv2.bitwise_and(back_projection, back_projection, mask=hsv_mask)
        _, self.latest_binary_mask_image = cv2.threshold(back_projection, 10, 255, cv2.THRESH_BINARY)

        # tracking_term_criteria: CamShift 终止条件 / CamShift termination criteria.
        tracking_term_criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

        try:
            # rotated_rectangle/updated_track_window: CamShift 更新结果 / CamShift update result.
            rotated_rectangle, updated_track_window = cv2.CamShift(
                back_projection,
                self.current_track_window,
                tracking_term_criteria,
            )
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"CamShift update failed: {exception_error}")
            return TargetTrackingResult(False)

        self.current_track_window = updated_track_window

        # target_box_points: 旋转矩形顶点 / Rotated rectangle points.
        target_box_points = cv2.boxPoints(rotated_rectangle)
        target_box_points = np.intp(target_box_points)

        # bounding_rectangle: 旋转框外接矩形 / Bounding rectangle of rotated box.
        bounding_rectangle = cv2.boundingRect(target_box_points)
        rectangle_x, rectangle_y, rectangle_width, rectangle_height = bounding_rectangle

        target_area = float(rectangle_width * rectangle_height)
        if target_area < self.tracking_min_area:
            return TargetTrackingResult(False)

        # target_center_x/target_center_y: 目标中心坐标 / Target center coordinates.
        target_center_x, target_center_y = rotated_rectangle[0]
        if not self._is_point_inside_image(
            target_center_x,
            target_center_y,
            image_frame.shape[1],
            image_frame.shape[0],
        ):
            return TargetTrackingResult(False)

        return TargetTrackingResult(
            True,
            float(target_center_x),
            float(target_center_y),
            (int(rectangle_x), int(rectangle_y), int(rectangle_width), int(rectangle_height)),
            target_box_points,
        )

    def _update_filtered_target_state(
        self,
        tracking_result: TargetTrackingResult,
        image_width: int,
        image_height: int,
    ) -> None:
        """
        更新滤波目标状态 / Update filtered target state.

        Args:
            tracking_result: 目标追踪结果 / Target tracking result.
            image_width: 图像宽度 / Image width.
            image_height: 图像高度 / Image height.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.monotonic()

        with self.target_state_lock:
            self.raw_target_rectangle = tracking_result.target_rectangle
            self.raw_target_polygon = tracking_result.target_polygon
            self.latest_tracking_image_width = int(image_width)
            self.latest_tracking_image_height = int(image_height)

            if self.filtered_target_center_x is None or self.filtered_target_center_y is None:
                self.filtered_target_center_x = float(tracking_result.target_center_x)
                self.filtered_target_center_y = float(tracking_result.target_center_y)
            else:
                self.filtered_target_center_x = (
                    (1.0 - self.target_smooth_alpha) * self.filtered_target_center_x
                    + self.target_smooth_alpha * float(tracking_result.target_center_x)
                )
                self.filtered_target_center_y = (
                    (1.0 - self.target_smooth_alpha) * self.filtered_target_center_y
                    + self.target_smooth_alpha * float(tracking_result.target_center_y)
                )

            self.target_is_valid = True
            self.last_valid_target_time_sec = current_time_sec

    def _handle_tracking_update_failure(self, image_width: int, image_height: int) -> None:
        """
        处理追踪更新失败 / Handle tracking update failure.

        Args:
            image_width: 图像宽度 / Image width.
            image_height: 图像高度 / Image height.
        """
        self._refresh_tracking_image_size(image_width, image_height)

        # target_age_sec: 目标距离上次有效时间 / Target age since last valid update.
        target_age_sec = time.monotonic() - self.last_valid_target_time_sec
        if target_age_sec <= self.target_lost_timeout_sec:
            return

        with self.target_state_lock:
            self.target_is_valid = False
            self.filtered_target_center_x = None
            self.filtered_target_center_y = None

        if self.tracking_state != TRACKING_STATE_LOST:
            self.tracking_state = TRACKING_STATE_LOST
            self._reset_pd_control_state()
            self.get_logger().warn("Target lost. Press r to reselect target.")

    def _refresh_tracking_image_size(self, image_width: int, image_height: int) -> None:
        """
        刷新追踪图像尺寸 / Refresh tracking image size.

        Args:
            image_width: 图像宽度 / Image width.
            image_height: 图像高度 / Image height.
        """
        with self.target_state_lock:
            self.latest_tracking_image_width = int(image_width)
            self.latest_tracking_image_height = int(image_height)

    def _reset_filtered_target_state(self) -> None:
        """重置滤波目标状态 / Reset filtered target state."""
        with self.target_state_lock:
            self.raw_target_rectangle = (0, 0, 0, 0)
            self.raw_target_polygon = None
            self.filtered_target_center_x = None
            self.filtered_target_center_y = None
            self.target_is_valid = False
            self.last_valid_target_time_sec = time.monotonic()

    def _reset_pd_control_state(self) -> None:
        """重置 PD 控制历史状态 / Reset PD control history."""
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

    def ptz_control_timer_callback(self) -> None:
        """固定频率云台控制回调 / Fixed-rate PTZ control callback."""
        if self.is_shutdown_requested or self.tracking_state != TRACKING_STATE_TRACKING:
            return

        # target_state_snapshot: 目标状态快照 / Target state snapshot.
        target_state_snapshot = self._get_target_state_snapshot()
        if not self._is_target_snapshot_usable(target_state_snapshot):
            self._handle_unusable_target_snapshot(target_state_snapshot)
            return

        self.calculate_ptz_angle_command(target_state_snapshot)
        self.publish_ptz_angle_command()

    def _get_target_state_snapshot(self) -> TargetStateSnapshot:
        """
        获取目标状态快照 / Get target state snapshot.

        Returns:
            目标状态快照 / Target state snapshot.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.monotonic()

        with self.target_state_lock:
            return TargetStateSnapshot(
                bool(self.target_is_valid),
                current_time_sec - self.last_valid_target_time_sec,
                self.filtered_target_center_x,
                self.filtered_target_center_y,
                int(self.latest_tracking_image_width),
                int(self.latest_tracking_image_height),
            )

    def _is_target_snapshot_usable(self, target_state_snapshot: TargetStateSnapshot) -> bool:
        """
        判断目标快照是否可用于控制 / Check whether target snapshot is usable for control.

        Args:
            target_state_snapshot: 目标状态快照 / Target state snapshot.

        Returns:
            是否可用于控制 / Whether usable for control.
        """
        return (
            target_state_snapshot.target_is_valid
            and target_state_snapshot.filtered_target_center_x is not None
            and target_state_snapshot.filtered_target_center_y is not None
            and target_state_snapshot.image_width > 0
            and target_state_snapshot.image_height > 0
            and target_state_snapshot.target_age_sec <= self.target_lost_timeout_sec
        )

    def _handle_unusable_target_snapshot(self, target_state_snapshot: TargetStateSnapshot) -> None:
        """
        处理不可用目标快照 / Handle unusable target snapshot.

        Args:
            target_state_snapshot: 目标状态快照 / Target state snapshot.
        """
        self._reset_pd_control_state()

        if self.lost_target_stop and target_state_snapshot.target_age_sec > self.target_lost_timeout_sec:
            with self.target_state_lock:
                self.target_is_valid = False
                self.filtered_target_center_x = None
                self.filtered_target_center_y = None

    def calculate_ptz_angle_command(self, target_state_snapshot: TargetStateSnapshot) -> None:
        """
        使用 PD 控制根据目标像素误差计算云台角度命令（无积分项，归一化误差）。
        Calculate PTZ angle command from pixel error using PD control (no integral, normalized error).

        Args:
            target_state_snapshot: 目标状态快照 / Target state snapshot.
        """
        # 图像中心 / Image center.
        image_center_x = float(target_state_snapshot.image_width) * 0.5
        image_center_y = float(target_state_snapshot.image_height) * 0.5

        # 像素误差 / Pixel error.
        pixel_error_x = float(target_state_snapshot.filtered_target_center_x) - image_center_x
        pixel_error_y = float(target_state_snapshot.filtered_target_center_y) - image_center_y

        # 死区判断 / Deadband check.
        target_is_centered_x = abs(pixel_error_x) < self.deadband_x
        target_is_centered_y = abs(pixel_error_y) < self.deadband_y
        if target_is_centered_x:
            pixel_error_x = 0.0
        if target_is_centered_y:
            pixel_error_y = 0.0

        # 控制时间间隔 / Control time delta.
        current_time_sec = time.monotonic()
        if self.previous_control_time > 0.0:
            delta_time_sec = current_time_sec - self.previous_control_time
        else:
            delta_time_sec = 1.0 / max(float(self.control_rate_hz), MINIMUM_TIMER_RATE_HZ)
        delta_time_sec = max(delta_time_sec, 1.0e-3)

        # 按图像半尺寸归一化误差 / Normalize error by half image size.
        normalized_error_x = pixel_error_x / image_center_x
        normalized_error_y = pixel_error_y / image_center_y

        # PD 微分项 / Derivative term.
        derivative_x = (normalized_error_x - self.previous_error_x) / delta_time_sec
        derivative_y = (normalized_error_y - self.previous_error_y) / delta_time_sec

        # 原始 PD 步进 / Raw PD step.
        raw_step_x = (self.pd_kp * normalized_error_x + self.pd_kd * derivative_x) * self.max_ptz_step
        raw_step_y = (self.pd_kp * normalized_error_y + self.pd_kd * derivative_y) * self.max_ptz_step
        raw_step_x = clamp_numeric_value(raw_step_x, -self.max_ptz_step, self.max_ptz_step)
        raw_step_y = clamp_numeric_value(raw_step_y, -self.max_ptz_step, self.max_ptz_step)

        # 步进平滑 / Step smoothing.
        step_smoothing_alpha = self.ptz_step_smooth_alpha
        step_x = (1.0 - step_smoothing_alpha) * self.last_step_x + step_smoothing_alpha * raw_step_x
        step_y = (1.0 - step_smoothing_alpha) * self.last_step_y + step_smoothing_alpha * raw_step_y

        # 死区内强制归零 / Force zero inside deadband.
        if target_is_centered_x:
            step_x = 0.0
            normalized_error_x = 0.0
        if target_is_centered_y:
            step_y = 0.0
            normalized_error_y = 0.0

        self.previous_error_x = normalized_error_x
        self.previous_error_y = normalized_error_y
        self.previous_control_time = current_time_sec
        self.last_step_x = step_x
        self.last_step_y = step_y

        # 应用方向反转 / Apply direction reversal.
        if self.reverse_horizontal_direction:
            step_x = -step_x
        if self.reverse_pitch_direction:
            step_y = -step_y

        # 期望角度 = 当前 + 步进 / Desired angle = current + step.
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

        # 云台角度平滑 / PTZ angle smoothing.
        ptz_smoothing_alpha = self.ptz_smooth_alpha
        self.horizontal_angle_deg = (
            (1.0 - ptz_smoothing_alpha) * self.horizontal_angle_deg
            + ptz_smoothing_alpha * desired_horizontal_angle_deg
        )
        self.pitch_angle_deg = (
            (1.0 - ptz_smoothing_alpha) * self.pitch_angle_deg
            + ptz_smoothing_alpha * desired_pitch_angle_deg
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

        self._log_debug_control_state(pixel_error_x, pixel_error_y)

    def publish_ptz_angle_command(self) -> None:
        """发布云台角度命令 / Publish PTZ angle command."""
        # ptz_angle_message: 云台角度消息 / PTZ angle message.
        ptz_angle_message = Vector3()
        ptz_angle_message.x = float(self.horizontal_angle_deg)
        ptz_angle_message.y = float(self.pitch_angle_deg)
        ptz_angle_message.z = 0.0
        self.ptz_angle_publisher.publish(ptz_angle_message)

    def _log_debug_control_state(self, horizontal_error_pixel: float, pitch_error_pixel: float) -> None:
        """
        按固定间隔打印调试控制信息 / Log debug control state periodically.

        Args:
            horizontal_error_pixel: 水平像素误差 / Horizontal pixel error.
            pitch_error_pixel: 俯仰像素误差 / Pitch pixel error.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.monotonic()

        if self.print_ptz_angle and self.print_interval_sec >= 0.0:
            # print_interval_sec: 终端打印间隔 / Terminal print interval.
            print_interval_sec = max(float(self.print_interval_sec), 0.01)
            if current_time_sec - self.last_ptz_print_time_sec >= print_interval_sec:
                self.last_ptz_print_time_sec = current_time_sec
                self.get_logger().info(
                    f"PTZ angle debug: error=({horizontal_error_pixel:.1f}, {pitch_error_pixel:.1f}), "
                    f"s1={self.horizontal_angle_deg:.2f}deg, s2={self.pitch_angle_deg:.2f}deg"
                )

        if self.debug_log_interval_sec <= 0.0:
            return

        if current_time_sec - self.last_debug_log_time_sec < self.debug_log_interval_sec:
            return

        self.last_debug_log_time_sec = current_time_sec
        self.get_logger().debug(
            f"PTZ control error=({horizontal_error_pixel:.1f}, {pitch_error_pixel:.1f}), "
            f"angle=({self.horizontal_angle_deg:.1f}, {self.pitch_angle_deg:.1f})"
        )

    def process_keyboard_command(self, key_code: int) -> None:
        """
        处理键盘按键 / Process keyboard command.

        Args:
            key_code: OpenCV 按键编码 / OpenCV key code.
        """
        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self.request_node_shutdown()
            return

        if key_code in (ord("r"), ord("R")):
            self.reset_target_selection()
            return

        if key_code in (ord("i"), ord("I")):
            if self.selected_tracking_rectangle is None:
                self.get_logger().info("Please drag mouse to select target before identify mode.")
                return
            if self.tracking_state == TRACKING_STATE_SELECTING:
                self.initialize_tracker_from_selected_region()
            self.tracking_state = TRACKING_STATE_IDENTIFY
            self._reset_pd_control_state()
            self.get_logger().info("Identify mode enabled: tracking target without PTZ control.")
            return

        if key_code == KEY_CODE_SPACE:
            if self.tracking_state in (TRACKING_STATE_READY, TRACKING_STATE_IDENTIFY):
                self.tracking_state = TRACKING_STATE_TRACKING
                self._reset_pd_control_state()
                self.get_logger().info("Tracking started.")
            elif self.tracking_state == TRACKING_STATE_TRACKING:
                self.tracking_state = TRACKING_STATE_READY
                self._reset_pd_control_state()
                self.get_logger().info("Tracking paused.")
            elif self.tracking_state == TRACKING_STATE_LOST:
                self.get_logger().warn("Target is lost. Press r and select target again.")
            else:
                self.get_logger().info("Please drag mouse to select target first.")

    def reset_target_selection(self) -> None:
        """重置目标选择状态 / Reset target selection state."""
        self.selected_region_start_point = None
        self.selected_region_end_point = None
        self.selected_tracking_rectangle = None
        self.camshift_histogram = None
        self.current_track_window = None
        self.opencv_tracker = None
        self.is_opencv_tracker_available = False
        self.latest_binary_mask_image = None
        self.is_mouse_dragging = False
        self.tracking_state = TRACKING_STATE_SELECTING
        self._reset_pd_control_state()
        self._reset_filtered_target_state()
        self.get_logger().info("Target selection reset. Drag mouse to select target.")

    def on_dynamic_parameter_update(self, parameter_list: list[Parameter]) -> SetParametersResult:
        """
        rqt/ROS2 动态参数更新回调 / Dynamic parameter update callback for rqt/ROS2.

        Args:
            parameter_list: 参数列表 / Parameter list.

        Returns:
            参数更新结果 / Parameter update result.
        """
        if not parameter_list:
            return SetParametersResult(successful=True, reason="No parameter changed.")

        # changed_parameter_names: 发生变化的参数名称 / Changed parameter names.
        changed_parameter_names = {parameter.name for parameter in parameter_list}

        # unsupported_parameter_names: 不支持的参数名称 / Unsupported parameter names.
        unsupported_parameter_names = changed_parameter_names - DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if unsupported_parameter_names:
            return SetParametersResult(
                successful=False,
                reason=f"Unsupported runtime parameters: {sorted(unsupported_parameter_names)}",
            )

        # restart_required_parameter_names: 需要重启的参数名称 / Restart-required parameter names.
        restart_required_parameter_names = changed_parameter_names & RESTART_REQUIRED_PARAMETER_NAMES
        if restart_required_parameter_names:
            return SetParametersResult(
                successful=False,
                reason=f"These parameters require node restart: {sorted(restart_required_parameter_names)}",
            )

        # parameter_values: 新旧合并后的参数字典 / Merged parameter dictionary.
        parameter_values = self._build_runtime_parameter_values(parameter_list)

        # dynamic_parameter_enabled: 动态调参开关 / Dynamic tuning switch.
        dynamic_parameter_enabled = convert_value_to_boolean(
            parameter_values["enable_rqt_dynamic_parameters"],
            self.enable_rqt_dynamic_parameters,
        )
        if not dynamic_parameter_enabled and changed_parameter_names != {"enable_rqt_dynamic_parameters"}:
            return SetParametersResult(
                successful=False,
                reason="Runtime dynamic parameters are disabled by enable_rqt_dynamic_parameters.",
            )

        # validation_result: 参数校验结果 / Parameter validation result.
        validation_result = self._validate_dynamic_parameters(parameter_values)
        if not validation_result.successful:
            return validation_result

        # previous_show_debug_window: 更新前窗口开关 / Previous window switch.
        previous_show_debug_window = self.show_debug_window

        # previous_window_size: 更新前窗口大小 / Previous window size.
        previous_window_size = (self.window_width, self.window_height)

        # previous_control_rate_hz: 更新前控制频率 / Previous control rate.
        previous_control_rate_hz = self.control_rate_hz

        # previous_image_processing_rate_hz: 更新前图像处理频率 / Previous image processing rate.
        previous_image_processing_rate_hz = self.image_processing_rate_hz

        self._apply_dynamic_parameter_values(parameter_values, changed_parameter_names)

        if abs(self.control_rate_hz - previous_control_rate_hz) > 1.0e-9:
            self._recreate_ptz_control_timer(True)

        if abs(self.image_processing_rate_hz - previous_image_processing_rate_hz) > 1.0e-9:
            self._recreate_image_processing_timer(True)

        self._synchronize_debug_window_after_parameter_update(
            previous_show_debug_window,
            previous_window_size,
        )

        return SetParametersResult(successful=True, reason="Runtime parameters updated.")

    def _build_runtime_parameter_values(self, parameter_list: list[Parameter]) -> Dict[str, Any]:
        """
        构建运行时参数值字典 / Build runtime parameter value dictionary.

        Args:
            parameter_list: 参数列表 / Parameter list.

        Returns:
            参数值字典 / Parameter value dictionary.
        """
        # parameter_values: 参数值字典 / Parameter value dictionary.
        parameter_values: Dict[str, Any] = {
            "show_debug_window": self.show_debug_window,
            "window_width": self.window_width,
            "window_height": self.window_height,
            "overlay_font_scale": self.overlay_font_scale,
            "overlay_line_height": self.overlay_line_height,
            "overlay_margin_x": self.overlay_margin_x,
            "overlay_margin_y": self.overlay_margin_y,
            "overlay_show_background": self.overlay_show_background,
            "show_binary_window": self.show_binary_window,
            "img_flip": self.img_flip,
            "enable_horizontal_flip": self.enable_horizontal_flip,
            "enable_vertical_flip": self.enable_vertical_flip,
            "rotate_image_code": self.rotate_image_code,
            "horizontal_min_deg": self.horizontal_min_deg,
            "horizontal_max_deg": self.horizontal_max_deg,
            "pitch_min_deg": self.pitch_min_deg,
            "pitch_max_deg": self.pitch_max_deg,
            "reverse_horizontal_direction": self.reverse_horizontal_direction,
            "reverse_pitch_direction": self.reverse_pitch_direction,
            "deadband_x": self.deadband_x,
            "deadband_y": self.deadband_y,
            "pd_kp": self.pd_kp,
            "pd_kd": self.pd_kd,
            "ptz_step_smooth_alpha": self.ptz_step_smooth_alpha,
            "max_ptz_step": self.max_ptz_step,
            "target_smooth_alpha": self.target_smooth_alpha,
            "ptz_smooth_alpha": self.ptz_smooth_alpha,
            "control_rate_hz": self.control_rate_hz,
            "image_processing_rate_hz": self.image_processing_rate_hz,
            "tracking_min_area": self.tracking_min_area,
            "tracking_backend": self.tracking_backend,
            "opencv_tracker_type": self.opencv_tracker_type,
            "hsv_saturation_min": self.hsv_saturation_min,
            "hsv_value_min": self.hsv_value_min,
            "hsv_value_max": self.hsv_value_max,
            "Hmin": self.hue_minimum,
            "Smin": self.saturation_minimum,
            "Vmin": self.value_minimum,
            "Hmax": self.hue_maximum,
            "Smax": self.saturation_maximum,
            "Vmax": self.value_maximum,
            "h_expand": self.hue_expand,
            "s_expand": self.saturation_expand,
            "v_expand": self.value_expand,
            "morph_kernel_size": self.morph_kernel_size,
            "min_target_radius": self.min_target_radius,
            "image_width": self.image_width,
            "image_height": self.image_height,
            "hsv_file": self.hsv_file,
            "target_lost_timeout_sec": self.target_lost_timeout_sec,
            "lost_target_stop": self.lost_target_stop,
            "publish_initial_ptz_command": self.publish_initial_ptz_command,
            "initial_publish_count": self.initial_publish_count,
            "initial_publish_interval_sec": self.initial_publish_interval_sec,
            "debug_log_interval_sec": self.debug_log_interval_sec,
            "print_ptz_angle": self.print_ptz_angle,
            "print_interval_sec": self.print_interval_sec,
            "use_header_stamp_for_fps": self.use_header_stamp_for_fps,
            "show_image_stamp_delay": self.show_image_stamp_delay,
            "enable_rqt_dynamic_parameters": self.enable_rqt_dynamic_parameters,
        }

        for parameter in parameter_list:
            if parameter.type_ == Parameter.Type.NOT_SET:
                parameter_values[parameter.name] = None
            else:
                parameter_values[parameter.name] = parameter.value

        return parameter_values

    def _apply_dynamic_parameter_values(
        self,
        parameter_values: Dict[str, Any],
        changed_parameter_names: set[str],
    ) -> None:
        """
        应用动态参数值 / Apply dynamic parameter values.

        Args:
            parameter_values: 参数值字典 / Parameter value dictionary.
            changed_parameter_names: 变化参数名称集合 / Changed parameter names.
        """
        self.show_debug_window = convert_value_to_boolean(parameter_values["show_debug_window"], True)
        self.window_width = int(parameter_values["window_width"])
        self.window_height = int(parameter_values["window_height"])
        self.overlay_font_scale = float(parameter_values["overlay_font_scale"])
        self.overlay_line_height = int(parameter_values["overlay_line_height"])
        self.overlay_margin_x = int(parameter_values["overlay_margin_x"])
        self.overlay_margin_y = int(parameter_values["overlay_margin_y"])
        self.overlay_show_background = convert_value_to_boolean(
            parameter_values["overlay_show_background"],
            True,
        )
        self.show_binary_window = convert_value_to_boolean(
            parameter_values["show_binary_window"],
            False,
        )
        self.img_flip = convert_value_to_boolean(parameter_values["img_flip"], False)
        self.enable_horizontal_flip = convert_value_to_boolean(
            parameter_values["enable_horizontal_flip"],
            False,
        )
        self.enable_vertical_flip = convert_value_to_boolean(
            parameter_values["enable_vertical_flip"],
            False,
        )
        self.rotate_image_code = str(parameter_values["rotate_image_code"]).lower()

        self.horizontal_min_deg = float(parameter_values["horizontal_min_deg"])
        self.horizontal_max_deg = float(parameter_values["horizontal_max_deg"])
        self.pitch_min_deg = float(parameter_values["pitch_min_deg"])
        self.pitch_max_deg = float(parameter_values["pitch_max_deg"])

        self.reverse_horizontal_direction = convert_value_to_boolean(
            parameter_values["reverse_horizontal_direction"],
            self.reverse_horizontal_direction,
        )
        self.reverse_pitch_direction = convert_value_to_boolean(
            parameter_values["reverse_pitch_direction"],
            self.reverse_pitch_direction,
        )
        self.deadband_x = float(parameter_values["deadband_x"])
        self.deadband_y = float(parameter_values["deadband_y"])
        self.pd_kp = max(float(parameter_values["pd_kp"]), 0.0)
        self.pd_kd = max(float(parameter_values["pd_kd"]), 0.0)
        self.ptz_step_smooth_alpha = clamp_numeric_value(
            float(parameter_values["ptz_step_smooth_alpha"]),
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
        )
        self.max_ptz_step = float(parameter_values["max_ptz_step"])
        self.target_smooth_alpha = clamp_numeric_value(
            float(parameter_values["target_smooth_alpha"]),
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
        )
        self.ptz_smooth_alpha = clamp_numeric_value(
            float(parameter_values["ptz_smooth_alpha"]),
            MINIMUM_ALPHA_VALUE,
            MAXIMUM_ALPHA_VALUE,
        )
        self.control_rate_hz = float(parameter_values["control_rate_hz"])
        self.image_processing_rate_hz = float(parameter_values["image_processing_rate_hz"])
        self.tracking_min_area = float(parameter_values["tracking_min_area"])
        self.tracking_backend = str(parameter_values["tracking_backend"]).lower()
        self.opencv_tracker_type = str(parameter_values["opencv_tracker_type"]).upper()
        self.hsv_saturation_min = int(parameter_values["hsv_saturation_min"])
        self.hsv_value_min = int(parameter_values["hsv_value_min"])
        self.hsv_value_max = int(parameter_values["hsv_value_max"])
        self.hue_minimum = int(parameter_values["Hmin"])
        self.saturation_minimum = int(parameter_values["Smin"])
        self.value_minimum = int(parameter_values["Vmin"])
        self.hue_maximum = int(parameter_values["Hmax"])
        self.saturation_maximum = int(parameter_values["Smax"])
        self.value_maximum = int(parameter_values["Vmax"])
        self.hue_expand = int(parameter_values["h_expand"])
        self.saturation_expand = int(parameter_values["s_expand"])
        self.value_expand = int(parameter_values["v_expand"])
        self.morph_kernel_size = int(parameter_values["morph_kernel_size"])
        self.min_target_radius = float(parameter_values["min_target_radius"])
        self.image_width = int(parameter_values["image_width"])
        self.image_height = int(parameter_values["image_height"])
        self.hsv_file = os.path.expanduser(str(parameter_values["hsv_file"]))
        self.target_lost_timeout_sec = max(
            float(parameter_values["target_lost_timeout_sec"]),
            MINIMUM_TARGET_LOST_TIMEOUT_SEC,
        )
        self.lost_target_stop = convert_value_to_boolean(parameter_values["lost_target_stop"], True)
        self.publish_initial_ptz_command = convert_value_to_boolean(
            parameter_values["publish_initial_ptz_command"],
            True,
        )
        self.initial_publish_count = int(parameter_values["initial_publish_count"])
        self.initial_publish_interval_sec = float(parameter_values["initial_publish_interval_sec"])
        self.debug_log_interval_sec = float(parameter_values["debug_log_interval_sec"])
        self.print_ptz_angle = convert_value_to_boolean(parameter_values["print_ptz_angle"], False)
        self.print_interval_sec = float(parameter_values["print_interval_sec"])
        self.use_header_stamp_for_fps = convert_value_to_boolean(
            parameter_values["use_header_stamp_for_fps"],
            True,
        )
        self.show_image_stamp_delay = convert_value_to_boolean(
            parameter_values["show_image_stamp_delay"],
            True,
        )
        self.enable_rqt_dynamic_parameters = convert_value_to_boolean(
            parameter_values["enable_rqt_dynamic_parameters"],
            True,
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

        self._reset_pd_control_state()

    def _validate_dynamic_parameters(self, parameter_values: Dict[str, Any]) -> SetParametersResult:
        """
        校验动态参数合法性 / Validate dynamic parameters.

        Args:
            parameter_values: 参数字典 / Parameter dictionary.

        Returns:
            参数校验结果 / Parameter validation result.
        """
        try:
            if float(parameter_values["horizontal_min_deg"]) >= float(parameter_values["horizontal_max_deg"]):
                return SetParametersResult(successful=False, reason="horizontal_min_deg must be smaller than horizontal_max_deg.")
            if float(parameter_values["pitch_min_deg"]) >= float(parameter_values["pitch_max_deg"]):
                return SetParametersResult(successful=False, reason="pitch_min_deg must be smaller than pitch_max_deg.")
            if float(parameter_values["deadband_x"]) < 0.0:
                return SetParametersResult(successful=False, reason="deadband_x must be non-negative.")
            if float(parameter_values["deadband_y"]) < 0.0:
                return SetParametersResult(successful=False, reason="deadband_y must be non-negative.")
            if float(parameter_values["pd_kp"]) < 0.0:
                return SetParametersResult(successful=False, reason="pd_kp must be non-negative.")
            if float(parameter_values["pd_kd"]) < 0.0:
                return SetParametersResult(successful=False, reason="pd_kd must be non-negative.")
            if not (0.0 <= float(parameter_values["ptz_step_smooth_alpha"]) <= 1.0):
                return SetParametersResult(successful=False, reason="ptz_step_smooth_alpha must be in [0.0, 1.0].")
            if float(parameter_values["max_ptz_step"]) <= 0.0:
                return SetParametersResult(successful=False, reason="max_ptz_step must be greater than 0.")
            if not MINIMUM_ALPHA_VALUE <= float(parameter_values["target_smooth_alpha"]) <= MAXIMUM_ALPHA_VALUE:
                return SetParametersResult(successful=False, reason="target_smooth_alpha must be in [0.01, 1.0].")
            if not MINIMUM_ALPHA_VALUE <= float(parameter_values["ptz_smooth_alpha"]) <= MAXIMUM_ALPHA_VALUE:
                return SetParametersResult(successful=False, reason="ptz_smooth_alpha must be in [0.01, 1.0].")
            if float(parameter_values["control_rate_hz"]) < MINIMUM_TIMER_RATE_HZ:
                return SetParametersResult(successful=False, reason="control_rate_hz must be >= 1.0.")
            if float(parameter_values["image_processing_rate_hz"]) < MINIMUM_TIMER_RATE_HZ:
                return SetParametersResult(successful=False, reason="image_processing_rate_hz must be >= 1.0.")
            if float(parameter_values["tracking_min_area"]) < 0.0:
                return SetParametersResult(successful=False, reason="tracking_min_area must be non-negative.")
            if str(parameter_values["tracking_backend"]).lower() not in ("camshift", "opencv", "color_hsv"):
                return SetParametersResult(successful=False, reason="tracking_backend must be camshift, opencv or color_hsv.")
            if int(parameter_values["hsv_saturation_min"]) < 0 or int(parameter_values["hsv_saturation_min"]) > 255:
                return SetParametersResult(successful=False, reason="hsv_saturation_min must be in [0, 255].")
            if int(parameter_values["hsv_value_min"]) < 0 or int(parameter_values["hsv_value_min"]) > 255:
                return SetParametersResult(successful=False, reason="hsv_value_min must be in [0, 255].")
            if int(parameter_values["hsv_value_max"]) < 0 or int(parameter_values["hsv_value_max"]) > 255:
                return SetParametersResult(successful=False, reason="hsv_value_max must be in [0, 255].")
            if int(parameter_values["hsv_value_min"]) > int(parameter_values["hsv_value_max"]):
                return SetParametersResult(successful=False, reason="hsv_value_min must not be greater than hsv_value_max.")
            if int(parameter_values["Hmin"]) < 0 or int(parameter_values["Hmin"]) > 179:
                return SetParametersResult(successful=False, reason="Hmin must be in [0, 179].")
            if int(parameter_values["Hmax"]) < 0 or int(parameter_values["Hmax"]) > 179:
                return SetParametersResult(successful=False, reason="Hmax must be in [0, 179].")
            if int(parameter_values["Hmin"]) > int(parameter_values["Hmax"]):
                return SetParametersResult(successful=False, reason="Hmin must not be greater than Hmax.")
            for hsv_parameter_name in ("Smin", "Vmin", "Smax", "Vmax"):
                if int(parameter_values[hsv_parameter_name]) < 0 or int(parameter_values[hsv_parameter_name]) > 255:
                    return SetParametersResult(successful=False, reason=f"{hsv_parameter_name} must be in [0, 255].")
            if int(parameter_values["Smin"]) > int(parameter_values["Smax"]):
                return SetParametersResult(successful=False, reason="Smin must not be greater than Smax.")
            if int(parameter_values["Vmin"]) > int(parameter_values["Vmax"]):
                return SetParametersResult(successful=False, reason="Vmin must not be greater than Vmax.")
            if int(parameter_values["h_expand"]) < 0 or int(parameter_values["s_expand"]) < 0 or int(parameter_values["v_expand"]) < 0:
                return SetParametersResult(successful=False, reason="HSV expand values must be non-negative.")
            if int(parameter_values["morph_kernel_size"]) <= 0:
                return SetParametersResult(successful=False, reason="morph_kernel_size must be greater than 0.")
            if float(parameter_values["min_target_radius"]) < 0.0:
                return SetParametersResult(successful=False, reason="min_target_radius must be non-negative.")
            if int(parameter_values["image_width"]) <= 0:
                return SetParametersResult(successful=False, reason="image_width must be greater than 0.")
            if int(parameter_values["image_height"]) <= 0:
                return SetParametersResult(successful=False, reason="image_height must be greater than 0.")
            if str(parameter_values["rotate_image_code"]).lower() not in ("none", "90_clockwise", "cw90", "90_counterclockwise", "ccw90", "180", "rotate_180"):
                return SetParametersResult(successful=False, reason="rotate_image_code is invalid.")
            if int(parameter_values["initial_publish_count"]) < 0:
                return SetParametersResult(successful=False, reason="initial_publish_count must be non-negative.")
            if float(parameter_values["initial_publish_interval_sec"]) <= 0.0:
                return SetParametersResult(successful=False, reason="initial_publish_interval_sec must be greater than 0.")
            if float(parameter_values["print_interval_sec"]) < 0.0:
                return SetParametersResult(successful=False, reason="print_interval_sec must be non-negative.")
            if float(parameter_values["target_lost_timeout_sec"]) < MINIMUM_TARGET_LOST_TIMEOUT_SEC:
                return SetParametersResult(successful=False, reason="target_lost_timeout_sec is too small.")
            if int(parameter_values["window_width"]) <= 0 or int(parameter_values["window_height"]) <= 0:
                return SetParametersResult(successful=False, reason="window_width/window_height must be greater than 0.")
            if float(parameter_values["overlay_font_scale"]) <= 0.0:
                return SetParametersResult(successful=False, reason="overlay_font_scale must be greater than 0.")
            if int(parameter_values["overlay_line_height"]) <= 0:
                return SetParametersResult(successful=False, reason="overlay_line_height must be greater than 0.")
            if int(parameter_values["overlay_margin_x"]) < 0 or int(parameter_values["overlay_margin_y"]) < 0:
                return SetParametersResult(successful=False, reason="overlay margins must be non-negative.")
        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=f"Invalid parameter value: {exception_error}")

        return SetParametersResult(successful=True)

    def _synchronize_debug_window_after_parameter_update(
        self,
        previous_show_debug_window: bool,
        previous_window_size: Tuple[int, int],
    ) -> None:
        """
        动态参数更新后同步调试窗口 / Synchronize debug window after dynamic update.

        Args:
            previous_show_debug_window: 更新前窗口开关 / Previous window switch.
            previous_window_size: 更新前窗口尺寸 / Previous window size.
        """
        try:
            # window_size_changed: 窗口尺寸是否变化 / Whether window size changed.
            window_size_changed = previous_window_size != (self.window_width, self.window_height)
            if self.show_debug_window and not previous_show_debug_window:
                self._initialize_debug_window()
            elif not self.show_debug_window and previous_show_debug_window:
                cv2.destroyWindow(self.window_name)
            elif self.show_debug_window and window_size_changed:
                cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Synchronize debug window failed: {exception_error}")

    def _draw_debug_overlay(
        self,
        display_image_frame: np.ndarray,
        tracking_result: TargetTrackingResult,
    ) -> None:
        """
        绘制调试信息 / Draw debug overlay.

        Args:
            display_image_frame: 显示图像 / Display image.
            tracking_result: 追踪结果 / Tracking result.
        """
        image_height, image_width = display_image_frame.shape[:2]
        image_center_x = image_width // 2
        image_center_y = image_height // 2

        cv2.drawMarker(
            display_image_frame,
            (image_center_x, image_center_y),
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=1,
        )

        if self.is_mouse_dragging and self.selected_region_start_point and self.selected_region_end_point:
            cv2.rectangle(
                display_image_frame,
                self.selected_region_start_point,
                self.selected_region_end_point,
                (255, 0, 0),
                2,
            )

        # target_display_result: 优先显示当前追踪结果，否则显示滤波目标 / Prefer current result, otherwise filtered target.
        target_display_result = tracking_result
        if not target_display_result.is_tracking_successful:
            target_display_result = self._build_tracking_result_from_filtered_state()

        if target_display_result.is_tracking_successful:
            if target_display_result.target_polygon is not None:
                cv2.polylines(display_image_frame, [target_display_result.target_polygon], True, (0, 255, 0), 2)
            else:
                rectangle_x, rectangle_y, rectangle_width, rectangle_height = target_display_result.target_rectangle
                cv2.rectangle(
                    display_image_frame,
                    (rectangle_x, rectangle_y),
                    (rectangle_x + rectangle_width, rectangle_y + rectangle_height),
                    (0, 255, 0),
                    2,
                )

            # target_center_point: 目标中心点 / Target center point.
            target_center_point = (
                int(target_display_result.target_center_x),
                int(target_display_result.target_center_y),
            )
            cv2.circle(display_image_frame, target_center_point, 4, (0, 0, 255), -1)
            cv2.line(display_image_frame, (image_center_x, image_center_y), target_center_point, (0, 255, 255), 1)

        # target_state_text: 目标状态文本 / Target state text.
        target_state_text = "valid" if self._get_target_state_snapshot().target_is_valid else "lost"
        status_text = (
            f"state={self.tracking_state} target={target_state_text} "
            f"s1={self.horizontal_angle_deg:.1f}deg s2={self.pitch_angle_deg:.1f}deg "
            f"backend={self.tracking_backend} hz={self.control_rate_hz:.0f}"
        )
        help_text = "Drag: select | SPACE: track/pause | i: identify | r: reselect | q/ESC: quit"
        tune_text = (
            f"PD kp={self.pd_kp:.2f} kd={self.pd_kd:.2f} "
            f"dead=({self.deadband_x:.0f},{self.deadband_y:.0f}) step={self.max_ptz_step:.1f} "
            f"smooth_step={self.ptz_step_smooth_alpha:.2f} ptz_a={self.ptz_smooth_alpha:.2f}"
        )
        fps_text = (
            f"fps recv={self.receive_based_frame_rate:.1f} "
            f"stamp={self.header_based_frame_rate:.1f}"
        )
        if self.show_image_stamp_delay:
            fps_text += f" delay={self.image_stamp_delay_ms:.0f}ms"
        if self.tracking_backend == "color_hsv":
            fps_text += (
                f" HSV=({self.hue_minimum},{self.saturation_minimum},{self.value_minimum})-"
                f"({self.hue_maximum},{self.saturation_maximum},{self.value_maximum})"
            )

        self._draw_overlay_text_lines(display_image_frame, [status_text, tune_text, fps_text, help_text])

    def _draw_overlay_text_lines(self, display_image_frame: np.ndarray, overlay_text_lines: list[str]) -> None:
        """
        绘制多行叠加文字 / Draw multi-line overlay text.

        Args:
            display_image_frame: 显示图像 / Display image.
            overlay_text_lines: 叠加文字列表 / Overlay text line list.
        """
        # image_height/image_width: 图像尺寸 / Image size.
        image_height, image_width = display_image_frame.shape[:2]

        # overlay_font: 叠加显示字体 / Overlay font.
        overlay_font = cv2.FONT_HERSHEY_SIMPLEX

        # overlay_font_scale: 叠加文字大小 / Overlay font scale.
        overlay_font_scale = max(0.25, float(self.overlay_font_scale))

        # overlay_text_thickness: 叠加文字线宽 / Overlay text thickness.
        overlay_text_thickness = 1

        # overlay_line_height: 叠加行高 / Overlay line height.
        overlay_line_height = max(12, int(self.overlay_line_height))

        # overlay_origin_x/overlay_origin_y: 叠加文字起点 / Overlay text origin.
        overlay_origin_x = max(0, int(self.overlay_margin_x))
        overlay_origin_y = max(overlay_line_height, int(self.overlay_margin_y))

        # maximum_text_width: 最大文字宽度 / Maximum text width.
        maximum_text_width = 0
        for overlay_text_line in overlay_text_lines:
            (text_width, _), _ = cv2.getTextSize(
                overlay_text_line,
                overlay_font,
                overlay_font_scale,
                overlay_text_thickness,
            )
            maximum_text_width = max(maximum_text_width, text_width)

        # overlay_panel_width/overlay_panel_height: 文字背景面板尺寸 / Overlay panel size.
        overlay_panel_width = min(image_width - overlay_origin_x - 4, maximum_text_width + 12)
        overlay_panel_height = len(overlay_text_lines) * overlay_line_height + 8

        if self.overlay_show_background and overlay_panel_width > 0 and overlay_panel_height > 0:
            # overlay_background_frame: 背景透明混合图 / Transparent background frame.
            overlay_background_frame = display_image_frame.copy()
            cv2.rectangle(
                overlay_background_frame,
                (overlay_origin_x - 4, max(0, overlay_origin_y - overlay_line_height + 2)),
                (
                    overlay_origin_x - 4 + overlay_panel_width,
                    min(image_height - 1, overlay_origin_y - overlay_line_height + 2 + overlay_panel_height),
                ),
                (0, 0, 0),
                -1,
            )
            cv2.addWeighted(overlay_background_frame, 0.35, display_image_frame, 0.65, 0, display_image_frame)

        # current_overlay_y: 当前文字 y 坐标 / Current overlay y coordinate.
        current_overlay_y = overlay_origin_y
        for overlay_text_line in overlay_text_lines:
            cv2.putText(
                display_image_frame,
                overlay_text_line,
                (overlay_origin_x, current_overlay_y),
                overlay_font,
                overlay_font_scale,
                (0, 255, 255),
                overlay_text_thickness,
                cv2.LINE_AA,
            )
            current_overlay_y += overlay_line_height

    def _build_tracking_result_from_filtered_state(self) -> TargetTrackingResult:
        """
        根据滤波目标状态构造显示用追踪结果 / Build display tracking result from filtered state.

        Returns:
            显示用追踪结果 / Tracking result for display.
        """
        with self.target_state_lock:
            if (
                not self.target_is_valid
                or self.filtered_target_center_x is None
                or self.filtered_target_center_y is None
            ):
                return TargetTrackingResult(False)

            return TargetTrackingResult(
                True,
                float(self.filtered_target_center_x),
                float(self.filtered_target_center_y),
                self.raw_target_rectangle,
                self.raw_target_polygon,
            )

    def _get_latest_image_copy(self) -> Optional[np.ndarray]:
        """
        获取最新图像副本 / Get copy of latest image.

        Returns:
            最新图像副本，若无图像则返回 None / Latest image copy or None.
        """
        # latest_image_copy: 最新图像副本 / Latest image copy.
        latest_image_copy, _ = self._get_latest_image_copy_with_timestamp()
        return latest_image_copy

    def _get_latest_image_copy_with_timestamp(self) -> Tuple[Optional[np.ndarray], float]:
        """
        获取最新图像副本和时间戳 / Get latest image copy and timestamp.

        Returns:
            最新图像副本和时间戳 / Latest image copy and timestamp.
        """
        with self.latest_image_lock:
            if self.latest_image_frame is None:
                return None, 0.0
            return self.latest_image_frame.copy(), float(self.latest_image_timestamp_sec)

    def _normalize_rectangle_to_image(
        self,
        target_rectangle: Tuple[int, int, int, int],
        image_width: int,
        image_height: int,
    ) -> Optional[Tuple[int, int, int, int]]:
        """
        将矩形限制到图像范围内 / Clip rectangle inside image.

        Args:
            target_rectangle: 输入矩形 / Input rectangle.
            image_width: 图像宽度 / Image width.
            image_height: 图像高度 / Image height.

        Returns:
            修正后的矩形，失败返回 None / Clipped rectangle or None.
        """
        rectangle_x, rectangle_y, rectangle_width, rectangle_height = target_rectangle

        rectangle_x = int(clamp_numeric_value(rectangle_x, 0, image_width - 1))
        rectangle_y = int(clamp_numeric_value(rectangle_y, 0, image_height - 1))
        rectangle_width = int(min(rectangle_width, image_width - rectangle_x))
        rectangle_height = int(min(rectangle_height, image_height - rectangle_y))

        if rectangle_width < 10 or rectangle_height < 10:
            return None

        return rectangle_x, rectangle_y, rectangle_width, rectangle_height

    @staticmethod
    def _is_point_inside_image(point_x: float, point_y: float, image_width: int, image_height: int) -> bool:
        """
        判断点是否在图像内 / Check whether point is inside image.

        Args:
            point_x: 点 x 坐标 / Point x coordinate.
            point_y: 点 y 坐标 / Point y coordinate.
            image_width: 图像宽度 / Image width.
            image_height: 图像高度 / Image height.

        Returns:
            是否在图像内 / Whether point is inside image.
        """
        return 0.0 <= point_x < float(image_width) and 0.0 <= point_y < float(image_height)

    def request_node_shutdown(self) -> None:
        """请求关闭节点 / Request node shutdown."""
        self.is_shutdown_requested = True
        try:
            self.get_logger().info("Force exiting object_tracker_ptz_node: q/ESC key pressed.")
        except Exception:
            pass
        # 使用 os._exit(0) 直接退出，避免 rclpy.shutdown() 或 cv2.destroyAllWindows() 卡死。
        # Use os._exit(0) to avoid rclpy.shutdown() or cv2.destroyAllWindows() freeze.
        os._exit(0)  # pylint: disable=protected-access

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            父类销毁结果 / Parent destroy result.
        """
        # 注意：这里故意不调用 cv2.destroyAllWindows()。
        # 部分板子/远程桌面/OpenCV HighGUI 环境下 destroyAllWindows 会卡死。
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """
    节点入口函数 / Node entry function.

    Args:
        args: ROS2 启动参数 / ROS2 launch arguments.
    """
    rclpy.init(args=args)

    # object_tracker_ptz_node: 物体追踪云台节点 / Object tracker PTZ node.
    object_tracker_ptz_node = ObjectTrackerPtzNode()

    try:
        while rclpy.ok() and not object_tracker_ptz_node.is_shutdown_requested:
            rclpy.spin_once(object_tracker_ptz_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        # q/ESC 会直接 os._exit(0)，不会进入这里。
        # Ctrl+C 会进入这里，做 ROS 节点销毁；仍然不主动销毁 OpenCV 窗口。
        try:
            object_tracker_ptz_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
