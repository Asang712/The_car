#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MicroROS-V2 color tracker PTZ node, optimized for low FPS compressed camera input.
MicroROS-V2 颜色追踪云台控制节点，针对低帧率压缩摄像头输入进行优化。

Main functions / 主要功能：
1. Subscribe to /camera/image_raw by default and decode sensor_msgs/msg/Image.
   默认订阅 /camera/image_raw，并解码 sensor_msgs/msg/Image。
2. Use HSV threshold segmentation to detect the largest target color region.
   使用 HSV 阈值分割识别最大的目标颜色区域。
3. Smooth target center and PTZ angle to reduce jitter on low FPS cameras.
   对目标中心点和云台角度进行低通滤波，降低低帧率摄像头导致的抖动。
4. Separate image detection from PTZ control using a fixed-rate control timer.
   通过固定频率控制定时器，将图像检测和云台控制解耦。
5. Publish geometry_msgs/msg/Vector3 to /cmd_ptz_angle for PTZ angle control.
   发布 geometry_msgs/msg/Vector3 到 /cmd_ptz_angle 控制云台角度。

Keyboard shortcuts / 键盘快捷键：
  r/R: select ROI and resample target color / 重新框选 ROI 并采样目标颜色
  i/I: identify mode, detect only / 识别模式，只识别不控制云台
  SPACE: tracking mode, detect and control PTZ / 追踪模式，识别并控制云台
  q/Q: quit node / 退出节点
"""

import os
import threading
import time
from typing import Any, Dict, Optional, Tuple

import cv2 as cv
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr


# HSV threshold range type / HSV 阈值范围类型
HueSaturationValueRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]

# Target circle type: center_x, center_y, radius in pixels / 目标圆类型：像素中心 x、像素中心 y、像素半径
TargetCircleInPixels = Tuple[int, int, int]

# ROI type: left, top, right, bottom in pixels / ROI 类型：像素左、上、右、下边界
RegionOfInterestInPixels = Tuple[int, int, int, int]

# Minimum valid timer period frequency / 定时器最小有效频率
MINIMUM_TIMER_FREQUENCY_HZ = 1.0

# Default target lost timeout lower bound / 目标丢失超时下限
MINIMUM_TARGET_LOST_TIMEOUT_SECONDS = 0.05

# Runtime dynamic parameter names / 支持运行时动态修改的参数名称
RUNTIME_DYNAMIC_PARAMETER_NAMES = frozenset({
    "image_width",
    "image_height",
    "display_debug_window",
    "publish_debug_image",
    "show_binary_window",
    "window_width",
    "window_height",
    "auto_start_tracking",
    "overlay_font_scale",
    "overlay_line_height",
    "overlay_margin_x",
    "overlay_margin_y",
    "overlay_show_background",
    "param_reload_interval",
    "enable_rqt_dynamic_parameters",
    "hsv_file",
    "Hmin",
    "Smin",
    "Vmin",
    "Hmax",
    "Smax",
    "Vmax",
    "h_expand",
    "s_expand",
    "v_expand",
    "min_target_radius",
    "min_target_area",
    "morph_kernel_size",
    "ptz_init_x",
    "ptz_init_y",
    "ptz_x_min",
    "ptz_x_max",
    "ptz_y_min",
    "ptz_y_max",
    "ptz_x_reverse",
    "ptz_y_reverse",
    "init_publish_count",
    "deadband_x",
    "deadband_y",
    "pd_kp",
    "pd_kd",
    "ptz_step_smooth_alpha",
    "max_ptz_step",
    "target_smooth_alpha",
    "ptz_smooth_alpha",
    "ptz_control_rate",
    "lost_target_stop",
    "lost_timeout",
})

# Restart-required parameter names / 需要重启节点后才能修改的参数名称
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "ptz_topic",
    "debug_image_topic",
    "window_name",
})

# HSV threshold parameter names / HSV 阈值参数名称
HSV_THRESHOLD_PARAMETER_NAMES = frozenset({
    "Hmin",
    "Smin",
    "Vmin",
    "Hmax",
    "Smax",
    "Vmax",
})


def clamp_numeric_value(input_value: float, minimum_value: float, maximum_value: float) -> float:
    """Clamp a numeric value into a closed interval.

    将数值限制在闭区间范围内。

    Args:
        input_value: Value to clamp / 需要限制的输入值。
        minimum_value: Lower bound / 最小边界值。
        maximum_value: Upper bound / 最大边界值。

    Returns:
        Clamped value / 限幅后的数值。
    """
    # Clamped numeric result / 限幅后的数值结果
    clamped_numeric_value = max(minimum_value, min(maximum_value, input_value))
    return clamped_numeric_value


def convert_value_to_boolean(input_value, default_boolean_value: bool) -> bool:
    """Convert a ROS parameter value to bool safely.

    安全地将 ROS 参数值转换为布尔值。

    Args:
        input_value: Raw parameter value / 原始参数值。
        default_boolean_value: Fallback value / 转换失败时使用的默认值。

    Returns:
        Converted boolean value / 转换后的布尔值。
    """
    # Direct bool value / 直接布尔值
    if isinstance(input_value, bool):
        return input_value

    # String value from launch file / 来自 launch 文件的字符串值
    if isinstance(input_value, str):
        normalized_text_value = input_value.strip().lower()
        return normalized_text_value in ("true", "1", "yes", "y", "on")

    # Numeric value from YAML or command line / 来自 YAML 或命令行的数值
    if isinstance(input_value, (int, float)):
        return bool(input_value)

    return default_boolean_value


def read_hsv_threshold_file(hsv_file_path: str) -> Optional[HueSaturationValueRange]:
    """Read HSV threshold range from file.

    从文件读取 HSV 阈值范围。

    File format / 文件格式：Hmin, Smin, Vmin, Hmax, Smax, Vmax

    Args:
        hsv_file_path: HSV threshold file path / HSV 阈值文件路径。

    Returns:
        HSV range if valid, otherwise None / 有效时返回 HSV 范围，否则返回 None。
    """
    # Empty HSV file path / 空 HSV 文件路径
    if not hsv_file_path:
        return None

    # Expanded HSV file path / 展开后的 HSV 文件路径
    expanded_hsv_file_path = os.path.expanduser(hsv_file_path)
    if not os.path.exists(expanded_hsv_file_path):
        return None

    try:
        # First line text from HSV file / HSV 文件第一行文本
        with open(expanded_hsv_file_path, "r", encoding="utf-8") as hsv_file_reader:
            hsv_line_text = hsv_file_reader.readline().strip()

        if not hsv_line_text:
            return None

        # Parsed HSV integer list / 解析后的 HSV 整数列表
        hsv_integer_values = [int(text_value.strip()) for text_value in hsv_line_text.split(",")]
        if len(hsv_integer_values) != 6:
            return None

        # HSV lower threshold / HSV 下限阈值
        hsv_lower_threshold = (hsv_integer_values[0], hsv_integer_values[1], hsv_integer_values[2])

        # HSV upper threshold / HSV 上限阈值
        hsv_upper_threshold = (hsv_integer_values[3], hsv_integer_values[4], hsv_integer_values[5])

        return hsv_lower_threshold, hsv_upper_threshold
    except Exception:
        return None


def write_hsv_threshold_file(
    hsv_file_path: str,
    hsv_threshold_range: HueSaturationValueRange,
) -> None:
    """Write HSV threshold range to file.

    将 HSV 阈值范围写入文件。

    Args:
        hsv_file_path: HSV threshold file path / HSV 阈值文件路径。
        hsv_threshold_range: HSV threshold range / HSV 阈值范围。
    """
    # Empty HSV file path / 空 HSV 文件路径
    if not hsv_file_path:
        return

    # Expanded HSV file path / 展开后的 HSV 文件路径
    expanded_hsv_file_path = os.path.expanduser(hsv_file_path)

    # Parent directory path / 父级目录路径
    hsv_file_parent_directory = os.path.dirname(expanded_hsv_file_path)
    if hsv_file_parent_directory:
        os.makedirs(hsv_file_parent_directory, exist_ok=True)

    # HSV file content text / HSV 文件内容文本
    hsv_file_content_text = (
        f"{hsv_threshold_range[0][0]}, {hsv_threshold_range[0][1]}, {hsv_threshold_range[0][2]}, "
        f"{hsv_threshold_range[1][0]}, {hsv_threshold_range[1][1]}, {hsv_threshold_range[1][2]}"
    )

    with open(expanded_hsv_file_path, "w", encoding="utf-8") as hsv_file_writer:
        hsv_file_writer.write(hsv_file_content_text)
        hsv_file_writer.flush()


class ColorTrackerPtzNode(Node):
    """ROS2 color tracker PTZ node.

    ROS2 颜色追踪云台控制节点。
    """

    def __init__(self) -> None:
        """Initialize ROS2 node, parameters, publishers, subscribers and timers.

        初始化 ROS2 节点、参数、发布器、订阅器和定时器。
        """
        super().__init__("color_tracker_ptz_node")

        # CV bridge object / OpenCV 与 ROS 图像互转桥对象
        self.cv_bridge = CvBridge()

        # Target state lock / 目标状态线程锁
        self.target_state_lock = threading.Lock()

        self._declare_ros_parameters()
        self._load_ros_parameters()

        # PTZ angle publisher / 云台角度发布器
        self.ptz_angle_publisher = self.create_publisher(Vector3, self.ptz_topic, 10)

        # Debug image publisher / 调试图像发布器
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)

        # Image subscriber / 图像订阅器
        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        # Current tracking state / 当前追踪状态
        self.tracking_state = "identify"

        # Current HSV threshold range / 当前 HSV 阈值范围
        self.hsv_threshold_range: HueSaturationValueRange = self._get_hsv_range_from_parameters()

        # Saved HSV threshold range / 已保存 HSV 阈值范围
        saved_hsv_threshold_range = read_hsv_threshold_file(self.hsv_file)
        if saved_hsv_threshold_range is not None:
            self.hsv_threshold_range = saved_hsv_threshold_range
            self._apply_hsv_range_to_ros_parameters(saved_hsv_threshold_range)
            self.get_logger().info(
                f"Loaded HSV from file: {self.hsv_file}, hsv={saved_hsv_threshold_range}"
            )
        elif self.display_debug_window:
            self.tracking_state = "init"
            self.get_logger().warn(
                "No valid HSV file found. Press r and drag ROI to sample target color."
            )
        else:
            self.get_logger().warn("No valid HSV file found. Using HSV parameters from YAML/launch.")

        if self.auto_start_tracking and self.hsv_threshold_range is not None:
            self.tracking_state = "tracking"

        # Current PTZ horizontal angle / 当前云台水平角度
        self.current_ptz_horizontal_angle = float(self.ptz_init_x)

        # Current PTZ vertical angle / 当前云台俯仰角度
        self.current_ptz_vertical_angle = float(self.ptz_init_y)

        # PD control state / PD 控制状态
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

        # Raw target circle from image detection / 图像检测得到的原始目标圆
        self.raw_target_circle: TargetCircleInPixels = (0, 0, 0)

        # Raw target contour area / 原始目标轮廓面积
        self.raw_target_area = 0.0

        # Filtered target center x / 滤波后的目标中心 x
        self.filtered_target_center_x: Optional[float] = None

        # Filtered target center y / 滤波后的目标中心 y
        self.filtered_target_center_y: Optional[float] = None

        # Filtered target radius / 滤波后的目标半径
        self.filtered_target_radius: Optional[float] = None

        # Target validity flag / 目标有效标志
        self.target_is_valid = False

        # Last valid target monotonic time / 上一次有效目标的单调时间
        self.last_valid_target_time_seconds = time.perf_counter()

        # Last frame monotonic time / 上一帧图像的单调时间
        self.last_frame_time_seconds = time.perf_counter()

        # Smoothed input frame rate / 平滑后的输入帧率
        self.smoothed_frame_rate = 0.0

        # Mouse pressed flag / 鼠标按下标志
        self.mouse_button_is_pressed = False

        # ROI start pixel / ROI 起始像素坐标
        self.roi_start_pixel = (0, 0)

        # ROI end pixel / ROI 结束像素坐标
        self.roi_end_pixel = (0, 0)

        # ROI ready flag / ROI 已完成标志
        self.roi_is_ready = False

        # Remaining initial PTZ publish count / 剩余初始云台角度发布次数
        self.remaining_initial_ptz_publish_count = max(1, int(self.init_publish_count))

        # Initial PTZ timer / 初始云台角度发布定时器
        self.initial_ptz_timer = self.create_timer(0.1, self._publish_initial_ptz_angle)

        # Fixed-rate PTZ control timer handle / 固定频率云台控制定时器句柄
        self.ptz_control_timer = None

        # Create fixed-rate PTZ control timer / 创建固定频率云台控制定时器
        self._recreate_ptz_control_timer(False)

        if self.display_debug_window:
            cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
            cv.resizeWindow(self.window_name, self.window_width, self.window_height)
            cv.setMouseCallback(self.window_name, self._on_mouse_event)

        # Runtime parameter callback handle for rqt / 用于 rqt 动态调参的运行时参数回调句柄
        self.parameter_callback_handle = self.add_on_set_parameters_callback(
            self._handle_runtime_parameter_update
        )

        self.get_logger().info(
            "Color tracker PTZ node started. "
            f"image_topic={self.image_topic}, "
            f"ptz_topic={self.ptz_topic}, "
            f"ptz_control_rate={self.ptz_control_rate}Hz"
        )

    def _declare_ros_parameters(self) -> None:
        """Declare all ROS2 parameters used by this node.

        声明本节点使用的全部 ROS2 参数。
        """
        # Topic parameters / 话题参数
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ptz_topic", "/cmd_ptz_angle")
        self.declare_parameter("debug_image_topic", "/color_tracker_ptz/debug_image")

        # Image and GUI parameters / 图像和界面参数
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)
        self.declare_parameter("display_debug_window", True)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("show_binary_window", False)
        self.declare_parameter("window_name", "color_tracker_ptz")
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 360)
        self.declare_parameter("auto_start_tracking", False)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 16)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", True)
        self.declare_parameter("param_reload_interval", 1.0)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

        # HSV threshold parameters / HSV 阈值参数
        self.declare_parameter("hsv_file", "")
        self.declare_parameter("Hmin", 0)
        self.declare_parameter("Smin", 85)
        self.declare_parameter("Vmin", 126)
        self.declare_parameter("Hmax", 9)
        self.declare_parameter("Smax", 253)
        self.declare_parameter("Vmax", 253)
        self.declare_parameter("h_expand", 5)
        self.declare_parameter("s_expand", 20)
        self.declare_parameter("v_expand", 20)
        self.declare_parameter("min_target_radius", 5.0)
        self.declare_parameter("min_target_area", 60.0)
        self.declare_parameter("morph_kernel_size", 5)

        # PTZ angle parameters / 云台角度参数
        self.declare_parameter("ptz_init_x", 0.0)
        self.declare_parameter("ptz_init_y", 10.0)
        self.declare_parameter("ptz_x_min", -45.0)
        self.declare_parameter("ptz_x_max", 45.0)
        self.declare_parameter("ptz_y_min", -90.0)
        self.declare_parameter("ptz_y_max", 40.0)
        self.declare_parameter("ptz_x_reverse", False)
        self.declare_parameter("ptz_y_reverse", False)
        self.declare_parameter("init_publish_count", 10)

        # PD and smoothing parameters / PD 与平滑控制参数
        self.declare_parameter("deadband_x", 20.0)
        self.declare_parameter("deadband_y", 20.0)
        self.declare_parameter("pd_kp", 0.7)
        self.declare_parameter("pd_kd", 0.25)
        self.declare_parameter("ptz_step_smooth_alpha", 0.35)
        self.declare_parameter("max_ptz_step", 1.2)
        self.declare_parameter("target_smooth_alpha", 0.35)
        self.declare_parameter("ptz_smooth_alpha", 0.25)
        self.declare_parameter("ptz_control_rate", 20.0)

        # Target lost handling parameters / 目标丢失处理参数
        self.declare_parameter("lost_target_stop", True)
        self.declare_parameter("lost_timeout", 1.0)

    def _load_ros_parameters(self) -> None:
        """Load ROS2 parameters into semantic member variables.

        将 ROS2 参数读取到语义化成员变量中。
        """
        # Topic names / 话题名称
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.ptz_topic = str(self.get_parameter("ptz_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)

        # Image and GUI configuration / 图像与界面配置
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.display_debug_window = convert_value_to_boolean(
            self.get_parameter("display_debug_window").value,
            True,
        )
        self.publish_debug_image = convert_value_to_boolean(
            self.get_parameter("publish_debug_image").value,
            False,
        )
        self.show_binary_window = convert_value_to_boolean(
            self.get_parameter("show_binary_window").value,
            False,
        )
        self.window_name = str(self.get_parameter("window_name").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.auto_start_tracking = convert_value_to_boolean(
            self.get_parameter("auto_start_tracking").value,
            False,
        )
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = convert_value_to_boolean(
            self.get_parameter("overlay_show_background").value,
            True,
        )
        self.param_reload_interval = float(self.get_parameter("param_reload_interval").value)
        self.enable_rqt_dynamic_parameters = convert_value_to_boolean(
            self.get_parameter("enable_rqt_dynamic_parameters").value,
            True,
        )

        # HSV threshold configuration / HSV 阈值配置
        self.hsv_file = os.path.expanduser(str(self.get_parameter("hsv_file").value))
        self.hue_minimum = int(self.get_parameter("Hmin").value)
        self.saturation_minimum = int(self.get_parameter("Smin").value)
        self.value_minimum = int(self.get_parameter("Vmin").value)
        self.hue_maximum = int(self.get_parameter("Hmax").value)
        self.saturation_maximum = int(self.get_parameter("Smax").value)
        self.value_maximum = int(self.get_parameter("Vmax").value)
        self.hue_expand = int(self.get_parameter("h_expand").value)
        self.saturation_expand = int(self.get_parameter("s_expand").value)
        self.value_expand = int(self.get_parameter("v_expand").value)
        self.min_target_radius = float(self.get_parameter("min_target_radius").value)
        self.min_target_area = float(self.get_parameter("min_target_area").value)
        self.morph_kernel_size = int(self.get_parameter("morph_kernel_size").value)

        # PTZ angle configuration / 云台角度配置
        self.ptz_init_x = float(self.get_parameter("ptz_init_x").value)
        self.ptz_init_y = float(self.get_parameter("ptz_init_y").value)
        self.ptz_x_min = float(self.get_parameter("ptz_x_min").value)
        self.ptz_x_max = float(self.get_parameter("ptz_x_max").value)
        self.ptz_y_min = float(self.get_parameter("ptz_y_min").value)
        self.ptz_y_max = float(self.get_parameter("ptz_y_max").value)
        self.ptz_x_reverse = convert_value_to_boolean(
            self.get_parameter("ptz_x_reverse").value,
            False,
        )
        self.ptz_y_reverse = convert_value_to_boolean(
            self.get_parameter("ptz_y_reverse").value,
            False,
        )
        self.init_publish_count = int(self.get_parameter("init_publish_count").value)

        # PD and smoothing configuration / PD 与平滑控制配置
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
            0.01,
            1.0,
        )
        self.ptz_smooth_alpha = clamp_numeric_value(
            float(self.get_parameter("ptz_smooth_alpha").value),
            0.01,
            1.0,
        )
        self.ptz_control_rate = float(self.get_parameter("ptz_control_rate").value)

        # Target lost handling configuration / 目标丢失处理配置
        self.lost_target_stop = convert_value_to_boolean(
            self.get_parameter("lost_target_stop").value,
            True,
        )
        self.lost_timeout = max(
            float(self.get_parameter("lost_timeout").value),
            MINIMUM_TARGET_LOST_TIMEOUT_SECONDS,
        )

    def _handle_runtime_parameter_update(self, parameter_list) -> SetParametersResult:
        """Handle runtime parameter changes from rqt or ros2 param set.

        处理来自 rqt 或 ros2 param set 的运行时参数变化。

        Args:
            parameter_list: Parameter update list / 参数更新列表。

        Returns:
            Parameter update result / 参数更新结果。
        """
        # Empty parameter update / 空参数更新
        if not parameter_list:
            return SetParametersResult(successful=True, reason="No parameter changed.")

        # Changed parameter names / 发生变化的参数名称
        changed_parameter_names = {runtime_parameter.name for runtime_parameter in parameter_list}

        # Candidate parameter value map / 候选参数值映射
        candidate_parameter_value_map = self._build_candidate_parameter_value_map(parameter_list)

        # Validation result text / 参数校验结果文本
        validation_error_text = self._validate_runtime_parameter_update(
            parameter_list,
            candidate_parameter_value_map,
        )
        if validation_error_text:
            return SetParametersResult(successful=False, reason=validation_error_text)

        try:
            self._apply_runtime_parameter_update(
                candidate_parameter_value_map,
                changed_parameter_names,
            )
        except Exception as exception:
            return SetParametersResult(
                successful=False,
                reason=f"Apply runtime parameter failed: {exception}",
            )

        return SetParametersResult(successful=True, reason="Runtime parameters updated.")

    def _build_candidate_parameter_value_map(self, parameter_list) -> Dict[str, Any]:
        """Build merged parameter value map before applying runtime updates.

        构建运行时参数更新前的新旧合并参数值映射。

        Args:
            parameter_list: Parameter update list / 参数更新列表。

        Returns:
            Candidate parameter value map / 候选参数值映射。
        """
        # Candidate parameter value map / 候选参数值映射
        candidate_parameter_value_map: Dict[str, Any] = {}

        # All known parameter names / 全部已知参数名称
        known_parameter_names = RUNTIME_DYNAMIC_PARAMETER_NAMES | RESTART_REQUIRED_PARAMETER_NAMES
        for parameter_name in known_parameter_names:
            try:
                candidate_parameter_value_map[parameter_name] = self.get_parameter(parameter_name).value
            except Exception:
                pass

        for runtime_parameter in parameter_list:
            candidate_parameter_value_map[runtime_parameter.name] = runtime_parameter.value

        return candidate_parameter_value_map

    def _validate_runtime_parameter_update(
        self,
        parameter_list,
        candidate_parameter_value_map: Dict[str, Any],
    ) -> str:
        """Validate runtime parameter update before accepting it.

        在接受运行时参数更新前进行参数合法性检查。

        Args:
            parameter_list: Parameter update list / 参数更新列表。
            candidate_parameter_value_map: Candidate parameter value map / 候选参数值映射。

        Returns:
            Empty string if valid, otherwise error reason / 合法时返回空字符串，否则返回错误原因。
        """
        # Changed parameter names / 发生变化的参数名称
        changed_parameter_names = {runtime_parameter.name for runtime_parameter in parameter_list}

        for runtime_parameter in parameter_list:
            if runtime_parameter.type_ == Parameter.Type.NOT_SET:
                return f"Parameter {runtime_parameter.name} cannot be undeclared at runtime."

        # Unsupported parameter names / 不支持动态修改的参数名称
        unsupported_parameter_names = changed_parameter_names - RUNTIME_DYNAMIC_PARAMETER_NAMES - RESTART_REQUIRED_PARAMETER_NAMES
        if unsupported_parameter_names:
            return f"Unsupported runtime parameter(s): {sorted(unsupported_parameter_names)}"

        # Restart-required parameter names / 需要重启节点的参数名称
        restart_required_parameter_names = changed_parameter_names & RESTART_REQUIRED_PARAMETER_NAMES
        if restart_required_parameter_names:
            return (
                "These parameter(s) are fixed after startup and require node restart: "
                f"{sorted(restart_required_parameter_names)}"
            )

        # Requested dynamic parameter switch / 请求后的动态调参开关
        requested_dynamic_parameter_enabled = convert_value_to_boolean(
            candidate_parameter_value_map.get(
                "enable_rqt_dynamic_parameters",
                self.enable_rqt_dynamic_parameters,
            ),
            self.enable_rqt_dynamic_parameters,
        )
        if not requested_dynamic_parameter_enabled and changed_parameter_names != {"enable_rqt_dynamic_parameters"}:
            return "Runtime dynamic parameters are disabled by enable_rqt_dynamic_parameters."

        try:
            self._validate_candidate_numeric_ranges(candidate_parameter_value_map)
        except ValueError as exception:
            return str(exception)

        return ""

    def _validate_candidate_numeric_ranges(self, candidate_parameter_value_map: Dict[str, Any]) -> None:
        """Validate numeric ranges for runtime parameters.

        校验运行时参数的数值范围。

        Args:
            candidate_parameter_value_map: Candidate parameter value map / 候选参数值映射。

        Raises:
            ValueError: Parameter value is invalid / 参数值非法。
        """
        # HSV threshold values / HSV 阈值数值
        hue_minimum = int(candidate_parameter_value_map["Hmin"])
        hue_maximum = int(candidate_parameter_value_map["Hmax"])
        saturation_minimum = int(candidate_parameter_value_map["Smin"])
        saturation_maximum = int(candidate_parameter_value_map["Smax"])
        value_minimum = int(candidate_parameter_value_map["Vmin"])
        value_maximum = int(candidate_parameter_value_map["Vmax"])

        if not (0 <= hue_minimum <= 179 and 0 <= hue_maximum <= 179):
            raise ValueError("Hmin/Hmax must be in [0, 179].")
        if not (0 <= saturation_minimum <= 255 and 0 <= saturation_maximum <= 255):
            raise ValueError("Smin/Smax must be in [0, 255].")
        if not (0 <= value_minimum <= 255 and 0 <= value_maximum <= 255):
            raise ValueError("Vmin/Vmax must be in [0, 255].")
        if hue_minimum > hue_maximum:
            raise ValueError("Hmin must be less than or equal to Hmax.")
        if saturation_minimum > saturation_maximum:
            raise ValueError("Smin must be less than or equal to Smax.")
        if value_minimum > value_maximum:
            raise ValueError("Vmin must be less than or equal to Vmax.")

        # Expansion values / HSV 扩展数值
        if int(candidate_parameter_value_map["h_expand"]) < 0:
            raise ValueError("h_expand must be greater than or equal to 0.")
        if int(candidate_parameter_value_map["s_expand"]) < 0:
            raise ValueError("s_expand must be greater than or equal to 0.")
        if int(candidate_parameter_value_map["v_expand"]) < 0:
            raise ValueError("v_expand must be greater than or equal to 0.")

        # Image and display size values / 图像与显示尺寸数值
        if int(candidate_parameter_value_map["image_width"]) <= 0:
            raise ValueError("image_width must be greater than 0.")
        if int(candidate_parameter_value_map["image_height"]) <= 0:
            raise ValueError("image_height must be greater than 0.")
        if int(candidate_parameter_value_map["window_width"]) <= 0:
            raise ValueError("window_width must be greater than 0.")
        if int(candidate_parameter_value_map["window_height"]) <= 0:
            raise ValueError("window_height must be greater than 0.")
        if float(candidate_parameter_value_map["overlay_font_scale"]) <= 0.0:
            raise ValueError("overlay_font_scale must be greater than 0.")
        if int(candidate_parameter_value_map["overlay_line_height"]) <= 0:
            raise ValueError("overlay_line_height must be greater than 0.")
        if int(candidate_parameter_value_map["overlay_margin_x"]) < 0:
            raise ValueError("overlay_margin_x must be greater than or equal to 0.")
        if int(candidate_parameter_value_map["overlay_margin_y"]) < 0:
            raise ValueError("overlay_margin_y must be greater than or equal to 0.")
        if float(candidate_parameter_value_map["param_reload_interval"]) < 0.2:
            raise ValueError("param_reload_interval must be greater than or equal to 0.2.")

        # Target filter values / 目标检测过滤数值
        if float(candidate_parameter_value_map["min_target_radius"]) < 0.0:
            raise ValueError("min_target_radius must be greater than or equal to 0.")
        if float(candidate_parameter_value_map["min_target_area"]) < 0.0:
            raise ValueError("min_target_area must be greater than or equal to 0.")
        if int(candidate_parameter_value_map["morph_kernel_size"]) < 1:
            raise ValueError("morph_kernel_size must be greater than or equal to 1.")

        # PTZ range values / 云台角度范围数值
        ptz_x_minimum = float(candidate_parameter_value_map["ptz_x_min"])
        ptz_x_maximum = float(candidate_parameter_value_map["ptz_x_max"])
        ptz_y_minimum = float(candidate_parameter_value_map["ptz_y_min"])
        ptz_y_maximum = float(candidate_parameter_value_map["ptz_y_max"])
        if ptz_x_minimum >= ptz_x_maximum:
            raise ValueError("ptz_x_min must be less than ptz_x_max.")
        if ptz_y_minimum >= ptz_y_maximum:
            raise ValueError("ptz_y_min must be less than ptz_y_max.")
        if int(candidate_parameter_value_map["init_publish_count"]) < 0:
            raise ValueError("init_publish_count must be greater than or equal to 0.")

        # PD and smoothing values / PD 与平滑参数数值
        if float(candidate_parameter_value_map["deadband_x"]) < 0.0:
            raise ValueError("deadband_x must be greater than or equal to 0.")
        if float(candidate_parameter_value_map["deadband_y"]) < 0.0:
            raise ValueError("deadband_y must be greater than or equal to 0.")
        if float(candidate_parameter_value_map["pd_kp"]) < 0.0:
            raise ValueError("pd_kp must be greater than or equal to 0.")
        if float(candidate_parameter_value_map["pd_kd"]) < 0.0:
            raise ValueError("pd_kd must be greater than or equal to 0.")
        if not (0.0 <= float(candidate_parameter_value_map["ptz_step_smooth_alpha"]) <= 1.0):
            raise ValueError("ptz_step_smooth_alpha must be in [0.0, 1.0].")
        if float(candidate_parameter_value_map["max_ptz_step"]) <= 0.0:
            raise ValueError("max_ptz_step must be greater than 0.")
        if not (0.01 <= float(candidate_parameter_value_map["target_smooth_alpha"]) <= 1.0):
            raise ValueError("target_smooth_alpha must be in [0.01, 1.0].")
        if not (0.01 <= float(candidate_parameter_value_map["ptz_smooth_alpha"]) <= 1.0):
            raise ValueError("ptz_smooth_alpha must be in [0.01, 1.0].")
        if float(candidate_parameter_value_map["ptz_control_rate"]) < MINIMUM_TIMER_FREQUENCY_HZ:
            raise ValueError("ptz_control_rate must be greater than or equal to 1.0 Hz.")

        # Target lost values / 目标丢失数值
        if float(candidate_parameter_value_map["lost_timeout"]) < MINIMUM_TARGET_LOST_TIMEOUT_SECONDS:
            raise ValueError("lost_timeout is too small.")

    def _apply_runtime_parameter_update(
        self,
        candidate_parameter_value_map: Dict[str, Any],
        changed_parameter_names,
    ) -> None:
        """Apply validated runtime parameter update.

        应用已经通过校验的运行时参数更新。

        Args:
            candidate_parameter_value_map: Candidate parameter value map / 候选参数值映射。
            changed_parameter_names: Changed parameter name set / 发生变化的参数名称集合。
        """
        # Previous PTZ control rate / 之前的云台控制频率
        previous_ptz_control_rate = self.ptz_control_rate

        # Previous debug window state / 之前的调试窗口状态
        previous_display_debug_window = self.display_debug_window

        # Previous debug window size / 之前的调试窗口尺寸
        previous_debug_window_size = (self.window_width, self.window_height)

        self._load_runtime_values_from_candidate_map(candidate_parameter_value_map)

        # Refresh HSV range when HSV threshold changes / HSV 阈值变化时刷新 HSV 范围
        if changed_parameter_names & HSV_THRESHOLD_PARAMETER_NAMES:
            self.hsv_threshold_range = self._get_hsv_range_from_parameters()

        # Recreate PTZ timer when control rate changes / 控制频率变化时重建云台控制定时器
        if abs(float(self.ptz_control_rate) - float(previous_ptz_control_rate)) > 1.0e-9:
            self._recreate_ptz_control_timer(True)

        # Clamp current PTZ angle after range update / 范围更新后限制当前云台角度
        self.current_ptz_horizontal_angle = clamp_numeric_value(
            self.current_ptz_horizontal_angle,
            self.ptz_x_min,
            self.ptz_x_max,
        )
        self.current_ptz_vertical_angle = clamp_numeric_value(
            self.current_ptz_vertical_angle,
            self.ptz_y_min,
            self.ptz_y_max,
        )

        # Synchronize OpenCV debug window / 同步 OpenCV 调试窗口
        self._synchronize_debug_window_runtime_state(
            previous_display_debug_window,
            previous_debug_window_size,
        )

    def _load_runtime_values_from_candidate_map(self, candidate_parameter_value_map: Dict[str, Any]) -> None:
        """Load runtime variables from candidate parameter values.

        从候选参数值中加载运行时变量。

        Args:
            candidate_parameter_value_map: Candidate parameter value map / 候选参数值映射。
        """
        # Image and GUI configuration / 图像与界面配置
        self.image_width = int(candidate_parameter_value_map["image_width"])
        self.image_height = int(candidate_parameter_value_map["image_height"])
        self.display_debug_window = convert_value_to_boolean(
            candidate_parameter_value_map["display_debug_window"],
            self.display_debug_window,
        )
        self.publish_debug_image = convert_value_to_boolean(
            candidate_parameter_value_map["publish_debug_image"],
            self.publish_debug_image,
        )
        self.show_binary_window = convert_value_to_boolean(
            candidate_parameter_value_map["show_binary_window"],
            self.show_binary_window,
        )
        self.window_width = int(candidate_parameter_value_map["window_width"])
        self.window_height = int(candidate_parameter_value_map["window_height"])
        self.auto_start_tracking = convert_value_to_boolean(
            candidate_parameter_value_map["auto_start_tracking"],
            self.auto_start_tracking,
        )
        self.overlay_font_scale = float(candidate_parameter_value_map["overlay_font_scale"])
        self.overlay_line_height = int(candidate_parameter_value_map["overlay_line_height"])
        self.overlay_margin_x = int(candidate_parameter_value_map["overlay_margin_x"])
        self.overlay_margin_y = int(candidate_parameter_value_map["overlay_margin_y"])
        self.overlay_show_background = convert_value_to_boolean(
            candidate_parameter_value_map["overlay_show_background"],
            self.overlay_show_background,
        )
        self.param_reload_interval = float(candidate_parameter_value_map["param_reload_interval"])
        self.enable_rqt_dynamic_parameters = convert_value_to_boolean(
            candidate_parameter_value_map["enable_rqt_dynamic_parameters"],
            self.enable_rqt_dynamic_parameters,
        )

        # HSV threshold configuration / HSV 阈值配置
        self.hsv_file = os.path.expanduser(str(candidate_parameter_value_map["hsv_file"]))
        self.hue_minimum = int(candidate_parameter_value_map["Hmin"])
        self.saturation_minimum = int(candidate_parameter_value_map["Smin"])
        self.value_minimum = int(candidate_parameter_value_map["Vmin"])
        self.hue_maximum = int(candidate_parameter_value_map["Hmax"])
        self.saturation_maximum = int(candidate_parameter_value_map["Smax"])
        self.value_maximum = int(candidate_parameter_value_map["Vmax"])
        self.hue_expand = int(candidate_parameter_value_map["h_expand"])
        self.saturation_expand = int(candidate_parameter_value_map["s_expand"])
        self.value_expand = int(candidate_parameter_value_map["v_expand"])
        self.min_target_radius = float(candidate_parameter_value_map["min_target_radius"])
        self.min_target_area = float(candidate_parameter_value_map["min_target_area"])
        self.morph_kernel_size = int(candidate_parameter_value_map["morph_kernel_size"])

        # PTZ angle configuration / 云台角度配置
        self.ptz_init_x = float(candidate_parameter_value_map["ptz_init_x"])
        self.ptz_init_y = float(candidate_parameter_value_map["ptz_init_y"])
        self.ptz_x_min = float(candidate_parameter_value_map["ptz_x_min"])
        self.ptz_x_max = float(candidate_parameter_value_map["ptz_x_max"])
        self.ptz_y_min = float(candidate_parameter_value_map["ptz_y_min"])
        self.ptz_y_max = float(candidate_parameter_value_map["ptz_y_max"])
        self.ptz_x_reverse = convert_value_to_boolean(
            candidate_parameter_value_map["ptz_x_reverse"],
            self.ptz_x_reverse,
        )
        self.ptz_y_reverse = convert_value_to_boolean(
            candidate_parameter_value_map["ptz_y_reverse"],
            self.ptz_y_reverse,
        )
        self.init_publish_count = int(candidate_parameter_value_map["init_publish_count"])

        # PD and smoothing configuration / PD 与平滑控制配置
        self.deadband_x = float(candidate_parameter_value_map["deadband_x"])
        self.deadband_y = float(candidate_parameter_value_map["deadband_y"])
        self.pd_kp = max(float(candidate_parameter_value_map["pd_kp"]), 0.0)
        self.pd_kd = max(float(candidate_parameter_value_map["pd_kd"]), 0.0)
        self.ptz_step_smooth_alpha = clamp_numeric_value(
            float(candidate_parameter_value_map["ptz_step_smooth_alpha"]),
            0.0,
            1.0,
        )
        self.max_ptz_step = float(candidate_parameter_value_map["max_ptz_step"])
        self.target_smooth_alpha = clamp_numeric_value(
            float(candidate_parameter_value_map["target_smooth_alpha"]),
            0.01,
            1.0,
        )
        self.ptz_smooth_alpha = clamp_numeric_value(
            float(candidate_parameter_value_map["ptz_smooth_alpha"]),
            0.01,
            1.0,
        )
        self.ptz_control_rate = float(candidate_parameter_value_map["ptz_control_rate"])

        # Target lost handling configuration / 目标丢失处理配置
        self.lost_target_stop = convert_value_to_boolean(
            candidate_parameter_value_map["lost_target_stop"],
            self.lost_target_stop,
        )
        self.lost_timeout = max(
            float(candidate_parameter_value_map["lost_timeout"]),
            MINIMUM_TARGET_LOST_TIMEOUT_SECONDS,
        )

    def _recreate_ptz_control_timer(self, should_log_update: bool) -> None:
        """Recreate PTZ control timer using current control rate.

        按当前控制频率重建云台控制定时器。

        Args:
            should_log_update: Whether to log timer update / 是否打印定时器更新日志。
        """
        # Control timer period / 控制定时器周期
        control_timer_period_seconds = 1.0 / max(float(self.ptz_control_rate), MINIMUM_TIMER_FREQUENCY_HZ)

        if self.ptz_control_timer is not None:
            try:
                self.destroy_timer(self.ptz_control_timer)
            except Exception as exception:
                self.get_logger().warn(f"Destroy old PTZ timer failed: {exception}")

        # Fixed-rate PTZ control timer / 固定频率云台控制定时器
        self.ptz_control_timer = self.create_timer(
            control_timer_period_seconds,
            self._control_timer_callback,
        )

        if should_log_update:
            self.get_logger().info(
                f"PTZ control timer updated by rqt: rate={self.ptz_control_rate:.2f}Hz, "
                f"period={control_timer_period_seconds:.4f}s"
            )

    def _synchronize_debug_window_runtime_state(
        self,
        previous_display_debug_window: bool,
        previous_debug_window_size: Tuple[int, int],
    ) -> None:
        """Synchronize OpenCV debug window after dynamic parameter update.

        动态参数更新后同步 OpenCV 调试窗口状态。

        Args:
            previous_display_debug_window: Previous display switch / 之前的显示开关。
            previous_debug_window_size: Previous window size / 之前的窗口尺寸。
        """
        # Debug window size changed flag / 调试窗口尺寸变化标志
        debug_window_size_changed = previous_debug_window_size != (self.window_width, self.window_height)

        try:
            if self.display_debug_window and not previous_display_debug_window:
                cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
                cv.resizeWindow(self.window_name, self.window_width, self.window_height)
                cv.setMouseCallback(self.window_name, self._on_mouse_event)
                self.get_logger().info("Debug window enabled by rqt.")
            elif not self.display_debug_window and previous_display_debug_window:
                cv.destroyWindow(self.window_name)
                self.get_logger().info("Debug window disabled by rqt.")
            elif self.display_debug_window and debug_window_size_changed:
                cv.resizeWindow(self.window_name, self.window_width, self.window_height)
        except Exception as exception:
            self.get_logger().warn(f"Synchronize debug window failed: {exception}")

    def _reload_runtime_ros_parameters(self) -> None:
        """Reload runtime parameters for backward compatibility.

        以兼容旧逻辑的方式重新加载运行时参数。
        """
        # Candidate parameter value map / 候选参数值映射
        candidate_parameter_value_map = self._build_candidate_parameter_value_map([])

        # Reload all dynamic values / 重新加载全部动态参数值
        self._apply_runtime_parameter_update(
            candidate_parameter_value_map,
            set(RUNTIME_DYNAMIC_PARAMETER_NAMES),
        )

    def _get_hsv_range_from_parameters(self) -> HueSaturationValueRange:
        """Get HSV range from currently loaded parameter values.

        从当前已加载的参数值生成 HSV 范围。
        """
        # HSV lower threshold / HSV 下限阈值
        hsv_lower_threshold = (
            int(self.hue_minimum),
            int(self.saturation_minimum),
            int(self.value_minimum),
        )

        # HSV upper threshold / HSV 上限阈值
        hsv_upper_threshold = (
            int(self.hue_maximum),
            int(self.saturation_maximum),
            int(self.value_maximum),
        )

        return hsv_lower_threshold, hsv_upper_threshold

    def _apply_hsv_range_to_ros_parameters(
        self,
        hsv_threshold_range: HueSaturationValueRange,
    ) -> None:
        """Apply HSV range to ROS parameters and cached variables.

        将 HSV 范围写入 ROS 参数和缓存变量。
        """
        # HSV parameter list / HSV 参数列表
        hsv_parameter_list = [
            Parameter("Hmin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][0])),
            Parameter("Smin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][1])),
            Parameter("Vmin", Parameter.Type.INTEGER, int(hsv_threshold_range[0][2])),
            Parameter("Hmax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][0])),
            Parameter("Smax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][1])),
            Parameter("Vmax", Parameter.Type.INTEGER, int(hsv_threshold_range[1][2])),
        ]
        self.set_parameters(hsv_parameter_list)

        # Cached HSV lower values / 缓存的 HSV 下限值
        self.hue_minimum, self.saturation_minimum, self.value_minimum = hsv_threshold_range[0]

        # Cached HSV upper values / 缓存的 HSV 上限值
        self.hue_maximum, self.saturation_maximum, self.value_maximum = hsv_threshold_range[1]

    def _publish_initial_ptz_angle(self) -> None:
        """Publish initial PTZ angle several times after startup.

        启动后多次发布初始云台角度。
        """
        if self.remaining_initial_ptz_publish_count <= 0:
            try:
                self.initial_ptz_timer.cancel()
            except Exception:
                pass
            return

        self._publish_ptz_angle(
            self.current_ptz_horizontal_angle,
            self.current_ptz_vertical_angle,
        )
        self.remaining_initial_ptz_publish_count -= 1

    def _publish_ptz_angle(self, horizontal_angle: float, vertical_angle: float) -> None:
        """Publish PTZ angle command.

        发布云台角度命令。

        Args:
            horizontal_angle: PTZ horizontal angle / 云台水平角度。
            vertical_angle: PTZ vertical angle / 云台俯仰角度。
        """
        # PTZ command message / 云台控制消息
        ptz_angle_message = Vector3()
        ptz_angle_message.x = float(horizontal_angle)
        ptz_angle_message.y = float(vertical_angle)
        ptz_angle_message.z = 0.0
        self.ptz_angle_publisher.publish(ptz_angle_message)

    def _reset_target_state(self) -> None:
        """Reset all target detection state.

        重置所有目标检测状态。
        """
        with self.target_state_lock:
            self.raw_target_circle = (0, 0, 0)
            self.raw_target_area = 0.0
            self.filtered_target_center_x = None
            self.filtered_target_center_y = None
            self.filtered_target_radius = None
            self.target_is_valid = False
            self.last_valid_target_time_seconds = time.perf_counter()
        self._reset_pd_control_state()

    def _on_mouse_event(self, event, pixel_x, pixel_y, flags, param) -> None:
        """Handle OpenCV mouse event for ROI selection.

        处理 OpenCV 鼠标事件，用于 ROI 框选。

        Args:
            event: OpenCV mouse event / OpenCV 鼠标事件。
            pixel_x: Mouse x coordinate / 鼠标 x 坐标。
            pixel_y: Mouse y coordinate / 鼠标 y 坐标。
            flags: OpenCV event flags / OpenCV 事件标志。
            param: OpenCV callback parameter / OpenCV 回调参数。
        """
        del flags, param

        if event == cv.EVENT_LBUTTONDOWN:
            self.mouse_button_is_pressed = True
            self.roi_is_ready = False
            self.roi_start_pixel = (pixel_x, pixel_y)
            self.roi_end_pixel = (pixel_x, pixel_y)
            self.tracking_state = "init"
        elif event == cv.EVENT_MOUSEMOVE and self.mouse_button_is_pressed:
            self.roi_end_pixel = (pixel_x, pixel_y)
        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_button_is_pressed = False
            self.roi_end_pixel = (pixel_x, pixel_y)
            self.roi_is_ready = True

    def image_callback(self, image_message: Image) -> None:
        """Process every input image message.

        处理每一帧输入图像消息。

        Args:
            image_message: Raw or compressed ROS image message / 原始或压缩 ROS 图像消息。
        """
        # Decoded BGR frame / 解码后的 BGR 图像帧
        decoded_bgr_frame = self._decode_input_image_message(image_message)
        if decoded_bgr_frame is None:
            return

        if self.image_width > 0 and self.image_height > 0:
            decoded_bgr_frame = cv.resize(decoded_bgr_frame, (self.image_width, self.image_height))

        self._update_frame_rate()

        if self.display_debug_window:
            # Keyboard action code / 键盘操作码
            keyboard_action_code = cv.waitKey(1) & 0xFF
            self._handle_keyboard_action(keyboard_action_code)

        # Visualization frame / 可视化图像帧
        visualization_bgr_frame = decoded_bgr_frame.copy()

        # Binary mask image / 二值掩码图像
        binary_mask_image = None

        if self.tracking_state == "init":
            self._handle_roi_selection(visualization_bgr_frame)
            self._reset_target_state()
        else:
            (
                visualization_bgr_frame,
                binary_mask_image,
                detected_target_circle,
                detected_target_area,
            ) = self._detect_color_target(
                visualization_bgr_frame,
                self.hsv_threshold_range,
            )

            # Target validity flag / 目标有效标志
            detected_target_is_valid = self._is_target_valid(
                detected_target_circle,
                detected_target_area,
            )
            self._update_target_state(
                detected_target_circle,
                detected_target_area,
                detected_target_is_valid,
            )

        self._draw_overlay_information(visualization_bgr_frame)
        self._publish_debug_image(visualization_bgr_frame)
        self._show_debug_window(visualization_bgr_frame, binary_mask_image)

    def _decode_input_image_message(self, image_message: Image) -> Optional[np.ndarray]:
        """Convert raw Image to OpenCV BGR frame.

        将原始 Image 转换为 OpenCV BGR 图像。
        """
        try:
            return image_to_bgr(image_message)
        except Exception as exception:
            self.get_logger().error(f"Image decode failed: {exception}")
            return None

    def _update_frame_rate(self) -> None:
        """Update smoothed input frame rate.

        更新平滑后的输入帧率。
        """
        # Current monotonic time / 当前单调时间
        current_time_seconds = time.perf_counter()

        # Frame interval / 帧间隔
        frame_interval_seconds = max(current_time_seconds - self.last_frame_time_seconds, 1.0e-6)

        # Current frame rate / 当前帧率
        current_frame_rate = 1.0 / frame_interval_seconds

        self.smoothed_frame_rate = (
            0.85 * self.smoothed_frame_rate + 0.15 * current_frame_rate
            if self.smoothed_frame_rate > 0.0
            else current_frame_rate
        )
        self.last_frame_time_seconds = current_time_seconds

    def _handle_keyboard_action(self, keyboard_action_code: int) -> None:
        """Handle keyboard command from OpenCV window.

        处理 OpenCV 窗口键盘命令。
        """
        if keyboard_action_code in (ord("q"), ord("Q"), 27):
            self.get_logger().info("Quit key pressed.")
            os._exit(0)  # pylint: disable=protected-access
        elif keyboard_action_code == 32:
            if self.hsv_threshold_range is not None:
                self.tracking_state = "tracking"
                self._reset_pd_control_state()
                self.get_logger().info("Tracking mode enabled.")
        elif keyboard_action_code in (ord("i"), ord("I")):
            self.tracking_state = "identify"
            self._reset_pd_control_state()
            self.get_logger().info("Identify mode enabled.")
        elif keyboard_action_code in (ord("r"), ord("R")):
            self.tracking_state = "init"
            self.roi_is_ready = False
            self.mouse_button_is_pressed = False
            self._reset_pd_control_state()
            self._reset_target_state()
            self.get_logger().info("ROI sampling mode enabled. Drag mouse on target color.")

    def _handle_roi_selection(self, visualization_bgr_frame: np.ndarray) -> None:
        """Draw ROI rectangle and sample HSV after mouse selection.

        绘制 ROI 矩形，并在鼠标框选完成后采样 HSV。
        """
        if not (self.mouse_button_is_pressed or self.roi_is_ready):
            return

        # ROI left pixel / ROI 左边界像素
        roi_left_pixel = int(
            clamp_numeric_value(
                min(self.roi_start_pixel[0], self.roi_end_pixel[0]),
                0,
                visualization_bgr_frame.shape[1] - 1,
            )
        )

        # ROI top pixel / ROI 上边界像素
        roi_top_pixel = int(
            clamp_numeric_value(
                min(self.roi_start_pixel[1], self.roi_end_pixel[1]),
                0,
                visualization_bgr_frame.shape[0] - 1,
            )
        )

        # ROI right pixel / ROI 右边界像素
        roi_right_pixel = int(
            clamp_numeric_value(
                max(self.roi_start_pixel[0], self.roi_end_pixel[0]),
                0,
                visualization_bgr_frame.shape[1] - 1,
            )
        )

        # ROI bottom pixel / ROI 下边界像素
        roi_bottom_pixel = int(
            clamp_numeric_value(
                max(self.roi_start_pixel[1], self.roi_end_pixel[1]),
                0,
                visualization_bgr_frame.shape[0] - 1,
            )
        )

        cv.rectangle(
            visualization_bgr_frame,
            (roi_left_pixel, roi_top_pixel),
            (roi_right_pixel, roi_bottom_pixel),
            (0, 255, 0),
            2,
        )

        if self.roi_is_ready and (roi_right_pixel - roi_left_pixel) >= 3 and (roi_bottom_pixel - roi_top_pixel) >= 3:
            # Selected ROI / 选择的 ROI
            selected_region_of_interest = (
                roi_left_pixel,
                roi_top_pixel,
                roi_right_pixel,
                roi_bottom_pixel,
            )

            # Sampled HSV range / 采样得到的 HSV 范围
            sampled_hsv_threshold_range = self._sample_hsv_range_from_roi(
                visualization_bgr_frame,
                selected_region_of_interest,
            )
            if sampled_hsv_threshold_range is not None:
                self.hsv_threshold_range = sampled_hsv_threshold_range
                self._apply_hsv_range_to_ros_parameters(sampled_hsv_threshold_range)
                write_hsv_threshold_file(self.hsv_file, sampled_hsv_threshold_range)
                self.tracking_state = "identify"
                self.roi_is_ready = False
                self._reset_pd_control_state()
                self.get_logger().info(
                    f"HSV sampled and saved: {sampled_hsv_threshold_range}, file={self.hsv_file}"
                )
        elif self.roi_is_ready:
            self.roi_is_ready = False
            self.get_logger().warn("ROI too small. Please drag a larger single-color area.")

    def _sample_hsv_range_from_roi(
        self,
        frame_bgr: np.ndarray,
        region_of_interest: RegionOfInterestInPixels,
    ) -> Optional[HueSaturationValueRange]:
        """Sample HSV threshold range from ROI.

        从 ROI 区域采样 HSV 阈值范围。
        """
        # ROI pixel boundaries / ROI 像素边界
        roi_left_pixel, roi_top_pixel, roi_right_pixel, roi_bottom_pixel = region_of_interest

        # ROI BGR image / ROI 的 BGR 图像
        roi_bgr_image = frame_bgr[roi_top_pixel:roi_bottom_pixel, roi_left_pixel:roi_right_pixel]
        if roi_bgr_image.size == 0:
            return None

        # ROI HSV image / ROI 的 HSV 图像
        roi_hsv_image = cv.cvtColor(roi_bgr_image, cv.COLOR_BGR2HSV)

        # Hue channel / 色相通道
        hue_channel = roi_hsv_image[:, :, 0]

        # Saturation channel / 饱和度通道
        saturation_channel = roi_hsv_image[:, :, 1]

        # Value channel / 明度通道
        value_channel = roi_hsv_image[:, :, 2]

        # HSV lower and upper thresholds / HSV 下限和上限阈值
        hue_minimum = int(max(0, int(np.min(hue_channel)) - self.hue_expand))
        hue_maximum = int(min(179, int(np.max(hue_channel)) + self.hue_expand))
        saturation_minimum = int(max(0, int(np.min(saturation_channel)) - self.saturation_expand))
        saturation_maximum = int(min(255, int(np.max(saturation_channel)) + self.saturation_expand))
        value_minimum = int(max(0, int(np.min(value_channel)) - self.value_expand))
        value_maximum = int(min(255, int(np.max(value_channel)) + self.value_expand))

        # Keep upper bound high enough for brightness changes / 保持上限足够高以适应亮度变化
        saturation_maximum = max(saturation_maximum, 253)
        value_maximum = max(value_maximum, 253)

        return (
            (hue_minimum, saturation_minimum, value_minimum),
            (hue_maximum, saturation_maximum, value_maximum),
        )

    def _detect_color_target(
        self,
        frame_bgr: np.ndarray,
        hsv_threshold_range: HueSaturationValueRange,
    ) -> Tuple[np.ndarray, np.ndarray, TargetCircleInPixels, float]:
        """Detect the largest color target with HSV threshold.

        使用 HSV 阈值检测最大的颜色目标。
        """
        # HSV image / HSV 图像
        hsv_image = cv.cvtColor(frame_bgr, cv.COLOR_BGR2HSV)

        # HSV lower threshold array / HSV 下限阈值数组
        hsv_lower_threshold_array = np.array(hsv_threshold_range[0], dtype=np.uint8)

        # HSV upper threshold array / HSV 上限阈值数组
        hsv_upper_threshold_array = np.array(hsv_threshold_range[1], dtype=np.uint8)

        # Binary mask before morphology / 形态学处理前的二值掩码
        color_mask_image = cv.inRange(hsv_image, hsv_lower_threshold_array, hsv_upper_threshold_array)

        # Morphology kernel size / 形态学核尺寸
        morphology_kernel_size = max(1, int(self.morph_kernel_size))
        if morphology_kernel_size % 2 == 0:
            morphology_kernel_size += 1

        # Morphology kernel / 形态学核
        morphology_kernel = cv.getStructuringElement(
            cv.MORPH_RECT,
            (morphology_kernel_size, morphology_kernel_size),
        )

        # Closed mask image / 闭运算后的掩码图像
        closed_mask_image = cv.morphologyEx(color_mask_image, cv.MORPH_CLOSE, morphology_kernel)

        # Binary mask image / 二值掩码图像
        _, binary_mask_image = cv.threshold(closed_mask_image, 10, 255, cv.THRESH_BINARY)

        # External contours / 外部轮廓
        external_contours, _ = cv.findContours(
            binary_mask_image,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE,
        )
        if not external_contours:
            return frame_bgr, binary_mask_image, (0, 0, 0), 0.0

        # Largest contour / 最大轮廓
        largest_target_contour = max(external_contours, key=cv.contourArea)

        # Largest contour area / 最大轮廓面积
        largest_target_area = float(cv.contourArea(largest_target_contour))

        # Minimum area rectangle / 最小面积外接矩形
        minimum_area_rectangle = cv.minAreaRect(largest_target_contour)

        # Rectangle box points / 外接矩形顶点
        rectangle_box_points = cv.boxPoints(minimum_area_rectangle).astype(np.int32)

        # Minimum enclosing circle / 最小外接圆
        (target_center_x, target_center_y), target_radius = cv.minEnclosingCircle(rectangle_box_points)

        # Target circle in pixels / 目标圆像素坐标
        target_circle = (int(target_center_x), int(target_center_y), int(target_radius))

        # Draw target for debug only / 仅为调试绘制目标
        cv.drawContours(frame_bgr, [rectangle_box_points], 0, (0, 255, 0), 2)
        cv.circle(frame_bgr, (target_circle[0], target_circle[1]), target_circle[2], (255, 0, 255), 2)
        cv.circle(frame_bgr, (target_circle[0], target_circle[1]), 3, (0, 0, 255), -1)

        return frame_bgr, binary_mask_image, target_circle, largest_target_area

    def _is_target_valid(self, target_circle: TargetCircleInPixels, target_area: float) -> bool:
        """Check whether detected target satisfies minimum size conditions.

        检查检测目标是否满足最小尺寸条件。
        """
        # Target valid flag / 目标有效标志
        target_is_valid = target_circle[2] >= self.min_target_radius and target_area >= self.min_target_area
        return target_is_valid

    def _update_target_state(
        self,
        target_circle: TargetCircleInPixels,
        target_area: float,
        target_is_valid: bool,
    ) -> None:
        """Update raw and filtered target state.

        更新原始目标状态和滤波目标状态。
        """
        # Current monotonic time / 当前单调时间
        current_time_seconds = time.perf_counter()

        with self.target_state_lock:
            self.raw_target_circle = target_circle
            self.raw_target_area = target_area

            if target_is_valid:
                # Raw target center and radius / 原始目标中心和半径
                target_center_x, target_center_y, target_radius = target_circle

                # Target smoothing factor / 目标平滑系数
                target_smoothing_alpha = self.target_smooth_alpha

                if self.filtered_target_center_x is None:
                    self.filtered_target_center_x = float(target_center_x)
                    self.filtered_target_center_y = float(target_center_y)
                    self.filtered_target_radius = float(target_radius)
                else:
                    self.filtered_target_center_x = (
                        (1.0 - target_smoothing_alpha) * self.filtered_target_center_x
                        + target_smoothing_alpha * float(target_center_x)
                    )
                    self.filtered_target_center_y = (
                        (1.0 - target_smoothing_alpha) * self.filtered_target_center_y
                        + target_smoothing_alpha * float(target_center_y)
                    )
                    self.filtered_target_radius = (
                        (1.0 - target_smoothing_alpha) * self.filtered_target_radius
                        + target_smoothing_alpha * float(target_radius)
                    )
                self.target_is_valid = True
                self.last_valid_target_time_seconds = current_time_seconds
            else:
                # Keep the last valid target briefly to avoid sudden stop / 短时间保留上一帧有效目标，避免突然停顿
                if current_time_seconds - self.last_valid_target_time_seconds > self.lost_timeout:
                    self.target_is_valid = False
                    self.filtered_target_center_x = None
                    self.filtered_target_center_y = None
                    self.filtered_target_radius = None

    def _control_timer_callback(self) -> None:
        """Run fixed-rate PTZ control.

        执行固定频率云台控制。
        """
        if self.tracking_state != "tracking":
            return

        with self.target_state_lock:
            # Snapshot target state / 目标状态快照
            target_is_valid = self.target_is_valid
            target_age_seconds = time.perf_counter() - self.last_valid_target_time_seconds
            filtered_target_center_x = self.filtered_target_center_x
            filtered_target_center_y = self.filtered_target_center_y
            filtered_target_radius = self.filtered_target_radius

        if (
            not target_is_valid
            or filtered_target_center_x is None
            or filtered_target_center_y is None
            or filtered_target_radius is None
            or target_age_seconds > self.lost_timeout
        ):
            self._handle_lost_target()
            return

        # Filtered target circle / 滤波后的目标圆
        filtered_target_circle = (
            int(filtered_target_center_x),
            int(filtered_target_center_y),
            int(filtered_target_radius),
        )

        self._track_target_with_smooth_ptz(filtered_target_circle)

    def _track_target_with_smooth_ptz(self, target_circle: TargetCircleInPixels) -> None:
        """Track target with PD output and PTZ angle smoothing.

        使用 PD 输出和云台角度平滑追踪目标。
        """
        # Target center x, y / 目标中心 x、y
        target_center_x, target_center_y, _ = target_circle

        # Image center x, y / 图像中心 x、y
        image_center_x = self.image_width * 0.5
        image_center_y = self.image_height * 0.5

        # Pixel errors / 像素误差
        pixel_error_x = float(target_center_x - image_center_x)
        pixel_error_y = float(target_center_y - image_center_y)

        target_is_centered_x = abs(pixel_error_x) < self.deadband_x
        target_is_centered_y = abs(pixel_error_y) < self.deadband_y
        if abs(pixel_error_x) < self.deadband_x:
            pixel_error_x = 0.0
        if abs(pixel_error_y) < self.deadband_y:
            pixel_error_y = 0.0

        current_time_seconds = time.perf_counter()
        delta_time_seconds = (
            current_time_seconds - self.previous_control_time
            if self.previous_control_time > 0.0
            else 1.0 / max(float(self.ptz_control_rate), MINIMUM_TIMER_FREQUENCY_HZ)
        )
        delta_time_seconds = max(delta_time_seconds, 1.0e-3)

        normalized_error_x = pixel_error_x / image_center_x
        normalized_error_y = pixel_error_y / image_center_y
        derivative_x = (normalized_error_x - self.previous_error_x) / delta_time_seconds
        derivative_y = (normalized_error_y - self.previous_error_y) / delta_time_seconds

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
        self.previous_control_time = current_time_seconds
        self.last_step_x = step_x
        self.last_step_y = step_y

        # Desired PTZ angle / 期望云台角度
        desired_ptz_horizontal_angle = (
            self.current_ptz_horizontal_angle - step_x
            if self.ptz_x_reverse
            else self.current_ptz_horizontal_angle + step_x
        )
        desired_ptz_vertical_angle = (
            self.current_ptz_vertical_angle + step_y
            if self.ptz_y_reverse
            else self.current_ptz_vertical_angle - step_y
        )

        desired_ptz_horizontal_angle = clamp_numeric_value(
            desired_ptz_horizontal_angle,
            self.ptz_x_min,
            self.ptz_x_max,
        )
        desired_ptz_vertical_angle = clamp_numeric_value(
            desired_ptz_vertical_angle,
            self.ptz_y_min,
            self.ptz_y_max,
        )

        # PTZ smoothing factor / 云台平滑系数
        ptz_smoothing_alpha = self.ptz_smooth_alpha

        self.current_ptz_horizontal_angle = (
            (1.0 - ptz_smoothing_alpha) * self.current_ptz_horizontal_angle
            + ptz_smoothing_alpha * desired_ptz_horizontal_angle
        )
        self.current_ptz_vertical_angle = (
            (1.0 - ptz_smoothing_alpha) * self.current_ptz_vertical_angle
            + ptz_smoothing_alpha * desired_ptz_vertical_angle
        )

        self.current_ptz_horizontal_angle = clamp_numeric_value(
            self.current_ptz_horizontal_angle,
            self.ptz_x_min,
            self.ptz_x_max,
        )
        self.current_ptz_vertical_angle = clamp_numeric_value(
            self.current_ptz_vertical_angle,
            self.ptz_y_min,
            self.ptz_y_max,
        )

        self._publish_ptz_angle(
            self.current_ptz_horizontal_angle,
            self.current_ptz_vertical_angle,
        )

    def _handle_lost_target(self) -> None:
        """Handle target lost event.

        处理目标丢失事件。
        """
        self._reset_pd_control_state()

    def _reset_pd_control_state(self) -> None:
        """Reset PD control history.

        重置 PD 控制历史状态。
        """
        self.previous_error_x = 0.0
        self.previous_error_y = 0.0
        self.previous_control_time = 0.0
        self.last_step_x = 0.0
        self.last_step_y = 0.0

    def _draw_overlay_information(self, frame_bgr: np.ndarray) -> None:
        """Draw status information on debug image.

        在调试图像上绘制状态信息。
        """
        # Frame height and width / 图像高度和宽度
        frame_height, frame_width = frame_bgr.shape[:2]

        cv.line(
            frame_bgr,
            (frame_width // 2 - 10, frame_height // 2),
            (frame_width // 2 + 10, frame_height // 2),
            (255, 255, 0),
            1,
        )
        cv.line(
            frame_bgr,
            (frame_width // 2, frame_height // 2 - 10),
            (frame_width // 2, frame_height // 2 + 10),
            (255, 255, 0),
            1,
        )

        with self.target_state_lock:
            # Target overlay state / 目标叠加显示状态
            target_display_is_valid = (
                self.target_is_valid
                and time.perf_counter() - self.last_valid_target_time_seconds <= self.lost_timeout
            )
            raw_target_circle = self.raw_target_circle
            filtered_target_center_x = self.filtered_target_center_x
            filtered_target_center_y = self.filtered_target_center_y
            filtered_target_radius = self.filtered_target_radius

        if (
            target_display_is_valid
            and filtered_target_center_x is not None
            and filtered_target_center_y is not None
            and filtered_target_radius is not None
        ):
            cv.circle(
                frame_bgr,
                (int(filtered_target_center_x), int(filtered_target_center_y)),
                4,
                (0, 255, 255),
                -1,
            )
            target_status_text = (
                f"Target raw:{raw_target_circle} "
                f"filt:({filtered_target_center_x:.0f},{filtered_target_center_y:.0f},{filtered_target_radius:.0f})"
            )
        else:
            target_status_text = "Target: lost"

        if self.tracking_state == "init":
            action_hint_text = "ROI: drag target color | SPACE track | i identify | q quit"
        elif self.tracking_state == "tracking":
            action_hint_text = "Tracking | r ROI | i identify | q quit"
        else:
            action_hint_text = "Identify | r ROI | SPACE track | q quit"

        # Overlay text lines / 叠加显示文本行
        overlay_text_lines = [
            f"Mode:{self.tracking_state} FPS:{self.smoothed_frame_rate:.1f} "
            f"PTZHz:{self.ptz_control_rate:.0f}",
            f"PTZ x:{self.current_ptz_horizontal_angle:.1f} "
            f"y:{self.current_ptz_vertical_angle:.1f} step:{self.max_ptz_step:.1f}",
            f"PD kp:{self.pd_kp:.2f} kd:{self.pd_kd:.2f} smooth:{self.ptz_step_smooth_alpha:.2f}",
            target_status_text,
            f"HSV {self.hsv_threshold_range[0]}-{self.hsv_threshold_range[1]}",
            action_hint_text,
        ]

        # Overlay font / 叠加显示字体
        overlay_font = cv.FONT_HERSHEY_SIMPLEX

        # Overlay font scale / 叠加显示字体大小
        overlay_font_scale = max(0.25, float(self.overlay_font_scale))

        # Overlay text thickness / 叠加显示文字线宽
        overlay_text_thickness = 1

        # Overlay line height / 叠加显示行高
        overlay_line_height = max(12, int(self.overlay_line_height))

        # Overlay origin x, y / 叠加显示起点 x、y
        overlay_origin_x = max(0, int(self.overlay_margin_x))
        overlay_origin_y = max(overlay_line_height, int(self.overlay_margin_y))

        # Maximum overlay text width / 叠加显示文本最大宽度
        maximum_overlay_text_width = 0
        for overlay_text_line in overlay_text_lines:
            (text_width, _), _ = cv.getTextSize(
                overlay_text_line,
                overlay_font,
                overlay_font_scale,
                overlay_text_thickness,
            )
            maximum_overlay_text_width = max(maximum_overlay_text_width, text_width)

        # Overlay panel width and height / 叠加显示面板宽高
        overlay_panel_width = min(frame_width - overlay_origin_x - 4, maximum_overlay_text_width + 12)
        overlay_panel_height = len(overlay_text_lines) * overlay_line_height + 8

        if self.overlay_show_background and overlay_panel_width > 0 and overlay_panel_height > 0:
            # Overlay copy for transparent background / 用于透明背景的图像副本
            overlay_background_frame = frame_bgr.copy()
            cv.rectangle(
                overlay_background_frame,
                (overlay_origin_x - 4, max(0, overlay_origin_y - overlay_line_height + 2)),
                (
                    overlay_origin_x - 4 + overlay_panel_width,
                    min(frame_height - 1, overlay_origin_y - overlay_line_height + 2 + overlay_panel_height),
                ),
                (0, 0, 0),
                -1,
            )
            cv.addWeighted(overlay_background_frame, 0.35, frame_bgr, 0.65, 0, frame_bgr)

        # Current overlay text y / 当前叠加显示文本 y 坐标
        current_overlay_y = overlay_origin_y
        for overlay_text_line in overlay_text_lines:
            cv.putText(
                frame_bgr,
                overlay_text_line,
                (overlay_origin_x, current_overlay_y),
                overlay_font,
                overlay_font_scale,
                (0, 255, 255),
                overlay_text_thickness,
                cv.LINE_AA,
            )
            current_overlay_y += overlay_line_height

    def _publish_debug_image(self, frame_bgr: np.ndarray) -> None:
        """Publish debug image when enabled.

        在启用时发布调试图像。
        """
        if not self.publish_debug_image:
            return

        try:
            # Debug image message / 调试图像消息
            debug_image_message = self.cv_bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            self.debug_image_publisher.publish(debug_image_message)
        except Exception as exception:
            self.get_logger().warn(f"Publish debug image failed: {exception}")

    def _show_debug_window(
        self,
        frame_bgr: np.ndarray,
        binary_mask_image: Optional[np.ndarray],
    ) -> None:
        """Show OpenCV debug window when enabled.

        在启用时显示 OpenCV 调试窗口。
        """
        if not self.display_debug_window:
            return

        try:
            if self.show_binary_window and binary_mask_image is not None:
                # Binary mask converted to BGR / 转成 BGR 的二值掩码
                binary_mask_bgr_image = cv.cvtColor(binary_mask_image, cv.COLOR_GRAY2BGR)

                # Side-by-side display image / 并排显示图像
                display_image = np.hstack([frame_bgr, binary_mask_bgr_image])
            else:
                display_image = frame_bgr

            cv.imshow(self.window_name, display_image)
        except Exception as exception:
            self.get_logger().warn(
                f"OpenCV imshow failed, disable display_debug_window if running headless: {exception}"
            )

    def cleanup(self) -> None:
        """Cleanup node resources.

        清理节点资源。
        """
        self._publish_ptz_angle(
            self.current_ptz_horizontal_angle,
            self.current_ptz_vertical_angle,
        )
        if self.display_debug_window:
            cv.destroyAllWindows()


def main(args=None) -> None:
    """Run color tracker PTZ node.

    运行颜色追踪云台控制节点。

    Args:
        args: Optional ROS2 command line arguments / 可选 ROS2 命令行参数。
    """
    rclpy.init(args=args)

    # Color tracker PTZ node instance / 颜色追踪云台控制节点实例
    color_tracker_ptz_node = ColorTrackerPtzNode()

    try:
        rclpy.spin(color_tracker_ptz_node)
    except KeyboardInterrupt:
        pass
    finally:
        color_tracker_ptz_node.cleanup()
        color_tracker_ptz_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
