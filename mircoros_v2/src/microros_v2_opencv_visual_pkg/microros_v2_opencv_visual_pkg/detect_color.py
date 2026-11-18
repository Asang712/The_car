#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_color.py

中文说明:
    ROS2 Humble 颜色识别/检测节点。
    该版本已经去掉云台控制、PID、PTZ 角度发布、底盘 cmd_vel 控制等功能，
    只保留图像订阅、HSV 颜色检测、ROI 取色、画框/画圆显示和识别数量日志。

    本版本重点修复 q/ESC 退出卡死问题：
    1. q/ESC 不再调用 rclpy.shutdown()
    2. q/ESC 不再调用 cv.destroyAllWindows()
    3. q/ESC 直接 os._exit(0) 结束当前节点进程
    4. main 使用 spin_once 循环，不再使用阻塞式 rclpy.spin()
    5. 不再通过 cv.getWindowProperty 自动判断窗口关闭，避免启动时误退出

English Description:
    ROS2 Humble color detection node.
    This version removes PTZ control, PID, PTZ angle publishing, and base cmd_vel control.
    It only keeps image subscription, HSV color detection, ROI color sampling, visualization,
    and target-count logging.

    This version fixes q/ESC exit freeze:
    1. q/ESC no longer calls rclpy.shutdown()
    2. q/ESC no longer calls cv.destroyAllWindows()
    3. q/ESC directly exits the node process with os._exit(0)
    4. main uses spin_once loop instead of blocking rclpy.spin()
    5. It does not auto-exit when cv.getWindowProperty reports invisible,
       because some embedded OpenCV HighGUI backends report invisible at startup.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple, Union

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


# VISUAL_PACKAGE_NAME: 当前 OpenCV 视觉功能包名称 / Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "detect_color_node"

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# HSV threshold range type / HSV 阈值范围类型.
HueSaturationValueRange = Tuple[Tuple[int, int, int], Tuple[int, int, int]]

# Single OpenCV HSV color value (H: 0-179, S/V: 0-255).
HueSaturationValue = Tuple[int, int, int]

# Target circle type: center_x, center_y, radius in pixels / 目标圆类型：像素中心 x、像素中心 y、像素半径.
TargetCircleInPixels = Tuple[int, int, int]

# ROI type: left, top, right, bottom in pixels / ROI 类型：像素左、上、右、下边界.
RegionOfInterestInPixels = Tuple[int, int, int, int]

# Image message type accepted by this node / 本节点支持的图像消息类型.
ImageMessageType = Union[Image, CompressedImage]

# DYNAMIC_PARAMETER_NAMES: 支持运行时动态修改的参数 / Runtime dynamic parameters.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "display_debug_window",
    "publish_debug_image",
    "show_binary_window",
    "image_width",
    "image_height",
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
    "print_detection_log",
    "detection_log_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
    "enable_rqt_dynamic_parameters",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "use_compressed_image",
    "debug_image_topic",
    "window_name",
})


@dataclass
class ColorDetectionResult:
    """颜色识别结果 / Color detection result."""

    # is_color_detected: 是否检测到颜色目标 / Whether at least one color target is detected.
    is_color_detected: bool

    # target_count: 检测到的有效颜色目标数量 / Number of valid detected color targets.
    target_count: int = 0

    # main_target_circle: 主目标外接圆 (center_x, center_y, radius) / Main target circle.
    main_target_circle: TargetCircleInPixels = (0, 0, 0)

    # main_target_area: 主目标轮廓面积 / Main target contour area.
    main_target_area: float = 0.0

    # all_target_circles: 全部有效颜色目标外接圆 / All valid target circles.
    all_target_circles: Tuple[TargetCircleInPixels, ...] = ()

    # all_target_areas: 全部有效颜色目标轮廓面积 / All valid target contour areas.
    all_target_areas: Tuple[float, ...] = ()


@dataclass(frozen=True)
class RoiColorSample:
    """ROI HSV sampling result / ROI HSV 取色结果。"""

    hsv_threshold_range: HueSaturationValueRange
    representative_hsv: HueSaturationValue
    color_name: str


def hsv_to_color_name(hue: int, saturation: int, value: int) -> str:
    """Map an OpenCV HSV value to a common color name."""
    hue = max(0, min(179, int(hue)))
    saturation = max(0, min(255, int(saturation)))
    value = max(0, min(255, int(value)))

    if value < 46:
        return "black"
    if saturation < 43:
        return "white" if value >= 221 else "gray"

    if 10 <= hue <= 24 and value < 170:
        return "brown"
    if hue <= 9 or hue >= 170:
        return "red"
    if hue <= 24:
        return "orange"
    if hue <= 34:
        return "yellow"
    if hue <= 84:
        return "green"
    if hue <= 99:
        return "cyan"
    if hue <= 129:
        return "blue"
    if hue <= 159:
        return "purple"
    return "pink" if saturation < 150 else "magenta"


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
        normalized_text_value = input_value.strip().lower()
        if normalized_text_value in ("true", "1", "yes", "y", "on"):
            return True
        if normalized_text_value in ("false", "0", "no", "n", "off"):
            return False

    if isinstance(input_value, (int, float)):
        return bool(input_value)

    return default_boolean_value


def read_hsv_threshold_file(hsv_file_path: str) -> Optional[HueSaturationValueRange]:
    """
    从文件读取 HSV 阈值范围 / Read HSV threshold range from file.

    文件格式 / File format:
        Hmin, Smin, Vmin, Hmax, Smax, Vmax

    Args:
        hsv_file_path: HSV 阈值文件路径 / HSV threshold file path.

    Returns:
        有效时返回 HSV 范围，否则返回 None / HSV range if valid, otherwise None.
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

        hsv_lower_threshold = (
            int(clamp_numeric_value(hsv_integer_values[0], 0, 179)),
            int(clamp_numeric_value(hsv_integer_values[1], 0, 255)),
            int(clamp_numeric_value(hsv_integer_values[2], 0, 255)),
        )
        hsv_upper_threshold = (
            int(clamp_numeric_value(hsv_integer_values[3], 0, 179)),
            int(clamp_numeric_value(hsv_integer_values[4], 0, 255)),
            int(clamp_numeric_value(hsv_integer_values[5], 0, 255)),
        )

        if (
            hsv_lower_threshold[0] > hsv_upper_threshold[0]
            or hsv_lower_threshold[1] > hsv_upper_threshold[1]
            or hsv_lower_threshold[2] > hsv_upper_threshold[2]
        ):
            return None

        return hsv_lower_threshold, hsv_upper_threshold
    except Exception:
        return None


def write_hsv_threshold_file(
    hsv_file_path: str,
    hsv_threshold_range: HueSaturationValueRange,
) -> None:
    """
    将 HSV 阈值范围写入文件 / Write HSV threshold range to file.

    Args:
        hsv_file_path: HSV 阈值文件路径 / HSV threshold file path.
        hsv_threshold_range: HSV 阈值范围 / HSV threshold range.
    """
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


class DetectColorNode(Node):
    """ROS2 颜色识别节点 / ROS2 color detection node."""

    def __init__(self) -> None:
        """初始化节点 / Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # cv_bridge: OpenCV 与 ROS Image 互转桥 / Bridge between OpenCV image and ROS Image.
        self.cv_bridge = CvBridge()

        # is_shutdown_requested: 是否请求关闭节点 / Whether shutdown is requested.
        self.is_shutdown_requested = False

        # operation_mode: 当前操作模式 detect/sample / Current operation mode.
        self.operation_mode = "detect"

        # previous_header_stamp_sec: 上一帧 header 时间戳 / Previous frame header timestamp.
        self.previous_header_stamp_sec = 0.0

        # header_based_frame_rate: 基于 header 时间戳的帧率 / Header stamp based FPS.
        self.header_based_frame_rate = 0.0

        # receive_based_frame_rate: 基于接收时间的帧率 / Receive time based FPS.
        self.receive_based_frame_rate = 0.0

        # previous_receive_time_sec: 上一帧接收时间 / Previous receive time.
        self.previous_receive_time_sec = 0.0

        # image_stamp_delay_ms: 图像 header 到接收时刻延迟 / Image header-to-receive delay in ms.
        self.image_stamp_delay_ms = 0.0

        # last_detection_log_time_sec: 上一次识别日志时间 / Last detection log time.
        self.last_detection_log_time_sec = 0.0

        # mouse_button_is_pressed: 鼠标是否按下 / Whether mouse button is pressed.
        self.mouse_button_is_pressed = False

        # roi_start_pixel: ROI 起始点 / ROI start pixel.
        self.roi_start_pixel = (0, 0)

        # roi_end_pixel: ROI 结束点 / ROI end pixel.
        self.roi_end_pixel = (0, 0)

        # roi_is_ready: ROI 是否已完成 / Whether ROI is ready.
        self.roi_is_ready = False

        # Last representative HSV value and common color name sampled from an ROI.
        self.last_sampled_hsv: Optional[HueSaturationValue] = None
        self.last_sampled_color_name = ""

        # latest_detection_result: 最近一次颜色检测结果 / Latest color detection result.
        self.latest_detection_result = ColorDetectionResult(False)

        # keyboard_poll_timer: OpenCV 键盘轮询定时器 / OpenCV keyboard polling timer.
        self.keyboard_poll_timer = None

        self._declare_ros_parameters()
        self._load_ros_parameters()

        # hsv_threshold_range: 当前 HSV 阈值范围 / Current HSV threshold range.
        self.hsv_threshold_range: HueSaturationValueRange = self._get_hsv_range_from_parameters()

        # 优先从 hsv_file 加载阈值，如果文件不存在则使用 launch/参数中的 HSV。
        # Prefer HSV file. If file does not exist, use HSV values from launch/parameters.
        saved_hsv_threshold_range = read_hsv_threshold_file(self.hsv_file)
        if saved_hsv_threshold_range is not None:
            self.hsv_threshold_range = saved_hsv_threshold_range
            self._apply_hsv_range_to_ros_parameters(saved_hsv_threshold_range)
            self.get_logger().info(
                f"Loaded HSV from file: {self.hsv_file}, hsv={self.hsv_threshold_range}"
            )
        else:
            self.get_logger().warn(
                "No valid HSV file found. Using HSV parameters from launch. "
                "Press r and drag ROI to sample target color."
            )

        # subscribed_image_is_compressed: 当前订阅输入是否为压缩图像 / Whether subscribed input is compressed.
        self.subscribed_image_is_compressed = bool(self.use_compressed_image)
        subscribed_image_message_type = CompressedImage if self.subscribed_image_is_compressed else Image

        # image_subscription: 图像订阅器 / Image subscriber.
        self.image_subscription = self.create_subscription(
            subscribed_image_message_type,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        # debug_image_publisher: 调试图像发布器 / Debug image publisher.
        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic_name, 10)

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        self._initialize_debug_window()

        # 低帧率或摄像头停止出图时，图像回调不会执行，所以增加一个独立按键轮询定时器。
        # If camera FPS is low or frames stop, image_callback may not run, so poll keys independently.
        if self.display_debug_window:
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)

        self.get_logger().info(
            "Detect color node started. "
            "Only HSV color detection is enabled; PTZ/base control has been removed. "
            "Press r to sample ROI, q or ESC to quit. "
            f"image_topic={self.image_topic_name}, use_compressed_image={self.subscribed_image_is_compressed}, "
            f"hsv={self.hsv_threshold_range}"
        )

    def _declare_ros_parameters(self) -> None:
        """声明 ROS 参数 / Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_compressed_image", False)
        self.declare_parameter("hsv_file", "")
        self.declare_parameter("window_name", "detect_color_node")
        self.declare_parameter("display_debug_window", True)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/detect_color/debug_image")
        self.declare_parameter("show_binary_window", False)
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 14)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)

        self.declare_parameter("img_flip", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

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

        self.declare_parameter("print_detection_log", True)
        self.declare_parameter("detection_log_interval_sec", 1.0)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_ros_parameters(self) -> None:
        """读取 ROS 参数 / Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
        self.use_compressed_image = convert_value_to_boolean(
            self.get_parameter("use_compressed_image").value,
            False,
        )
        self.hsv_file = os.path.expanduser(str(self.get_parameter("hsv_file").value))
        self.window_name = str(self.get_parameter("window_name").value)

        self.display_debug_window = convert_value_to_boolean(
            self.get_parameter("display_debug_window").value,
            True,
        )
        self.publish_debug_image = convert_value_to_boolean(
            self.get_parameter("publish_debug_image").value,
            False,
        )
        self.debug_image_topic_name = str(self.get_parameter("debug_image_topic").value)
        self.show_binary_window = convert_value_to_boolean(
            self.get_parameter("show_binary_window").value,
            False,
        )
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = convert_value_to_boolean(
            self.get_parameter("overlay_show_background").value,
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

        self.hue_minimum = int(clamp_numeric_value(int(self.get_parameter("Hmin").value), 0, 179))
        self.saturation_minimum = int(clamp_numeric_value(int(self.get_parameter("Smin").value), 0, 255))
        self.value_minimum = int(clamp_numeric_value(int(self.get_parameter("Vmin").value), 0, 255))
        self.hue_maximum = int(clamp_numeric_value(int(self.get_parameter("Hmax").value), 0, 179))
        self.saturation_maximum = int(clamp_numeric_value(int(self.get_parameter("Smax").value), 0, 255))
        self.value_maximum = int(clamp_numeric_value(int(self.get_parameter("Vmax").value), 0, 255))
        self.hue_expand = max(int(self.get_parameter("h_expand").value), 0)
        self.saturation_expand = max(int(self.get_parameter("s_expand").value), 0)
        self.value_expand = max(int(self.get_parameter("v_expand").value), 0)
        self.min_target_radius = max(float(self.get_parameter("min_target_radius").value), 0.0)
        self.min_target_area = max(float(self.get_parameter("min_target_area").value), 0.0)
        self.morph_kernel_size = max(int(self.get_parameter("morph_kernel_size").value), 1)

        self.print_detection_log = convert_value_to_boolean(
            self.get_parameter("print_detection_log").value,
            True,
        )
        self.detection_log_interval_sec = max(
            float(self.get_parameter("detection_log_interval_sec").value),
            0.05,
        )
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

    def _initialize_debug_window(self) -> None:
        """初始化 OpenCV 窗口 / Initialize OpenCV window."""
        if not self.display_debug_window:
            return

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, self.window_width, self.window_height)
        cv.setMouseCallback(self.window_name, self._on_mouse_event)

    def _update_debug_window(self) -> None:
        """更新 OpenCV 窗口 / Update OpenCV window."""
        if not self.display_debug_window:
            # 某些板子上 destroyWindow/destroyAllWindows 可能卡死，所以这里不主动销毁窗口。
            # On some boards destroyWindow/destroyAllWindows may freeze, so do not destroy actively.
            return

        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, self.window_width, self.window_height)
        cv.setMouseCallback(self.window_name, self._on_mouse_event)

        if self.keyboard_poll_timer is None:
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)

    def _keyboard_poll_timer_callback(self) -> None:
        """独立轮询 OpenCV 按键，避免低帧率时 q/ESC 无响应 / Poll OpenCV keys independently."""
        if self.is_shutdown_requested or not self.display_debug_window:
            return

        try:
            key_code = cv.waitKey(1) & 0xFF
            self._handle_key_input(key_code)

            # 不再使用 cv.getWindowProperty 判断窗口是否关闭。
            # Some OpenCV HighGUI backends return invisible immediately after startup,
            # especially on embedded boards or remote desktop sessions.
            # Exit is handled only by q/ESC to avoid false shutdown.

        except Exception:
            pass

    def _get_hsv_range_from_parameters(self) -> HueSaturationValueRange:
        """从参数生成 HSV 阈值范围 / Get HSV range from parameters."""
        return (
            (
                int(self.hue_minimum),
                int(self.saturation_minimum),
                int(self.value_minimum),
            ),
            (
                int(self.hue_maximum),
                int(self.saturation_maximum),
                int(self.value_maximum),
            ),
        )

    def _apply_hsv_range_to_ros_parameters(
        self,
        hsv_threshold_range: HueSaturationValueRange,
    ) -> None:
        """
        将 HSV 范围写入 ROS 参数和缓存变量 / Apply HSV range to ROS parameters and cached variables.

        Args:
            hsv_threshold_range: HSV 阈值范围 / HSV threshold range.
        """
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

    def image_callback(self, image_message: ImageMessageType) -> None:
        """
        接收、解码并识别颜色目标 / Receive, decode, and detect color targets.

        Args:
            image_message: 原始或压缩图像消息 / Raw or compressed image message.
        """
        if self.is_shutdown_requested:
            return

        try:
            image_frame = self._decode_input_image_message(image_message)
            if image_frame is None:
                return

            self._update_image_timing_statistics(image_message)
            image_frame = self._apply_image_orientation(image_frame)

            if self.image_width > 0 and self.image_height > 0:
                image_frame = cv.resize(
                    image_frame,
                    (int(self.image_width), int(self.image_height)),
                    interpolation=cv.INTER_AREA,
                )

            display_image = image_frame.copy()
            binary_mask_image: Optional[np.ndarray] = None

            if self.operation_mode == "sample":
                self._handle_roi_selection(display_image)
                detection_result = ColorDetectionResult(False)
            else:
                detection_result, binary_mask_image = self.detect_color_targets(image_frame)
                self.latest_detection_result = detection_result
                self._draw_detection_result(display_image, detection_result)
                self._print_detection_log_if_needed(detection_result)

            self._draw_overlay_text(display_image, detection_result)
            self._publish_debug_image(display_image)

            if self.display_debug_window:
                self._show_debug_window(display_image, binary_mask_image)

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def _decode_input_image_message(self, image_message: ImageMessageType) -> Optional[np.ndarray]:
        """
        解码输入图像消息为 OpenCV BGR 图像 / Decode input image message to OpenCV BGR image.

        Args:
            image_message: 原始或压缩图像消息 / Raw or compressed image message.

        Returns:
            BGR 图像，失败返回 None / BGR image, or None on failure.
        """
        try:
            if self.subscribed_image_is_compressed:
                compressed_image_buffer = np.frombuffer(image_message.data, dtype=np.uint8)
                decoded_bgr_frame = cv.imdecode(compressed_image_buffer, cv.IMREAD_COLOR)
                if decoded_bgr_frame is None:
                    self.get_logger().warn("Compressed image decode failed.")
                    return None
                return decoded_bgr_frame

            return self.cv_bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image decode failed: {exception_error}")
            return None

    def _apply_image_orientation(self, image_frame: np.ndarray) -> np.ndarray:
        """
        应用图像方向修正 / Apply image orientation correction.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            方向修正后的图像 / Orientation-corrected image.
        """
        corrected_image_frame = image_frame

        if self.img_flip or self.enable_horizontal_flip:
            corrected_image_frame = cv.flip(corrected_image_frame, 1)
        if self.enable_vertical_flip:
            corrected_image_frame = cv.flip(corrected_image_frame, 0)

        if self.rotate_image_code == "90_clockwise":
            corrected_image_frame = cv.rotate(corrected_image_frame, cv.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code == "90_counterclockwise":
            corrected_image_frame = cv.rotate(corrected_image_frame, cv.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code == "180":
            corrected_image_frame = cv.rotate(corrected_image_frame, cv.ROTATE_180)

        return corrected_image_frame

    def _update_image_timing_statistics(self, image_message: ImageMessageType) -> None:
        """
        更新图像帧率和延迟统计 / Update image FPS and delay statistics.

        Args:
            image_message: 原始或压缩图像消息 / Raw or compressed image message.
        """
        current_receive_time_sec = time.monotonic()
        if self.previous_receive_time_sec > 0.0:
            receive_interval_sec = current_receive_time_sec - self.previous_receive_time_sec
            if receive_interval_sec > 0.0:
                self.receive_based_frame_rate = 1.0 / receive_interval_sec
        self.previous_receive_time_sec = current_receive_time_sec

        try:
            header_stamp_sec = (
                float(image_message.header.stamp.sec)
                + float(image_message.header.stamp.nanosec) * 1e-9
            )
        except Exception:  # pylint: disable=broad-except
            header_stamp_sec = 0.0

        if header_stamp_sec > 0.0:
            if self.previous_header_stamp_sec > 0.0:
                header_interval_sec = header_stamp_sec - self.previous_header_stamp_sec
                if header_interval_sec > 0.0:
                    self.header_based_frame_rate = 1.0 / header_interval_sec
            self.previous_header_stamp_sec = header_stamp_sec

            node_now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.image_stamp_delay_ms = max((node_now_sec - header_stamp_sec) * 1000.0, 0.0)

    def detect_color_targets(
        self,
        image_frame: np.ndarray,
    ) -> Tuple[ColorDetectionResult, np.ndarray]:
        """
        检测图像中的颜色目标 / Detect color targets in image.

        Args:
            image_frame: 输入 BGR 图像 / Input BGR image.

        Returns:
            颜色检测结果和二值掩码 / Color detection result and binary mask.
        """
        hsv_image = cv.cvtColor(image_frame, cv.COLOR_BGR2HSV)

        hsv_lower_threshold_array = np.array(self.hsv_threshold_range[0], dtype=np.uint8)
        hsv_upper_threshold_array = np.array(self.hsv_threshold_range[1], dtype=np.uint8)

        color_mask_image = cv.inRange(
            hsv_image,
            hsv_lower_threshold_array,
            hsv_upper_threshold_array,
        )

        morphology_kernel_size = max(1, int(self.morph_kernel_size))
        if morphology_kernel_size % 2 == 0:
            morphology_kernel_size += 1

        morphology_kernel = cv.getStructuringElement(
            cv.MORPH_RECT,
            (morphology_kernel_size, morphology_kernel_size),
        )

        opened_mask_image = cv.morphologyEx(color_mask_image, cv.MORPH_OPEN, morphology_kernel)
        closed_mask_image = cv.morphologyEx(opened_mask_image, cv.MORPH_CLOSE, morphology_kernel)
        _, binary_mask_image = cv.threshold(closed_mask_image, 10, 255, cv.THRESH_BINARY)

        external_contours, _ = cv.findContours(
            binary_mask_image,
            cv.RETR_EXTERNAL,
            cv.CHAIN_APPROX_SIMPLE,
        )

        valid_targets: List[Tuple[float, TargetCircleInPixels]] = []
        for target_contour in external_contours:
            target_area = float(cv.contourArea(target_contour))
            if target_area < self.min_target_area:
                continue

            (target_center_x, target_center_y), target_radius = cv.minEnclosingCircle(target_contour)
            target_circle = (
                int(round(target_center_x)),
                int(round(target_center_y)),
                int(round(target_radius)),
            )
            if target_circle[2] < self.min_target_radius:
                continue

            valid_targets.append((target_area, target_circle))

        if not valid_targets:
            return ColorDetectionResult(False), binary_mask_image

        valid_targets.sort(key=lambda target: target[0], reverse=True)
        all_target_areas = tuple(float(target[0]) for target in valid_targets)
        all_target_circles = tuple(target[1] for target in valid_targets)
        main_target_area = all_target_areas[0]
        main_target_circle = all_target_circles[0]

        return (
            ColorDetectionResult(
                is_color_detected=True,
                target_count=len(valid_targets),
                main_target_circle=main_target_circle,
                main_target_area=main_target_area,
                all_target_circles=all_target_circles,
                all_target_areas=all_target_areas,
            ),
            binary_mask_image,
        )

    def _draw_detection_result(
        self,
        display_image: np.ndarray,
        detection_result: ColorDetectionResult,
    ) -> None:
        """
        绘制颜色检测结果 / Draw color detection result.

        Args:
            display_image: 显示图像 / Display image.
            detection_result: 颜色检测结果 / Color detection result.
        """
        for target_index, target_circle in enumerate(detection_result.all_target_circles, start=1):
            target_center_x, target_center_y, target_radius = target_circle
            circle_color = (0, 255, 0)
            circle_thickness = 2

            if target_circle == detection_result.main_target_circle:
                circle_color = (255, 0, 255)
                circle_thickness = 2

            cv.circle(
                display_image,
                (target_center_x, target_center_y),
                target_radius,
                circle_color,
                circle_thickness,
            )
            cv.circle(
                display_image,
                (target_center_x, target_center_y),
                3,
                (0, 0, 255),
                -1,
            )
            cv.putText(
                display_image,
                f"Color {target_index}",
                (target_center_x + 5, max(target_center_y - 8, 15)),
                cv.FONT_HERSHEY_SIMPLEX,
                0.35,
                circle_color,
                1,
                cv.LINE_AA,
            )

    def _print_detection_log_if_needed(self, detection_result: ColorDetectionResult) -> None:
        """
        按固定间隔打印识别结果 / Print detection result periodically.

        Args:
            detection_result: 颜色检测结果 / Color detection result.
        """
        if not self.print_detection_log:
            return

        current_time_sec = time.monotonic()
        if current_time_sec - self.last_detection_log_time_sec < self.detection_log_interval_sec:
            return

        self.last_detection_log_time_sec = current_time_sec
        if detection_result.is_color_detected:
            self.get_logger().info(
                f"Detected color targets: {detection_result.target_count}, "
                f"main_circle={detection_result.main_target_circle}, "
                f"main_area={detection_result.main_target_area:.1f}, "
                f"hsv={self.hsv_threshold_range}"
            )
        else:
            self.get_logger().info(f"Detected color targets: 0, hsv={self.hsv_threshold_range}")

    def _draw_overlay_text(
        self,
        display_image: np.ndarray,
        detection_result: ColorDetectionResult,
    ) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            display_image: 显示图像 / Display image.
            detection_result: 颜色检测结果 / Color detection result.
        """
        frame_height, frame_width = display_image.shape[:2]

        cv.line(
            display_image,
            (frame_width // 2 - 10, frame_height // 2),
            (frame_width // 2 + 10, frame_height // 2),
            (255, 255, 0),
            1,
        )
        cv.line(
            display_image,
            (frame_width // 2, frame_height // 2 - 10),
            (frame_width // 2, frame_height // 2 + 10),
            (255, 255, 0),
            1,
        )

        frame_rate_value = self.header_based_frame_rate if self.use_header_stamp_for_fps else self.receive_based_frame_rate

        if self.operation_mode == "sample":
            action_hint_text = "ROI: drag target color | q/ESC quit"
        else:
            action_hint_text = "r ROI sample | q/ESC quit"

        overlay_lines = [
            f"mode: color detection only / {self.operation_mode}",
            f"targets: {detection_result.target_count}",
            f"fps: {frame_rate_value:.1f}",
            f"hsv: {self.hsv_threshold_range[0]} - {self.hsv_threshold_range[1]}",
            action_hint_text,
        ]
        if self.last_sampled_hsv is not None:
            overlay_lines.insert(
                1,
                f"sample: {self.last_sampled_color_name} | HSV: {self.last_sampled_hsv}",
            )
        if self.show_image_stamp_delay:
            overlay_lines.insert(3, f"delay: {self.image_stamp_delay_ms:.1f}ms")

        if detection_result.is_color_detected:
            overlay_lines.insert(
                3,
                f"main: circle={detection_result.main_target_circle}, area={detection_result.main_target_area:.0f}",
            )

        if self.overlay_show_background:
            background_height = max(self.overlay_line_height * len(overlay_lines) + 8, 1)
            cv.rectangle(
                display_image,
                (0, 0),
                (min(display_image.shape[1], 620), min(display_image.shape[0], background_height)),
                (0, 0, 0),
                -1,
            )

        for line_index, overlay_text in enumerate(overlay_lines):
            text_y = self.overlay_margin_y + line_index * self.overlay_line_height
            cv.putText(
                display_image,
                overlay_text,
                (self.overlay_margin_x, text_y),
                cv.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv.LINE_AA,
            )

    def _publish_debug_image(self, display_image: np.ndarray) -> None:
        """
        发布调试图像 / Publish debug image.

        Args:
            display_image: 调试图像 / Debug image.
        """
        if not self.publish_debug_image:
            return

        try:
            debug_image_message = self.cv_bridge.cv2_to_imgmsg(display_image, encoding="bgr8")
            self.debug_image_publisher.publish(debug_image_message)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Publish debug image failed: {exception_error}")

    def _show_debug_window(
        self,
        display_image: np.ndarray,
        binary_mask_image: Optional[np.ndarray],
    ) -> None:
        """
        显示 OpenCV 调试窗口 / Show OpenCV debug window.

        Args:
            display_image: 显示图像 / Display image.
            binary_mask_image: 二值掩码图像 / Binary mask image.
        """
        try:
            if self.show_binary_window and binary_mask_image is not None:
                binary_mask_bgr_image = cv.cvtColor(binary_mask_image, cv.COLOR_GRAY2BGR)
                shown_image = np.hstack([display_image, binary_mask_bgr_image])
            else:
                shown_image = display_image

            cv.imshow(self.window_name, shown_image)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"OpenCV imshow failed, disable display_debug_window if running headless: {exception_error}"
            )

    def _on_mouse_event(self, event, pixel_x, pixel_y, flags, param) -> None:
        """
        处理 OpenCV 鼠标事件，用于 ROI 取色 / Handle OpenCV mouse event for ROI sampling.

        Args:
            event: OpenCV 鼠标事件 / OpenCV mouse event.
            pixel_x: 鼠标 x 坐标 / Mouse x coordinate.
            pixel_y: 鼠标 y 坐标 / Mouse y coordinate.
            flags: OpenCV 事件标志 / OpenCV event flags.
            param: OpenCV 回调参数 / OpenCV callback parameter.
        """
        del flags, param

        if event == cv.EVENT_LBUTTONDOWN:
            self.mouse_button_is_pressed = True
            self.roi_is_ready = False
            self.roi_start_pixel = (pixel_x, pixel_y)
            self.roi_end_pixel = (pixel_x, pixel_y)
            self.operation_mode = "sample"
        elif event == cv.EVENT_MOUSEMOVE and self.mouse_button_is_pressed:
            self.roi_end_pixel = (pixel_x, pixel_y)
        elif event == cv.EVENT_LBUTTONUP:
            self.mouse_button_is_pressed = False
            self.roi_end_pixel = (pixel_x, pixel_y)
            self.roi_is_ready = True

    def _handle_roi_selection(self, display_image: np.ndarray) -> None:
        """
        处理 ROI 框选和 HSV 取色 / Handle ROI selection and HSV sampling.

        Args:
            display_image: 显示图像 / Display image.
        """
        if not (self.mouse_button_is_pressed or self.roi_is_ready):
            return

        roi_left_pixel = int(
            clamp_numeric_value(
                min(self.roi_start_pixel[0], self.roi_end_pixel[0]),
                0,
                display_image.shape[1] - 1,
            )
        )
        roi_top_pixel = int(
            clamp_numeric_value(
                min(self.roi_start_pixel[1], self.roi_end_pixel[1]),
                0,
                display_image.shape[0] - 1,
            )
        )
        roi_right_pixel = int(
            clamp_numeric_value(
                max(self.roi_start_pixel[0], self.roi_end_pixel[0]),
                0,
                display_image.shape[1] - 1,
            )
        )
        roi_bottom_pixel = int(
            clamp_numeric_value(
                max(self.roi_start_pixel[1], self.roi_end_pixel[1]),
                0,
                display_image.shape[0] - 1,
            )
        )

        if self.roi_is_ready and (roi_right_pixel - roi_left_pixel) >= 3 and (roi_bottom_pixel - roi_top_pixel) >= 3:
            selected_region_of_interest = (
                roi_left_pixel,
                roi_top_pixel,
                roi_right_pixel,
                roi_bottom_pixel,
            )

            color_sample = self._sample_hsv_range_from_roi(
                display_image,
                selected_region_of_interest,
            )
            if color_sample is not None:
                self.hsv_threshold_range = color_sample.hsv_threshold_range
                self.last_sampled_hsv = color_sample.representative_hsv
                self.last_sampled_color_name = color_sample.color_name
                self._apply_hsv_range_to_ros_parameters(color_sample.hsv_threshold_range)
                write_hsv_threshold_file(self.hsv_file, color_sample.hsv_threshold_range)
                self.operation_mode = "detect"
                self.roi_is_ready = False
                self.get_logger().info(
                    f"Color sampled: {self.last_sampled_color_name}, "
                    f"representative_hsv={self.last_sampled_hsv}, "
                    f"threshold={self.hsv_threshold_range}, file={self.hsv_file}"
                )
        elif self.roi_is_ready:
            self.roi_is_ready = False
            self.operation_mode = "detect"
            self.get_logger().warn("ROI too small. Please drag a larger single-color area.")

        cv.rectangle(
            display_image,
            (roi_left_pixel, roi_top_pixel),
            (roi_right_pixel, roi_bottom_pixel),
            (0, 255, 0),
            2,
        )

    def _sample_hsv_range_from_roi(
        self,
        image_frame: np.ndarray,
        region_of_interest: RegionOfInterestInPixels,
    ) -> Optional[RoiColorSample]:
        """
        从 ROI 区域采样 HSV 阈值范围 / Sample HSV threshold range from ROI.

        Args:
            image_frame: 输入 BGR 图像 / Input BGR image.
            region_of_interest: ROI 区域 / ROI region.

        Returns:
            ROI 取色结果，失败返回 None / ROI color sample, or None on failure.
        """
        roi_left_pixel, roi_top_pixel, roi_right_pixel, roi_bottom_pixel = region_of_interest

        roi_bgr_image = image_frame[roi_top_pixel:roi_bottom_pixel, roi_left_pixel:roi_right_pixel]
        if roi_bgr_image.size == 0:
            return None

        roi_hsv_image = cv.cvtColor(roi_bgr_image, cv.COLOR_BGR2HSV)

        hue_channel = roi_hsv_image[:, :, 0]
        saturation_channel = roi_hsv_image[:, :, 1]
        value_channel = roi_hsv_image[:, :, 2]

        # Hue is circular: 179 and 0 are both red, so use a circular mean.
        hue_angles = hue_channel.astype(np.float32) * (2.0 * np.pi / 180.0)
        mean_hue_angle = np.arctan2(
            float(np.mean(np.sin(hue_angles))),
            float(np.mean(np.cos(hue_angles))),
        )
        normalized_hue_angle = mean_hue_angle % (2.0 * np.pi)
        hue_angle_to_opencv_scale = 180.0 / (2.0 * np.pi)
        representative_hue = int(
            round(normalized_hue_angle * hue_angle_to_opencv_scale)
        ) % 180
        representative_hsv = (
            representative_hue,
            int(round(float(np.median(saturation_channel)))),
            int(round(float(np.median(value_channel)))),
        )

        hue_minimum = int(max(0, int(np.min(hue_channel)) - self.hue_expand))
        hue_maximum = int(min(179, int(np.max(hue_channel)) + self.hue_expand))
        saturation_minimum = int(max(0, int(np.min(saturation_channel)) - self.saturation_expand))
        saturation_maximum = int(min(255, int(np.max(saturation_channel)) + self.saturation_expand))
        value_minimum = int(max(0, int(np.min(value_channel)) - self.value_expand))
        value_maximum = int(min(255, int(np.max(value_channel)) + self.value_expand))

        # 保持上限较高，增强亮度变化适应能力。
        # Keep upper bound high enough for lighting changes.
        saturation_maximum = max(saturation_maximum, 253)
        value_maximum = max(value_maximum, 253)

        hsv_threshold_range = (
            (hue_minimum, saturation_minimum, value_minimum),
            (hue_maximum, saturation_maximum, value_maximum),
        )
        return RoiColorSample(
            hsv_threshold_range=hsv_threshold_range,
            representative_hsv=representative_hsv,
            color_name=hsv_to_color_name(*representative_hsv),
        )

    def _handle_key_input(self, key_code: int) -> None:
        """
        处理键盘输入 / Handle keyboard input.

        Args:
            key_code: OpenCV 按键编码 / OpenCV key code.
        """
        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._force_exit_process("q/ESC key pressed.")
            return

        if key_code in (ord("r"), ord("R")):
            self.operation_mode = "sample"
            self.roi_is_ready = False
            self.mouse_button_is_pressed = False
            self.get_logger().info("ROI sampling mode enabled. Drag mouse on target color.")

    def _force_exit_process(self, reason: str) -> None:
        """
        直接退出当前 Python 进程，避免 rclpy.shutdown 或 cv.destroyAllWindows 卡死。
        Directly exit the current Python process to avoid shutdown/destroy window freeze.

        Args:
            reason: 退出原因 / Exit reason.
        """
        self.is_shutdown_requested = True
        try:
            self.get_logger().info(f"Force exiting detect_color_node: {reason}")
        except Exception:
            pass

        # 不调用 rclpy.shutdown，不调用 cv.destroyAllWindows。
        # Do not call rclpy.shutdown or cv.destroyAllWindows here.
        os._exit(0)  # pylint: disable=protected-access

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        动态参数更新回调 / Dynamic parameter update callback.

        Args:
            parameters: 参数列表 / Parameter list.

        Returns:
            参数更新结果 / Parameter update result.
        """
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

        previous_hsv_file = self.hsv_file

        try:
            self._load_ros_parameters()

            if previous_hsv_file != self.hsv_file:
                saved_hsv_threshold_range = read_hsv_threshold_file(self.hsv_file)
                if saved_hsv_threshold_range is not None:
                    self.hsv_threshold_range = saved_hsv_threshold_range
                    self._apply_hsv_range_to_ros_parameters(saved_hsv_threshold_range)
                else:
                    self.hsv_threshold_range = self._get_hsv_range_from_parameters()
            else:
                self.hsv_threshold_range = self._get_hsv_range_from_parameters()

            self._update_debug_window()
        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception_error))

        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            父类销毁结果 / Parent destroy result.
        """
        # 注意：这里故意不调用 cv.destroyAllWindows()。
        # 部分板子/远程桌面/OpenCV HighGUI 环境下 destroyAllWindows 会卡死。
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    节点入口函数 / Node entry function.

    Args:
        args: ROS2 参数 / ROS2 arguments.
    """
    rclpy.init(args=args)

    detect_color_node = DetectColorNode()
    try:
        while rclpy.ok() and not detect_color_node.is_shutdown_requested:
            rclpy.spin_once(detect_color_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        # q/ESC 会直接 os._exit(0)，不会进入这里。
        # Ctrl+C 会进入这里，做 ROS 节点销毁；仍然不主动销毁 OpenCV 窗口。
        try:
            detect_color_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
