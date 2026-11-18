#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qr_control_node.py

中文说明:
    ROS2 Humble 二维码控制节点（No-PTZ 版）。
    本节点参考巡线/物体跟踪/人脸跟踪节点的低帧率稳定控制思路，将图像处理与底盘控制解耦：
    图像回调只缓存最新图像，图像处理定时器负责二维码识别，底盘控制定时器固定频率发布 /cmd_vel。
    摄像头固定朝向正前方，无云台控制。

English Description:
    ROS2 Humble QR code control node (No-PTZ version).
    This node follows the low-FPS stable control idea used by line/object/face tracking nodes:
    the image callback only caches the latest image, the image processing timer detects QR codes,
    and a fixed-rate control timer publishes /cmd_vel. Camera is fixed forward, no PTZ control.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from .image_conversion import image_to_bgr
from std_msgs.msg import Bool, UInt16

try:
    # pyzbar_module: pyzbar 二维码识别模块 / pyzbar QR recognition module.
    import pyzbar.pyzbar as pyzbar_module
except Exception:  # pylint: disable=broad-except
    pyzbar_module = None


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "qr_control_node"

# CONTROL_STATE_IDENTIFY: 只识别二维码但不控制底盘状态 / Identify QR without chassis control state.
CONTROL_STATE_IDENTIFY = "identify"

# CONTROL_STATE_TRACKING: 二维码控制底盘状态 / Active QR control state.
CONTROL_STATE_TRACKING = "tracking"

# CONTROL_STATE_LOST: 二维码丢失状态 / QR lost state.
CONTROL_STATE_LOST = "lost"

# CONTROL_STATE_PAUSED: 暂停控制状态 / Paused control state.
CONTROL_STATE_PAUSED = "paused"

# KEY_CODE_SPACE: 空格键编码 / Space key code.
KEY_CODE_SPACE = 32

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# MINIMUM_TIMER_RATE_HZ: 定时器最小频率 / Minimum timer frequency.
MINIMUM_TIMER_RATE_HZ = 1.0

# MINIMUM_QR_HOLD_TIMEOUT_SEC: 二维码命令保持时间下限 / Lower bound of QR command hold timeout.
MINIMUM_QR_HOLD_TIMEOUT_SEC = 0.05

# VALID_QR_COMMANDS: 合法二维码命令集合 / Valid QR command set.
VALID_QR_COMMANDS = frozenset({
    "forward",
    "back",
    "left",
    "right",
    "turnright",
    "turnleft",
    "stop",
})

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
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "qr_detector_backend",
    "qr_select_strategy",
    "qr_command_hold_timeout_sec",
    "forward_linear_speed",
    "back_linear_speed",
    "rotate_angular_speed",
    "turn_linear_speed",
    "turn_angular_speed",
    "control_rate_hz",
    "image_processing_rate_hz",
    "auto_start_control",
    "stop_when_qr_lost",
    "unknown_command_stop",
    "enable_joy_override",
    "enable_beep",
    "debug_log_interval_sec",
    "print_cmd_vel",
    "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require node restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "cmd_vel_topic",
    "joy_state_topic",
    "beep_topic",
    "window_name",
})


@dataclass
class QrDetectionResult:
    """二维码识别结果 / QR detection result."""

    # is_qr_found: 是否识别到二维码 / Whether a QR code is found.
    is_qr_found: bool

    # qr_payload: 二维码内容 / QR payload text.
    qr_payload: str = ""

    # qr_type: 二维码类型 / QR type.
    qr_type: str = ""

    # qr_rectangle: 二维码外接矩形 (x, y, width, height) / QR bounding rectangle.
    qr_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)

    # qr_polygon: 二维码多边形点 / QR polygon points.
    qr_polygon: Tuple[Tuple[int, int], ...] = tuple()

    # qr_center: 二维码中心点 / QR center point.
    qr_center: Tuple[float, float] = (0.0, 0.0)


class QrControlNode(Node):
    """二维码控制节点（No-PTZ 版） / QR control node (No-PTZ version)."""

    def __init__(self) -> None:
        """初始化二维码控制节点 / Initialize the QR control node."""
        super().__init__(DEFAULT_NODE_NAME)

        # image_lock: 图像缓存互斥锁 / Image cache mutex.
        self.image_lock = threading.Lock()
        # qr_state_lock: 二维码状态互斥锁 / QR state mutex.
        self.qr_state_lock = threading.Lock()

        # latest_image_msg: 最新图像消息 / Latest image message.
        self.latest_image_msg: Optional[Image] = None
        # latest_image_sequence_id: 最新图像序号 / Latest image sequence ID.
        self.latest_image_sequence_id = 0
        # processed_image_sequence_id: 已处理图像序号 / Processed image sequence ID.
        self.processed_image_sequence_id = 0

        # control_state: 当前控制状态 / Current control state.
        self.control_state = CONTROL_STATE_IDENTIFY
        # control_is_enabled: 是否允许二维码控制底盘 / Whether QR control is enabled.
        self.control_is_enabled = False
        # is_node_running: 节点运行标志 / Node running flag.
        self.is_node_running = True
        # joy_is_active: 手柄接管标志 / Joystick override flag.
        self.joy_is_active = False

        # qr_is_valid: 二维码有效标志 / QR valid flag.
        self.qr_is_valid = False
        # qr_payload: 最新二维码内容 / Latest QR payload.
        self.qr_payload = ""
        # qr_type: 最新二维码类型 / Latest QR type.
        self.qr_type = ""
        # qr_rectangle: 最新二维码外接矩形 / Latest QR bounding rectangle.
        self.qr_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)
        # qr_polygon: 最新二维码多边形 / Latest QR polygon.
        self.qr_polygon: Tuple[Tuple[int, int], ...] = tuple()
        # qr_center: 最新二维码中心 / Latest QR center.
        self.qr_center: Tuple[float, float] = (0.0, 0.0)
        # latest_qr_update_time_sec: 最近二维码更新时间 / Latest QR update time.
        self.latest_qr_update_time_sec = 0.0
        # unknown_command_is_active: 未知命令标志 / Unknown command flag.
        self.unknown_command_is_active = False

        # image_width: 图像宽度 / Image width.
        self.image_width = 0
        # image_height: 图像高度 / Image height.
        self.image_height = 0
        # latest_display_frame: 最新显示图像 / Latest display frame.
        self.latest_display_frame: Optional[np.ndarray] = None

        # beep_is_active: 蜂鸣器状态 / Buzzer state.
        self.beep_is_active = False
        # debug_window_initialized: 调试窗口初始化标志 / Debug window initialized flag.
        self.debug_window_initialized = False
        # opencv_qr_detector: OpenCV 二维码检测器 / OpenCV QR code detector.
        self.opencv_qr_detector = cv2.QRCodeDetector()

        # fps_last_time_sec: FPS 统计上一次时间 / Previous FPS statistics time.
        self.fps_last_time_sec = time.perf_counter()
        # fps_frame_count: FPS 统计帧数 / FPS statistics frame count.
        self.fps_frame_count = 0
        # measured_fps: 测量 FPS / Measured FPS.
        self.measured_fps = 0.0
        # last_debug_log_time_sec: 上一次调试日志时间 / Previous debug log time.
        self.last_debug_log_time_sec = 0.0
        # last_cmd_print_time_sec: 上一次速度打印时间 / Previous command print time.
        self.last_cmd_print_time_sec = 0.0
        # previous_twist_command: 上一次底盘速度命令 / Previous chassis velocity command.
        self.previous_twist_command = Twist()
        # last_reported_pyzbar_missing_time_sec: 上一次 pyzbar 缺失提示时间 / Previous pyzbar missing warning time.
        self.last_reported_pyzbar_missing_time_sec = 0.0

        self.declare_and_get_parameters()
        self.control_is_enabled = self.auto_start_control
        self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY

        # image_subscription: 图像订阅器 / Image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )
        # joy_state_subscription: 手柄状态订阅器 / Joystick state subscriber.
        self.joy_state_subscription = self.create_subscription(
            Bool,
            self.joy_state_topic,
            self.joy_state_callback,
            10,
        )
        # cmd_vel_qos: 底盘速度 QoS（与 ESP32 固件端 BEST_EFFORT 对齐）
        self.cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # cmd_vel_publisher: 底盘速度发布器 / Chassis velocity publisher.
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, self.cmd_vel_qos)
        # beep_publisher: 蜂鸣器发布器 / Buzzer publisher.
        self.beep_publisher = self.create_publisher(UInt16, self.beep_topic, 10)

        # image_processing_timer: 图像处理定时器 / Image processing timer.
        self.image_processing_timer = self.create_timer(
            1.0 / max(self.image_processing_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self.process_latest_image,
        )
        # control_timer: 底盘控制定时器 / Chassis control timer.
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self.publish_chassis_control_command,
        )

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        self.get_logger().info(
            "QR control node started. image_topic=%s, cmd_vel_topic=%s"
            % (self.image_topic, self.cmd_vel_topic)
        )

    def declare_and_get_parameters(self) -> None:
        """声明并读取参数 / Declare and read parameters."""
        # 图像和话题参数 / Image and topic parameters.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_state_topic", "/JoyState")
        self.declare_parameter("beep_topic", "/beep")
        self.declare_parameter("window_name", "qr_control")

        # 显示参数 / Display parameters.
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 15)
        self.declare_parameter("overlay_margin_x", 6)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)

        # 图像方向参数 / Image orientation parameters.
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        # 二维码识别参数 / QR detection parameters.
        self.declare_parameter("qr_detector_backend", "pyzbar")
        self.declare_parameter("qr_select_strategy", "largest")
        self.declare_parameter("qr_command_hold_timeout_sec", 1.5)

        # 二维码动作参数 / QR action parameters.
        self.declare_parameter("forward_linear_speed", 0.30)
        self.declare_parameter("back_linear_speed", -0.30)
        self.declare_parameter("rotate_angular_speed", 1.00)
        self.declare_parameter("turn_linear_speed", 0.30)
        self.declare_parameter("turn_angular_speed", 1.50)

        # 控制频率参数 / Control frequency parameters.
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)

        # 安全和状态参数 / Safety and state parameters.
        self.declare_parameter("auto_start_control", False)
        self.declare_parameter("stop_when_qr_lost", True)
        self.declare_parameter("unknown_command_stop", True)
        self.declare_parameter("enable_joy_override", True)
        self.declare_parameter("enable_beep", False)

        # 调试参数 / Debug parameters.
        self.declare_parameter("debug_log_interval_sec", 2.0)
        self.declare_parameter("print_cmd_vel", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

        self.refresh_parameters_from_ros()

    def refresh_parameters_from_ros(self) -> None:
        """从 ROS 参数服务器刷新参数 / Refresh parameters from ROS parameter server."""
        # image_topic: 图像输入话题 / Image input topic.
        self.image_topic = str(self.get_parameter("image_topic").value)
        # cmd_vel_topic: 底盘速度输出话题 / Chassis velocity output topic.
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        # joy_state_topic: 手柄状态输入话题 / Joystick state input topic.
        self.joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        # beep_topic: 蜂鸣器输出话题 / Buzzer output topic.
        self.beep_topic = str(self.get_parameter("beep_topic").value)
        # window_name: 调试窗口名称 / Debug window name.
        self.window_name = str(self.get_parameter("window_name").value)

        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = bool(self.get_parameter("overlay_show_background").value)

        self.enable_horizontal_flip = bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)

        self.qr_detector_backend = str(self.get_parameter("qr_detector_backend").value).lower()
        self.qr_select_strategy = str(self.get_parameter("qr_select_strategy").value).lower()
        self.qr_command_hold_timeout_sec = max(
            float(self.get_parameter("qr_command_hold_timeout_sec").value),
            MINIMUM_QR_HOLD_TIMEOUT_SEC,
        )

        self.forward_linear_speed = float(self.get_parameter("forward_linear_speed").value)
        self.back_linear_speed = float(self.get_parameter("back_linear_speed").value)
        self.rotate_angular_speed = abs(float(self.get_parameter("rotate_angular_speed").value))
        self.turn_linear_speed = float(self.get_parameter("turn_linear_speed").value)
        self.turn_angular_speed = abs(float(self.get_parameter("turn_angular_speed").value))

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self.image_processing_rate_hz = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)

        self.auto_start_control = bool(self.get_parameter("auto_start_control").value)
        self.stop_when_qr_lost = bool(self.get_parameter("stop_when_qr_lost").value)
        self.unknown_command_stop = bool(self.get_parameter("unknown_command_stop").value)
        self.enable_joy_override = bool(self.get_parameter("enable_joy_override").value)
        self.enable_beep = bool(self.get_parameter("enable_beep").value)

        self.debug_log_interval_sec = float(self.get_parameter("debug_log_interval_sec").value)
        self.print_cmd_vel = bool(self.get_parameter("print_cmd_vel").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
        self.enable_rqt_dynamic_parameters = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

    def image_callback(self, image_msg: Image) -> None:
        """
        缓存最新图像 / Cache the latest image.

        Args:
            image_msg: 图像消息 / Image message.
        """
        with self.image_lock:
            self.latest_image_msg = image_msg
            self.latest_image_sequence_id += 1

    def joy_state_callback(self, joy_msg: Bool) -> None:
        """
        更新手柄接管状态 / Update joystick override state.

        Args:
            joy_msg: 手柄状态消息 / Joystick state message.
        """
        self.joy_is_active = bool(joy_msg.data)
        if self.enable_joy_override and self.joy_is_active:
            self.publish_zero_twist()

    def process_latest_image(self) -> None:
        """处理最新图像并更新二维码状态 / Process latest image and update QR state."""
        with self.image_lock:
            if self.latest_image_msg is None:
                return
            # local_image_msg: 当前要处理的图像消息 / Local image message to process.
            local_image_msg = self.latest_image_msg
            # local_sequence_id: 当前图像序号 / Local image sequence ID.
            local_sequence_id = self.latest_image_sequence_id

        if local_sequence_id == self.processed_image_sequence_id:
            self.update_debug_window_if_needed()
            return
        self.processed_image_sequence_id = local_sequence_id

        # decoded_frame: 解码后的 BGR 图像 / Decoded BGR image.
        decoded_frame = self.decode_image(local_image_msg)
        if decoded_frame is None:
            return

        # processed_frame: 方向修正并缩放后的图像 / Orientation-corrected and resized frame.
        processed_frame = self.prepare_image_frame(decoded_frame)
        self.image_height, self.image_width = processed_frame.shape[:2]

        # qr_result: 二维码识别结果 / QR detection result.
        qr_result = self.detect_qr_code(processed_frame)
        self.update_qr_state(qr_result)

        # display_frame: 调试显示图像 / Debug display frame.
        display_frame = processed_frame.copy()
        self.draw_qr_detection(display_frame, qr_result)
        self.draw_overlay_text(display_frame)
        self.latest_display_frame = display_frame

        self.update_fps_statistics()
        self.update_debug_window_if_needed()

    def decode_image(self, image_msg: Image) -> Optional[np.ndarray]:
        """
        转换图像 / Convert image.

        Args:
            image_msg: 图像消息 / Image message.

        Returns:
            Optional[np.ndarray]: 解码后的 BGR 图像 / Decoded BGR frame.
        """
        try:
            return image_to_bgr(image_msg)
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().warn("Image decode failed: %s" % exception)
            return None

    def prepare_image_frame(self, image_frame: np.ndarray) -> np.ndarray:
        """
        准备图像帧 / Prepare image frame.

        Args:
            image_frame: 原始图像 / Raw image.

        Returns:
            np.ndarray: 处理后的图像 / Processed image.
        """
        # resized_frame: 统一尺寸图像 / Resized image.
        resized_frame = cv2.resize(image_frame, (640, 480))
        if self.enable_horizontal_flip:
            resized_frame = cv2.flip(resized_frame, 1)
        if self.enable_vertical_flip:
            resized_frame = cv2.flip(resized_frame, 0)

        if self.rotate_image_code == "90_clockwise":
            resized_frame = cv2.rotate(resized_frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code == "90_counterclockwise":
            resized_frame = cv2.rotate(resized_frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code == "180":
            resized_frame = cv2.rotate(resized_frame, cv2.ROTATE_180)
        return resized_frame

    def detect_qr_code(self, image_frame: np.ndarray) -> QrDetectionResult:
        """
        检测二维码 / Detect QR code.

        Args:
            image_frame: BGR 图像 / BGR frame.

        Returns:
            QrDetectionResult: 二维码识别结果 / QR detection result.
        """
        # backend_name: 二维码识别后端名称 / QR detector backend name.
        backend_name = self.qr_detector_backend.lower()
        if backend_name not in {"pyzbar", "opencv", "auto"}:
            backend_name = "pyzbar"

        if backend_name in {"pyzbar", "auto"}:
            # pyzbar_result: pyzbar 识别结果 / pyzbar detection result.
            pyzbar_result = self.detect_qr_code_with_pyzbar(image_frame)
            if pyzbar_result.is_qr_found or backend_name == "pyzbar":
                return pyzbar_result

        # opencv_result: OpenCV 识别结果 / OpenCV detection result.
        return self.detect_qr_code_with_opencv(image_frame)

    def detect_qr_code_with_pyzbar(self, image_frame: np.ndarray) -> QrDetectionResult:
        """
        使用 pyzbar 检测二维码 / Detect QR code using pyzbar.

        Args:
            image_frame: BGR 图像 / BGR frame.

        Returns:
            QrDetectionResult: 二维码识别结果 / QR detection result.
        """
        if pyzbar_module is None:
            self.report_pyzbar_missing_if_needed()
            return QrDetectionResult(False)

        # gray_frame: 灰度图像 / Gray image.
        gray_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
        # decoded_barcodes: pyzbar 解码结果 / pyzbar decoded results.
        decoded_barcodes = pyzbar_module.decode(gray_frame)
        if not decoded_barcodes:
            return QrDetectionResult(False)

        # candidates: 候选二维码结果 / Candidate QR results.
        candidates: List[QrDetectionResult] = []
        for decoded_barcode in decoded_barcodes:
            # payload_text: 二维码内容 / QR payload text.
            payload_text = decoded_barcode.data.decode("utf-8", errors="ignore").strip()
            # rect_x, rect_y, rect_w, rect_h: 二维码外接矩形 / QR bounding rectangle.
            rect_x, rect_y, rect_w, rect_h = decoded_barcode.rect
            # polygon_points: 二维码多边形点 / QR polygon points.
            polygon_points = tuple((int(point.x), int(point.y)) for point in decoded_barcode.polygon)
            if not polygon_points:
                polygon_points = (
                    (int(rect_x), int(rect_y)),
                    (int(rect_x + rect_w), int(rect_y)),
                    (int(rect_x + rect_w), int(rect_y + rect_h)),
                    (int(rect_x), int(rect_y + rect_h)),
                )
            candidates.append(QrDetectionResult(
                True,
                payload_text,
                str(decoded_barcode.type),
                (int(rect_x), int(rect_y), int(rect_w), int(rect_h)),
                polygon_points,
                (float(rect_x + rect_w * 0.5), float(rect_y + rect_h * 0.5)),
            ))
        return self.select_qr_candidate(candidates)

    def detect_qr_code_with_opencv(self, image_frame: np.ndarray) -> QrDetectionResult:
        """
        使用 OpenCV QRCodeDetector 检测二维码 / Detect QR code using OpenCV QRCodeDetector.

        Args:
            image_frame: BGR 图像 / BGR frame.

        Returns:
            QrDetectionResult: 二维码识别结果 / QR detection result.
        """
        try:
            # decoded_info: 多二维码解码结果 / Multi QR decoded info.
            decoded_info: Sequence[str]
            # points: 多二维码角点 / Multi QR corner points.
            points: Optional[np.ndarray]
            detection_ok, decoded_info, points, _ = self.opencv_qr_detector.detectAndDecodeMulti(image_frame)
            if detection_ok and points is not None and decoded_info:
                # candidates: 候选二维码结果 / Candidate QR results.
                candidates: List[QrDetectionResult] = []
                for payload_text, point_array in zip(decoded_info, points):
                    if not payload_text:
                        continue
                    candidates.append(self.make_qr_result_from_points(payload_text.strip(), point_array, "QRCODE"))
                if candidates:
                    return self.select_qr_candidate(candidates)

            # payload_text: 单二维码内容 / Single QR payload.
            # point_array: 单二维码角点 / Single QR corner points.
            payload_text, point_array, _ = self.opencv_qr_detector.detectAndDecode(image_frame)
            if payload_text and point_array is not None:
                return self.make_qr_result_from_points(payload_text.strip(), point_array, "QRCODE")
        except Exception as exception:  # pylint: disable=broad-except
            self.throttled_debug_log("OpenCV QR detection failed: %s" % exception)
        return QrDetectionResult(False)

    def make_qr_result_from_points(self, payload_text: str, point_array: np.ndarray, qr_type: str) -> QrDetectionResult:
        """
        根据角点生成二维码结果 / Build QR result from corner points.

        Args:
            payload_text: 二维码内容 / QR payload text.
            point_array: 二维码角点 / QR corner points.
            qr_type: 二维码类型 / QR type.

        Returns:
            QrDetectionResult: 二维码识别结果 / QR detection result.
        """
        # reshaped_points: 规整后的点数组 / Reshaped point array.
        reshaped_points = np.array(point_array, dtype=np.float32).reshape(-1, 2)
        # polygon_points: 二维码多边形点 / QR polygon points.
        polygon_points = tuple((int(point[0]), int(point[1])) for point in reshaped_points)
        # x_coord, y_coord, width, height: 外接矩形 / Bounding rectangle.
        x_coord, y_coord, width, height = cv2.boundingRect(np.array(polygon_points, dtype=np.int32))
        # center_x, center_y: 二维码中心 / QR center.
        center_x = float(np.mean(reshaped_points[:, 0]))
        center_y = float(np.mean(reshaped_points[:, 1]))
        return QrDetectionResult(
            True,
            payload_text,
            qr_type,
            (int(x_coord), int(y_coord), int(width), int(height)),
            polygon_points,
            (center_x, center_y),
        )

    def select_qr_candidate(self, candidates: List[QrDetectionResult]) -> QrDetectionResult:
        """
        从多个二维码中选择一个目标 / Select one QR target from candidates.

        Args:
            candidates: 候选二维码列表 / Candidate QR list.

        Returns:
            QrDetectionResult: 选中的二维码 / Selected QR result.
        """
        if not candidates:
            return QrDetectionResult(False)
        if self.qr_select_strategy == "nearest_center" and self.image_width > 0 and self.image_height > 0:
            # image_center_x: 图像中心 x / Image center x.
            image_center_x = self.image_width * 0.5
            # image_center_y: 图像中心 y / Image center y.
            image_center_y = self.image_height * 0.5
            return min(
                candidates,
                key=lambda result: math.hypot(result.qr_center[0] - image_center_x, result.qr_center[1] - image_center_y),
            )
        if self.qr_select_strategy == "first":
            return candidates[0]
        return max(candidates, key=lambda result: result.qr_rectangle[2] * result.qr_rectangle[3])

    def update_qr_state(self, qr_result: QrDetectionResult) -> None:
        """
        更新二维码状态 / Update QR state.

        Args:
            qr_result: 二维码识别结果 / QR detection result.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        with self.qr_state_lock:
            if qr_result.is_qr_found:
                self.qr_is_valid = True
                self.qr_payload = qr_result.qr_payload
                self.qr_type = qr_result.qr_type
                self.qr_rectangle = qr_result.qr_rectangle
                self.qr_polygon = qr_result.qr_polygon
                self.qr_center = qr_result.qr_center
                self.latest_qr_update_time_sec = current_time_sec
                self.unknown_command_is_active = self.normalize_qr_command(qr_result.qr_payload) not in VALID_QR_COMMANDS
                self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            else:
                # qr_age_sec: 二维码丢失时间 / QR lost age.
                qr_age_sec = current_time_sec - self.latest_qr_update_time_sec
                if qr_age_sec > self.qr_command_hold_timeout_sec:
                    self.qr_is_valid = False
                    self.qr_payload = ""
                    self.qr_type = ""
                    self.qr_rectangle = (0, 0, 0, 0)
                    self.qr_polygon = tuple()
                    self.qr_center = (0.0, 0.0)
                    self.unknown_command_is_active = False
                    if self.control_is_enabled:
                        self.control_state = CONTROL_STATE_LOST

    @staticmethod
    def normalize_qr_command(qr_payload: str) -> str:
        """
        归一化二维码命令 / Normalize QR command.

        Args:
            qr_payload: 原始二维码内容 / Raw QR payload.

        Returns:
            str: 归一化命令 / Normalized command.
        """
        return qr_payload.strip().lower().replace(" ", "").replace("_", "-").replace("-", "")

    def publish_chassis_control_command(self) -> None:
        """固定频率发布底盘控制命令 / Publish chassis control command at a fixed rate."""
        if not self.is_node_running:
            return

        if self.enable_joy_override and self.joy_is_active:
            self.publish_zero_twist()
            self.publish_beep(False)
            return

        if not self.control_is_enabled:
            self.publish_zero_twist()
            self.publish_beep(False)
            return

        with self.qr_state_lock:
            # local_qr_is_valid: 本地二维码有效标志 / Local QR valid flag.
            local_qr_is_valid = self.qr_is_valid
            # local_qr_payload: 本地二维码内容 / Local QR payload.
            local_qr_payload = self.qr_payload
            # local_latest_qr_update_time_sec: 本地二维码更新时间 / Local QR update time.
            local_latest_qr_update_time_sec = self.latest_qr_update_time_sec

        # qr_age_sec: 距离上一次识别二维码的时间 / Age since latest QR update.
        qr_age_sec = time.perf_counter() - local_latest_qr_update_time_sec
        if (not local_qr_is_valid or qr_age_sec > self.qr_command_hold_timeout_sec) and self.stop_when_qr_lost:
            self.control_state = CONTROL_STATE_LOST
            self.publish_zero_twist()
            self.publish_beep(False)
            return

        # normalized_command: 归一化二维码命令 / Normalized QR command.
        normalized_command = self.normalize_qr_command(local_qr_payload)
        # twist_command: 底盘速度命令 / Chassis velocity command.
        twist_command = self.build_twist_from_qr_command(normalized_command)
        if twist_command is None:
            self.unknown_command_is_active = True
            if self.unknown_command_stop:
                self.publish_zero_twist()
            self.publish_beep(True)
            self.throttled_debug_log("Unknown QR command: %s" % local_qr_payload)
            return

        self.unknown_command_is_active = False
        self.publish_beep(False)
        self.cmd_vel_publisher.publish(twist_command)
        self.previous_twist_command = twist_command
        self.print_cmd_vel_if_needed(twist_command, normalized_command)

    def build_twist_from_qr_command(self, qr_command: str) -> Optional[Twist]:
        """
        根据二维码命令生成底盘速度 / Build chassis velocity from QR command.

        Args:
            qr_command: 归一化二维码命令 / Normalized QR command.

        Returns:
            Optional[Twist]: 速度命令；未知命令返回 None / Velocity command or None for unknown command.
        """
        # twist_command: 底盘速度命令 / Chassis velocity command.
        twist_command = Twist()
        if qr_command == "forward":
            twist_command.linear.x = self.forward_linear_speed
        elif qr_command == "back":
            twist_command.linear.x = self.back_linear_speed
        elif qr_command == "left":
            twist_command.angular.z = self.rotate_angular_speed
        elif qr_command == "right":
            twist_command.angular.z = -self.rotate_angular_speed
        elif qr_command == "turnleft":
            twist_command.linear.x = self.turn_linear_speed
            twist_command.angular.z = self.turn_angular_speed
        elif qr_command == "turnright":
            twist_command.linear.x = self.turn_linear_speed
            twist_command.angular.z = -self.turn_angular_speed
        elif qr_command == "stop":
            pass
        else:
            return None
        return twist_command

    def publish_zero_twist(self) -> None:
        """发布零速度 / Publish zero velocity."""
        # zero_twist_msg: 零速度消息 / Zero velocity message.
        zero_twist_msg = Twist()
        self.cmd_vel_publisher.publish(zero_twist_msg)
        self.previous_twist_command = zero_twist_msg

    def publish_beep(self, enable_beep: bool) -> None:
        """
        发布蜂鸣器状态 / Publish buzzer state.

        Args:
            enable_beep: 是否打开蜂鸣器 / Whether to enable buzzer.
        """
        if not self.enable_beep:
            enable_beep = False
        if self.beep_is_active == enable_beep:
            return
        # beep_msg: 蜂鸣器消息 / Buzzer message.
        beep_msg = UInt16()
        beep_msg.data = 1 if enable_beep else 0
        self.beep_publisher.publish(beep_msg)
        self.beep_is_active = enable_beep

    def initialize_debug_window_if_needed(self) -> None:
        """按需初始化调试窗口 / Initialize debug window if needed."""
        if not self.show_debug_window:
            return
        if self.debug_window_initialized:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, int(self.window_width), int(self.window_height))
        self.debug_window_initialized = True

    def update_debug_window_if_needed(self) -> None:
        """按需刷新调试窗口 / Update debug window if needed."""
        if not self.show_debug_window:
            if self.debug_window_initialized:
                cv2.destroyWindow(self.window_name)
                self.debug_window_initialized = False
            return
        if self.latest_display_frame is None:
            return
        self.initialize_debug_window_if_needed()
        cv2.imshow(self.window_name, self.latest_display_frame)
        # key_code: 键盘输入 / Keyboard input.
        key_code = cv2.waitKey(1) & 0xFF
        self.handle_keyboard_input(key_code)

    def handle_keyboard_input(self, key_code: int) -> None:
        """
        处理键盘输入 / Handle keyboard input.

        Args:
            key_code: 键盘编码 / Keyboard code.
        """
        if key_code in (255, -1):
            return
        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self.cancel_node()
        elif key_code in (ord("r"), ord("R")):
            self.reset_qr_control_state()
        elif key_code in (ord("i"), ord("I")):
            self.control_is_enabled = False
            self.control_state = CONTROL_STATE_IDENTIFY
            self.publish_zero_twist()
        elif key_code == KEY_CODE_SPACE:
            if self.control_is_enabled:
                self.control_is_enabled = False
                self.control_state = CONTROL_STATE_PAUSED
                self.publish_zero_twist()
            else:
                self.control_is_enabled = True
                self.control_state = CONTROL_STATE_TRACKING if self.qr_is_valid else CONTROL_STATE_LOST

    def reset_qr_control_state(self) -> None:
        """重置二维码控制状态 / Reset QR control state."""
        with self.qr_state_lock:
            self.qr_is_valid = False
            self.qr_payload = ""
            self.qr_type = ""
            self.qr_rectangle = (0, 0, 0, 0)
            self.qr_polygon = tuple()
            self.qr_center = (0.0, 0.0)
            self.latest_qr_update_time_sec = 0.0
            self.unknown_command_is_active = False
        self.control_is_enabled = False
        self.control_state = CONTROL_STATE_IDENTIFY
        self.publish_zero_twist()
        self.publish_beep(False)
        self.get_logger().info("QR control state reset. Press SPACE to enable QR control.")

    def cancel_node(self) -> None:
        """取消节点运行 / Cancel node running."""
        self.is_node_running = False
        self.publish_zero_twist()
        self.publish_beep(False)
        self.get_logger().info("QR control node requested to stop from keyboard.")
        # 参照 ptz-tracking-rules SKILL: q/ESC 使用 os._exit(0) 立即退出
        os._exit(0)

    def draw_qr_detection(self, display_frame: np.ndarray, qr_result: QrDetectionResult) -> None:
        """
        绘制二维码识别结果 / Draw QR detection result.

        Args:
            display_frame: 显示图像 / Display frame.
            qr_result: 二维码识别结果 / QR detection result.
        """
        # image_center_x: 图像中心 x / Image center x.
        image_center_x = int(display_frame.shape[1] * 0.5)
        cv2.line(display_frame, (image_center_x, 0), (image_center_x, display_frame.shape[0]), (255, 255, 0), 1)

        if not qr_result.is_qr_found:
            return

        # x_coord, y_coord, width, height: 外接矩形 / Bounding rectangle.
        x_coord, y_coord, width, height = qr_result.qr_rectangle
        cv2.rectangle(display_frame, (x_coord, y_coord), (x_coord + width, y_coord + height), (0, 255, 0), 2)
        cv2.circle(display_frame, (int(qr_result.qr_center[0]), int(qr_result.qr_center[1])), 5, (0, 0, 255), -1)
        if qr_result.qr_polygon:
            # polygon_array: 多边形点数组 / Polygon point array.
            polygon_array = np.array(qr_result.qr_polygon, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(display_frame, [polygon_array], True, (0, 180, 255), 2)
        cv2.putText(
            display_frame,
            qr_result.qr_payload,
            (x_coord, max(20, y_coord - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def draw_overlay_text(self, display_frame: np.ndarray) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            display_frame: 显示图像 / Display frame.
        """
        with self.qr_state_lock:
            # local_payload: 本地二维码内容 / Local QR payload.
            local_payload = self.qr_payload if self.qr_payload else "None"
            # qr_age_sec: 二维码更新时间间隔 / QR update age.
            qr_age_sec = time.perf_counter() - self.latest_qr_update_time_sec if self.latest_qr_update_time_sec > 0.0 else 999.0
            # unknown_text: 未知命令文本 / Unknown command text.
            unknown_text = "YES" if self.unknown_command_is_active else "NO"

        # overlay_lines: 叠加文字行 / Overlay text lines.
        overlay_lines = [
            "State: %s | FPS: %.1f" % (self.control_state, self.measured_fps),
            "QR: %s | age: %.2fs | unknown: %s" % (local_payload, qr_age_sec, unknown_text),
            "cmd_vel: x=%.2f z=%.2f" % (self.previous_twist_command.linear.x, self.previous_twist_command.angular.z),
            "backend: %s" % self.qr_detector_backend,
            "Keys: SPACE control/pause, i identify, r reset, q quit",
        ]

        if self.overlay_show_background:
            # background_height: 背景高度 / Background height.
            background_height = self.overlay_line_height * len(overlay_lines) + 8
            cv2.rectangle(
                display_frame,
                (0, 0),
                (min(display_frame.shape[1], 720), background_height),
                (0, 0, 0),
                -1,
            )

        for line_index, line_text in enumerate(overlay_lines):
            # text_origin: 文字起点 / Text origin.
            text_origin = (
                self.overlay_margin_x,
                self.overlay_margin_y + line_index * self.overlay_line_height,
            )
            cv2.putText(
                display_frame,
                line_text,
                text_origin,
                cv2.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    def update_fps_statistics(self) -> None:
        """更新 FPS 统计 / Update FPS statistics."""
        self.fps_frame_count += 1
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        # elapsed_time_sec: 经过时间 / Elapsed time.
        elapsed_time_sec = current_time_sec - self.fps_last_time_sec
        if elapsed_time_sec >= 1.0:
            self.measured_fps = self.fps_frame_count / elapsed_time_sec
            self.fps_frame_count = 0
            self.fps_last_time_sec = current_time_sec

    def throttled_debug_log(self, log_text: str) -> None:
        """
        节流输出调试日志 / Throttled debug log output.

        Args:
            log_text: 日志文本 / Log text.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_debug_log_time_sec >= self.debug_log_interval_sec:
            self.get_logger().info(log_text)
            self.last_debug_log_time_sec = current_time_sec

    def report_pyzbar_missing_if_needed(self) -> None:
        """按需提示 pyzbar 不可用 / Report unavailable pyzbar if needed."""
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_reported_pyzbar_missing_time_sec >= self.debug_log_interval_sec:
            self.get_logger().warn("pyzbar is unavailable; OpenCV QR detector is recommended or used in auto mode.")
            self.last_reported_pyzbar_missing_time_sec = current_time_sec

    def print_cmd_vel_if_needed(self, twist_command: Twist, qr_command: str) -> None:
        """
        按需打印速度命令 / Print velocity command if needed.

        Args:
            twist_command: 速度命令 / Velocity command.
            qr_command: 二维码命令 / QR command.
        """
        if not self.print_cmd_vel:
            return
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_cmd_print_time_sec < self.print_interval_sec:
            return
        self.get_logger().info(
            "QR command=%s cmd_vel linear.x=%.3f angular.z=%.3f"
            % (qr_command, twist_command.linear.x, twist_command.angular.z)
        )
        self.last_cmd_print_time_sec = current_time_sec

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        rqt/ros2 param 动态参数回调 / Dynamic parameter callback for rqt/ros2 param.

        Args:
            parameters: 参数列表 / Parameter list.

        Returns:
            SetParametersResult: 参数设置结果 / Parameter setting result.
        """
        if not self.enable_rqt_dynamic_parameters:
            return SetParametersResult(successful=False, reason="Dynamic parameters are disabled.")

        # should_recreate_control_timer: 是否重建控制定时器 / Whether to recreate control timer.
        should_recreate_control_timer = False
        # should_recreate_image_timer: 是否重建图像定时器 / Whether to recreate image timer.
        should_recreate_image_timer = False

        try:
            for parameter in parameters:
                parameter_name = parameter.name
                if parameter_name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(
                        successful=False,
                        reason="Parameter '%s' requires node restart." % parameter_name,
                    )
                if parameter_name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if parameter_name == "control_rate_hz":
                    should_recreate_control_timer = True
                elif parameter_name == "image_processing_rate_hz":
                    should_recreate_image_timer = True

            self.apply_dynamic_parameters_to_attributes(parameters)

            if should_recreate_control_timer:
                self.destroy_timer(self.control_timer)
                self.control_timer = self.create_timer(
                    1.0 / max(self.control_rate_hz, MINIMUM_TIMER_RATE_HZ),
                    self.publish_chassis_control_command,
                )
            if should_recreate_image_timer:
                self.destroy_timer(self.image_processing_timer)
                self.image_processing_timer = self.create_timer(
                    1.0 / max(self.image_processing_rate_hz, MINIMUM_TIMER_RATE_HZ),
                    self.process_latest_image,
                )
            if any(parameter.name == "show_debug_window" for parameter in parameters):
                if not self.show_debug_window and self.debug_window_initialized:
                    cv2.destroyWindow(self.window_name)
                    self.debug_window_initialized = False

            return SetParametersResult(successful=True)
        except Exception as exception:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception))

    def apply_dynamic_parameters_to_attributes(self, parameters: List[Parameter]) -> None:
        """
        将动态参数应用到属性 / Apply dynamic parameters to attributes.

        Args:
            parameters: 参数列表 / Parameter list.
        """
        for parameter in parameters:
            parameter_name = parameter.name
            parameter_value = parameter.value
            if parameter_name == "show_debug_window":
                self.show_debug_window = bool(parameter_value)
            elif parameter_name == "window_width":
                self.window_width = int(parameter_value)
            elif parameter_name == "window_height":
                self.window_height = int(parameter_value)
            elif parameter_name == "overlay_font_scale":
                self.overlay_font_scale = float(parameter_value)
            elif parameter_name == "overlay_line_height":
                self.overlay_line_height = int(parameter_value)
            elif parameter_name == "overlay_margin_x":
                self.overlay_margin_x = int(parameter_value)
            elif parameter_name == "overlay_margin_y":
                self.overlay_margin_y = int(parameter_value)
            elif parameter_name == "overlay_show_background":
                self.overlay_show_background = bool(parameter_value)
            elif parameter_name == "enable_horizontal_flip":
                self.enable_horizontal_flip = bool(parameter_value)
            elif parameter_name == "enable_vertical_flip":
                self.enable_vertical_flip = bool(parameter_value)
            elif parameter_name == "rotate_image_code":
                self.rotate_image_code = str(parameter_value)
            elif parameter_name == "qr_detector_backend":
                self.qr_detector_backend = str(parameter_value).lower()
            elif parameter_name == "qr_select_strategy":
                self.qr_select_strategy = str(parameter_value).lower()
            elif parameter_name == "qr_command_hold_timeout_sec":
                self.qr_command_hold_timeout_sec = max(float(parameter_value), MINIMUM_QR_HOLD_TIMEOUT_SEC)
            elif parameter_name in {"forward_linear_speed", "back_linear_speed", "turn_linear_speed"}:
                setattr(self, parameter_name, float(parameter_value))
            elif parameter_name in {"rotate_angular_speed", "turn_angular_speed"}:
                setattr(self, parameter_name, abs(float(parameter_value)))
            elif parameter_name == "control_rate_hz":
                self.control_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name == "image_processing_rate_hz":
                self.image_processing_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name in {"auto_start_control", "stop_when_qr_lost", "unknown_command_stop", "enable_joy_override", "enable_beep", "print_cmd_vel", "enable_rqt_dynamic_parameters"}:
                setattr(self, parameter_name, bool(parameter_value))
                if parameter_name == "auto_start_control":
                    self.control_is_enabled = bool(parameter_value)
                    self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            elif parameter_name in {"debug_log_interval_sec", "print_interval_sec"}:
                setattr(self, parameter_name, float(parameter_value))

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            bool: 销毁结果 / Destroy result.
        """
        self.is_node_running = False
        self.publish_zero_twist()
        self.publish_beep(False)
        if self.show_debug_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    节点入口函数 / Node entry point.

    Args:
        args: ROS 参数 / ROS arguments.
    """
    rclpy.init(args=args)
    # qr_control_node: 二维码控制节点 / QR control node.
    qr_control_node = QrControlNode()
    try:
        rclpy.spin(qr_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            qr_control_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
