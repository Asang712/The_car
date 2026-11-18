#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_qr.py

中文说明:
    ROS2 Humble 二维码识别/检测节点。
    该版本已经去掉底盘 cmd_vel 控制、云台 PTZ、蜂鸣器、手柄接管、二维码动作控制等功能，
    只保留原始图像订阅、二维码检测、画框显示和识别日志。
    默认使用 pyzbar 识别二维码，避免 OpenCV 未链接 QUIRC 时无法解码。
    文件名重命名为 detect_qr.py。

    本版本沿用 detect_color 已验证的退出方式：
    1. q/ESC 不调用 rclpy.shutdown()
    2. q/ESC 不调用 cv2.destroyAllWindows()
    3. q/ESC 直接 os._exit(0) 结束当前节点进程
    4. main 使用 spin_once 循环，不使用阻塞式 rclpy.spin()

English Description:
    ROS2 Humble QR code detection node.
    This version removes chassis cmd_vel control, PTZ control, buzzer, joystick override,
    and QR command actions. It only keeps raw image subscription, QR detection,
    visualization, and detection logging.

    This version follows the verified exit behavior used by detect_color:
    1. q/ESC does not call rclpy.shutdown()
    2. q/ESC does not call cv2.destroyAllWindows()
    3. q/ESC directly exits the node process with os._exit(0)
    4. main uses spin_once loop instead of blocking rclpy.spin()
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

try:
    # pyzbar_module: pyzbar 二维码识别模块 / pyzbar QR recognition module.
    import pyzbar.pyzbar as pyzbar_module
except Exception:  # pylint: disable=broad-except
    pyzbar_module = None


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "detect_qr_node"

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# DYNAMIC_PARAMETER_NAMES: 支持运行时动态修改的参数 / Runtime dynamic parameters.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "window_width",
    "window_height",
    "image_width",
    "image_height",
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
    "print_detection_log",
    "detection_log_interval_sec",
    "debug_log_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
    "enable_rqt_dynamic_parameters",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "window_name",
})


@dataclass
class QrCodeData:
    """单个二维码数据 / Single QR code data."""

    # payload: 二维码内容 / QR payload text.
    payload: str

    # qr_type: 二维码类型 / QR code type.
    qr_type: str

    # rectangle: 外接矩形 (x, y, width, height) / Bounding rectangle.
    rectangle: Tuple[int, int, int, int]

    # polygon: 多边形角点 / Polygon points.
    polygon: Tuple[Tuple[int, int], ...]

    # center: 中心点 / Center point.
    center: Tuple[float, float]


@dataclass
class QrDetectionResult:
    """二维码识别结果 / QR detection result."""

    # is_qr_found: 是否识别到二维码 / Whether at least one QR code is found.
    is_qr_found: bool

    # qr_count: 检测到的二维码数量 / Number of detected QR codes.
    qr_count: int = 0

    # main_qr: 主二维码 / Main selected QR code.
    main_qr: Optional[QrCodeData] = None

    # all_qr_codes: 全部二维码 / All QR codes.
    all_qr_codes: Tuple[QrCodeData, ...] = ()


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


class DetectQrNode(Node):
    """ROS2 二维码识别节点 / ROS2 QR detection node."""

    def __init__(self) -> None:
        """初始化节点 / Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # is_shutdown_requested: 是否请求关闭节点 / Whether shutdown is requested.
        self.is_shutdown_requested = False
        self.image_bridge = CvBridge()

        # opencv_qr_detector: OpenCV 二维码检测器 / OpenCV QR code detector.
        self.opencv_qr_detector = cv2.QRCodeDetector()

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

        # last_debug_log_time_sec: 上一次调试日志时间 / Last debug log time.
        self.last_debug_log_time_sec = 0.0

        # latest_detection_result: 最近一次二维码检测结果 / Latest QR detection result.
        self.latest_detection_result = QrDetectionResult(False)

        # keyboard_poll_timer: OpenCV 键盘轮询定时器 / OpenCV keyboard polling timer.
        self.keyboard_poll_timer = None

        self._declare_ros_parameters()
        self._load_ros_parameters()
        self._initialize_debug_window()

        # image_subscription: 图像订阅器 / Image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        if self.show_debug_window:
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)

        self.get_logger().info(
            "Detect QR node started. "
            "Only QR detection is enabled; chassis/PTZ/beep control has been removed. "
            "Press q or ESC to quit. Default backend is pyzbar to avoid OpenCV QUIRC missing issue. "
            f"image_topic={self.image_topic_name}, backend={self.qr_detector_backend}"
        )

    def _declare_ros_parameters(self) -> None:
        """声明 ROS 参数 / Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("window_name", "detect_qr_node")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 14)
        self.declare_parameter("overlay_margin_x", 6)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)

        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        self.declare_parameter("qr_detector_backend", "pyzbar")
        self.declare_parameter("qr_select_strategy", "largest")

        self.declare_parameter("print_detection_log", True)
        self.declare_parameter("detection_log_interval_sec", 1.0)
        self.declare_parameter("debug_log_interval_sec", 2.0)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_ros_parameters(self) -> None:
        """读取 ROS 参数 / Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.show_debug_window = convert_value_to_boolean(
            self.get_parameter("show_debug_window").value,
            True,
        )
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = convert_value_to_boolean(
            self.get_parameter("overlay_show_background").value,
            False,
        )

        self.enable_horizontal_flip = convert_value_to_boolean(
            self.get_parameter("enable_horizontal_flip").value,
            False,
        )
        self.enable_vertical_flip = convert_value_to_boolean(
            self.get_parameter("enable_vertical_flip").value,
            False,
        )
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value).lower()

        self.qr_detector_backend = str(self.get_parameter("qr_detector_backend").value).lower()
        if self.qr_detector_backend not in {"opencv", "pyzbar", "auto"}:
            self.qr_detector_backend = "pyzbar"

        self.qr_select_strategy = str(self.get_parameter("qr_select_strategy").value).lower()
        if self.qr_select_strategy not in {"largest", "nearest_center", "first"}:
            self.qr_select_strategy = "largest"

        self.print_detection_log = convert_value_to_boolean(
            self.get_parameter("print_detection_log").value,
            True,
        )
        self.detection_log_interval_sec = max(
            float(self.get_parameter("detection_log_interval_sec").value),
            0.05,
        )
        self.debug_log_interval_sec = max(
            float(self.get_parameter("debug_log_interval_sec").value),
            0.1,
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
        if not self.show_debug_window:
            return

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

    def _update_debug_window(self) -> None:
        """更新 OpenCV 窗口 / Update OpenCV window."""
        if not self.show_debug_window:
            # 部分板子上 destroyWindow/destroyAllWindows 可能卡死，所以这里不主动销毁窗口。
            # On some boards destroyWindow/destroyAllWindows may freeze, so do not destroy actively.
            return

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

        if self.keyboard_poll_timer is None:
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)

    def _keyboard_poll_timer_callback(self) -> None:
        """独立轮询 OpenCV 按键，避免低帧率时 q/ESC 无响应 / Poll OpenCV keys independently."""
        if self.is_shutdown_requested or not self.show_debug_window:
            return

        try:
            key_code = cv2.waitKey(1) & 0xFF
            self._handle_key_input(key_code)
        except Exception:
            pass

    def image_callback(self, image_msg: Image) -> None:
        """
        接收、解码并识别二维码 / Receive, decode, and detect QR codes.

        Args:
            image_msg: 原始图像消息 / Raw image message.
        """
        if self.is_shutdown_requested:
            return

        try:
            image_frame = self.image_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            self._update_image_timing_statistics(image_msg)
            image_frame = self._apply_image_orientation(image_frame)

            if self.image_width > 0 and self.image_height > 0:
                image_frame = cv2.resize(
                    image_frame,
                    (int(self.image_width), int(self.image_height)),
                    interpolation=cv2.INTER_AREA,
                )

            detection_result = self.detect_qr_codes(image_frame)
            self.latest_detection_result = detection_result

            self._print_detection_log_if_needed(detection_result)

            if self.show_debug_window:
                self._draw_and_show_debug_image(image_frame, detection_result)

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def _apply_image_orientation(self, image_frame: np.ndarray) -> np.ndarray:
        """
        应用图像方向修正 / Apply image orientation correction.

        Args:
            image_frame: 输入图像 / Input image.

        Returns:
            方向修正后的图像 / Orientation-corrected image.
        """
        corrected_image_frame = image_frame

        if self.enable_horizontal_flip:
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
        """
        更新图像帧率和延迟统计 / Update image FPS and delay statistics.

        Args:
            image_msg: 原始图像消息 / Raw image message.
        """
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

    def detect_qr_codes(self, image_frame: np.ndarray) -> QrDetectionResult:
        """
        检测图像中的二维码 / Detect QR codes in image.

        Args:
            image_frame: 输入 BGR 图像 / Input BGR image.

        Returns:
            二维码检测结果 / QR detection result.
        """
        backend_name = self.qr_detector_backend.lower()
        if backend_name not in {"opencv", "pyzbar", "auto"}:
            backend_name = "pyzbar"

        all_qr_codes: List[QrCodeData] = []

        if backend_name in {"pyzbar", "auto"}:
            pyzbar_qr_codes = self.detect_qr_codes_with_pyzbar(image_frame)
            all_qr_codes.extend(pyzbar_qr_codes)
            if pyzbar_qr_codes and backend_name == "pyzbar":
                return self._build_detection_result(all_qr_codes, image_frame)

        if backend_name in {"opencv", "auto"} and not all_qr_codes:
            opencv_qr_codes = self.detect_qr_codes_with_opencv(image_frame)
            all_qr_codes.extend(opencv_qr_codes)

        return self._build_detection_result(all_qr_codes, image_frame)

    def detect_qr_codes_with_pyzbar(self, image_frame: np.ndarray) -> List[QrCodeData]:
        """
        使用 pyzbar 检测二维码 / Detect QR codes using pyzbar.

        Args:
            image_frame: BGR 图像 / BGR image.

        Returns:
            二维码列表 / QR code list.
        """
        if pyzbar_module is None:
            self._report_pyzbar_missing_if_needed()
            return []

        try:
            gray_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2GRAY)
            decoded_barcodes = pyzbar_module.decode(gray_frame)
        except Exception as exception_error:  # pylint: disable=broad-except
            self._throttled_debug_log(f"pyzbar QR detection failed: {exception_error}")
            return []

        qr_codes: List[QrCodeData] = []
        for decoded_barcode in decoded_barcodes:
            payload_text = decoded_barcode.data.decode("utf-8", errors="ignore").strip()
            if not payload_text:
                continue

            rect_x, rect_y, rect_w, rect_h = decoded_barcode.rect
            polygon_points = tuple((int(point.x), int(point.y)) for point in decoded_barcode.polygon)
            if not polygon_points:
                polygon_points = (
                    (int(rect_x), int(rect_y)),
                    (int(rect_x + rect_w), int(rect_y)),
                    (int(rect_x + rect_w), int(rect_y + rect_h)),
                    (int(rect_x), int(rect_y + rect_h)),
                )

            qr_codes.append(
                QrCodeData(
                    payload=payload_text,
                    qr_type=str(decoded_barcode.type),
                    rectangle=(int(rect_x), int(rect_y), int(rect_w), int(rect_h)),
                    polygon=polygon_points,
                    center=(float(rect_x + rect_w * 0.5), float(rect_y + rect_h * 0.5)),
                )
            )

        return qr_codes

    def detect_qr_codes_with_opencv(self, image_frame: np.ndarray) -> List[QrCodeData]:
        """
        使用 OpenCV QRCodeDetector 检测二维码 / Detect QR codes using OpenCV QRCodeDetector.

        Args:
            image_frame: BGR 图像 / BGR image.

        Returns:
            二维码列表 / QR code list.
        """
        qr_codes: List[QrCodeData] = []

        try:
            if hasattr(self.opencv_qr_detector, "detectAndDecodeMulti"):
                detection_ok, decoded_info, points, _ = self.opencv_qr_detector.detectAndDecodeMulti(image_frame)
                if detection_ok and points is not None and decoded_info:
                    for payload_text, point_array in zip(decoded_info, points):
                        payload_text = str(payload_text).strip()
                        if not payload_text:
                            continue
                        qr_code_data = self._make_qr_code_data_from_points(
                            payload_text,
                            point_array,
                            "QRCODE",
                        )
                        if qr_code_data is not None:
                            qr_codes.append(qr_code_data)

            if qr_codes:
                return qr_codes

            payload_text, point_array, _ = self.opencv_qr_detector.detectAndDecode(image_frame)
            payload_text = str(payload_text).strip()
            if payload_text and point_array is not None:
                qr_code_data = self._make_qr_code_data_from_points(
                    payload_text,
                    point_array,
                    "QRCODE",
                )
                if qr_code_data is not None:
                    qr_codes.append(qr_code_data)

        except Exception as exception_error:  # pylint: disable=broad-except
            self._throttled_debug_log(f"OpenCV QR detection failed: {exception_error}")

        return qr_codes

    def _make_qr_code_data_from_points(
        self,
        payload_text: str,
        point_array: np.ndarray,
        qr_type: str,
    ) -> Optional[QrCodeData]:
        """
        根据角点生成二维码数据 / Build QR data from corner points.

        Args:
            payload_text: 二维码内容 / QR payload text.
            point_array: 角点数组 / Corner point array.
            qr_type: 二维码类型 / QR type.

        Returns:
            二维码数据，失败返回 None / QR data, or None on failure.
        """
        try:
            reshaped_points = np.array(point_array, dtype=np.float32).reshape(-1, 2)
            if reshaped_points.shape[0] < 4:
                return None

            polygon_points = tuple((int(point[0]), int(point[1])) for point in reshaped_points)
            x_coord, y_coord, width, height = cv2.boundingRect(np.array(polygon_points, dtype=np.int32))
            center_x = float(np.mean(reshaped_points[:, 0]))
            center_y = float(np.mean(reshaped_points[:, 1]))

            return QrCodeData(
                payload=payload_text,
                qr_type=qr_type,
                rectangle=(int(x_coord), int(y_coord), int(width), int(height)),
                polygon=polygon_points,
                center=(center_x, center_y),
            )
        except Exception as exception_error:  # pylint: disable=broad-except
            self._throttled_debug_log(f"Make QR data from points failed: {exception_error}")
            return None

    def _build_detection_result(
        self,
        qr_codes: Sequence[QrCodeData],
        image_frame: np.ndarray,
    ) -> QrDetectionResult:
        """
        根据候选二维码生成检测结果 / Build detection result from QR candidates.

        Args:
            qr_codes: 候选二维码列表 / QR candidate list.
            image_frame: 输入图像 / Input image.

        Returns:
            二维码检测结果 / QR detection result.
        """
        if not qr_codes:
            return QrDetectionResult(False)

        selected_qr_code = self._select_main_qr_code(qr_codes, image_frame)
        return QrDetectionResult(
            is_qr_found=True,
            qr_count=len(qr_codes),
            main_qr=selected_qr_code,
            all_qr_codes=tuple(qr_codes),
        )

    def _select_main_qr_code(
        self,
        qr_codes: Sequence[QrCodeData],
        image_frame: np.ndarray,
    ) -> QrCodeData:
        """
        从多个二维码中选择主二维码 / Select main QR code from candidates.

        Args:
            qr_codes: 二维码候选列表 / QR candidate list.
            image_frame: 输入图像 / Input image.

        Returns:
            主二维码 / Main QR code.
        """
        if self.qr_select_strategy == "nearest_center":
            image_height, image_width = image_frame.shape[:2]
            image_center_x = float(image_width) * 0.5
            image_center_y = float(image_height) * 0.5
            return min(
                qr_codes,
                key=lambda qr_code: math.hypot(qr_code.center[0] - image_center_x, qr_code.center[1] - image_center_y),
            )

        if self.qr_select_strategy == "first":
            return qr_codes[0]

        return max(qr_codes, key=lambda qr_code: qr_code.rectangle[2] * qr_code.rectangle[3])

    def _print_detection_log_if_needed(self, detection_result: QrDetectionResult) -> None:
        """
        按固定间隔打印识别结果 / Print detection result periodically.

        Args:
            detection_result: 二维码检测结果 / QR detection result.
        """
        if not self.print_detection_log:
            return

        current_time_sec = time.monotonic()
        if current_time_sec - self.last_detection_log_time_sec < self.detection_log_interval_sec:
            return

        self.last_detection_log_time_sec = current_time_sec
        if detection_result.is_qr_found and detection_result.main_qr is not None:
            self.get_logger().info(
                f"Detected QR codes: {detection_result.qr_count}, "
                f"main_payload='{detection_result.main_qr.payload}', "
                f"rect={detection_result.main_qr.rectangle}, backend={self.qr_detector_backend}"
            )
        else:
            self.get_logger().info(f"Detected QR codes: 0, backend={self.qr_detector_backend}")

    def _draw_and_show_debug_image(
        self,
        image_frame: np.ndarray,
        detection_result: QrDetectionResult,
    ) -> None:
        """
        绘制并显示识别图像 / Draw and show detection image.

        Args:
            image_frame: 待显示图像 / Image frame to display.
            detection_result: 二维码检测结果 / QR detection result.
        """
        display_image = image_frame.copy()
        self._draw_qr_detection_result(display_image, detection_result)
        self._draw_overlay_text(display_image, detection_result)

        try:
            cv2.imshow(self.window_name, display_image)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"OpenCV imshow failed, disable show_debug_window if running headless: {exception_error}"
            )

    def _draw_qr_detection_result(
        self,
        display_image: np.ndarray,
        detection_result: QrDetectionResult,
    ) -> None:
        """
        绘制二维码检测框 / Draw QR detection boxes.

        Args:
            display_image: 显示图像 / Display image.
            detection_result: 二维码检测结果 / QR detection result.
        """
        frame_height, frame_width = display_image.shape[:2]
        cv2.line(
            display_image,
            (frame_width // 2 - 10, frame_height // 2),
            (frame_width // 2 + 10, frame_height // 2),
            (255, 255, 0),
            1,
        )
        cv2.line(
            display_image,
            (frame_width // 2, frame_height // 2 - 10),
            (frame_width // 2, frame_height // 2 + 10),
            (255, 255, 0),
            1,
        )

        for qr_index, qr_code in enumerate(detection_result.all_qr_codes, start=1):
            x_coord, y_coord, width, height = qr_code.rectangle
            is_main_qr = detection_result.main_qr is not None and qr_code == detection_result.main_qr
            rectangle_color = (0, 255, 0) if not is_main_qr else (255, 0, 255)

            cv2.rectangle(
                display_image,
                (x_coord, y_coord),
                (x_coord + width, y_coord + height),
                rectangle_color,
                2,
            )

            if qr_code.polygon:
                polygon_array = np.array(qr_code.polygon, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(display_image, [polygon_array], True, (0, 180, 255), 1)

            cv2.circle(
                display_image,
                (int(round(qr_code.center[0])), int(round(qr_code.center[1]))),
                3,
                (0, 0, 255),
                -1,
            )

            label_text = f"QR {qr_index}: {qr_code.payload}"
            cv2.putText(
                display_image,
                label_text,
                (x_coord, max(15, y_coord - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                rectangle_color,
                1,
                cv2.LINE_AA,
            )

    def _draw_overlay_text(
        self,
        display_image: np.ndarray,
        detection_result: QrDetectionResult,
    ) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            display_image: 显示图像 / Display image.
            detection_result: 二维码检测结果 / QR detection result.
        """
        frame_rate_value = self.header_based_frame_rate if self.use_header_stamp_for_fps else self.receive_based_frame_rate
        main_payload = detection_result.main_qr.payload if detection_result.main_qr is not None else "None"

        overlay_lines = [
            "mode: QR detection only",
            f"qrs: {detection_result.qr_count}",
            f"payload: {main_payload}",
            f"fps: {frame_rate_value:.1f}",
            f"backend: {self.qr_detector_backend}",
            "q/ESC: quit",
        ]
        if self.show_image_stamp_delay:
            overlay_lines.insert(4, f"delay: {self.image_stamp_delay_ms:.1f}ms")

        if self.overlay_show_background:
            background_height = max(self.overlay_line_height * len(overlay_lines) + 8, 1)
            cv2.rectangle(
                display_image,
                (0, 0),
                (min(display_image.shape[1], 520), min(display_image.shape[0], background_height)),
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

        Args:
            key_code: OpenCV 按键编码 / OpenCV key code.
        """
        if key_code in (255, -1):
            return

        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._force_exit_process("q/ESC key pressed.")
            return

        if key_code in (ord("r"), ord("R")):
            self.latest_detection_result = QrDetectionResult(False)
            self.get_logger().info("QR detection result reset.")

    def _force_exit_process(self, reason: str) -> None:
        """
        直接退出当前 Python 进程，避免 rclpy.shutdown 或 cv2.destroyAllWindows 卡死。
        Directly exit the current Python process to avoid shutdown/destroy-window freeze.

        Args:
            reason: 退出原因 / Exit reason.
        """
        self.is_shutdown_requested = True
        try:
            self.get_logger().info(f"Force exiting detect_qr_node: {reason}")
        except Exception:
            pass

        # 不调用 rclpy.shutdown，不调用 cv2.destroyAllWindows。
        # Do not call rclpy.shutdown or cv2.destroyAllWindows here.
        os._exit(0)  # pylint: disable=protected-access

    def _throttled_debug_log(self, log_text: str) -> None:
        """
        节流输出调试日志 / Throttled debug log output.

        Args:
            log_text: 日志文本 / Log text.
        """
        current_time_sec = time.monotonic()
        if current_time_sec - self.last_debug_log_time_sec >= self.debug_log_interval_sec:
            self.get_logger().info(log_text)
            self.last_debug_log_time_sec = current_time_sec

    def _report_pyzbar_missing_if_needed(self) -> None:
        """按需提示 pyzbar 不可用 / Report unavailable pyzbar if needed."""
        self._throttled_debug_log("pyzbar is unavailable. Please install libzbar0 and python3-pyzbar, or run with qr_detector_backend:=opencv only if your OpenCV links QUIRC.")

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

        try:
            self._load_ros_parameters()
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
        # 注意：这里故意不调用 cv2.destroyAllWindows()。
        # 部分板子/远程桌面/OpenCV HighGUI 环境下 destroyAllWindows 会卡死。
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    节点入口函数 / Node entry function.

    Args:
        args: ROS2 参数 / ROS2 arguments.
    """
    rclpy.init(args=args)

    detect_qr_node = DetectQrNode()
    try:
        while rclpy.ok() and not detect_qr_node.is_shutdown_requested:
            rclpy.spin_once(detect_qr_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        # q/ESC 会直接 os._exit(0)，不会进入这里。
        # Ctrl+C 会进入这里，做 ROS 节点销毁；仍然不主动销毁 OpenCV 窗口。
        try:
            detect_qr_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
