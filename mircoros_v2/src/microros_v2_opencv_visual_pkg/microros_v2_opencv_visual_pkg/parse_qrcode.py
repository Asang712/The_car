#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parse_qrcode.py



English Description:
    ROS2 Humble QR code parser node.
    The QR parsing method follows the original program:
    pyzbar.decode(gray) + PIL ImageDraw + font_path based text rendering.
"""

from __future__ import annotations

import os
import time
from typing import Any, List, Optional, Tuple, Union

import cv2 as cv
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    import pyzbar.pyzbar as pyzbar
except Exception:  # pylint: disable=broad-except
    pyzbar = None


# Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# Default node name.
DEFAULT_NODE_NAME = "parse_qrcode_node"

# ESC key code.
KEY_CODE_ESCAPE = 27

# Image message type accepted by this node
ImageMessageType = Union[Image, CompressedImage]

# Runtime dynamic parameters.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "frame_width",
    "frame_height",
    "window_width",
    "window_height",
    "font_path",
    "font_size",
    "fps_font_scale",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "print_decode_log",
    "decode_log_interval_sec",
    "enable_rqt_dynamic_parameters",
})

# Parameters that require restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "use_compressed_image",
    "window_name",
})


def get_default_font_path() -> str:
    """
    Get default font path.
    """
    try:
        package_share_directory = get_package_share_directory(VISUAL_PACKAGE_NAME)
        package_font_path = os.path.join(
            package_share_directory,
            "config",
            "Block_Simplified.TTF",
        )
        if os.path.exists(package_font_path):
            return package_font_path
    except Exception:  # pylint: disable=broad-except
        pass

    return ""


def get_default_image_topic() -> str:
    """
    Get default image topic.

    Returns:
        Default image topic.
    """
    return "/camera/image_raw"


def convert_value_to_boolean(input_value: Any, default_boolean_value: bool) -> bool:
    """
    Safely convert value to boolean.
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


class QRCodeDecoderNode(Node):
    """QR code decoder node."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        self.bridge = CvBridge()
        self.is_shutdown_requested = False

        self.last_frame_time_sec = 0.0
        self.current_fps = 0.0
        self.last_decode_log_time_sec = 0.0
        self.latest_barcode_count = 0

        self._declare_ros_parameters()
        self._load_ros_parameters()

        subscribed_image_message_type = CompressedImage if self.use_compressed_image else Image
        self.subscription = self.create_subscription(
            subscribed_image_message_type,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        self.keyboard_poll_timer = None
        if self.show_debug_window:
            self._initialize_debug_window()
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)

        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        if pyzbar is None:
            self.get_logger().error(
                "pyzbar is unavailable. Please install libzbar0 and python3-pyzbar."
            )

        self.get_logger().info(
            "Parse QR code node started. "
            "QR decode uses pyzbar.decode(gray), drawing uses PIL ImageDraw. "
            "Press q or ESC to quit. "
            f"image_topic={self.image_topic_name}, use_compressed_image={self.use_compressed_image}, "
            f"font_path={self.font_path}"
        )

    def _declare_ros_parameters(self) -> None:
        """Declare ROS parameters."""
        self.declare_parameter("image_topic", get_default_image_topic())
        self.declare_parameter("use_compressed_image", False)
        self.declare_parameter("font_path", get_default_font_path())
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("window_name", "frame")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("font_size", 18)
        self.declare_parameter("fps_font_scale", 0.6)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("print_decode_log", True)
        self.declare_parameter("decode_log_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_ros_parameters(self) -> None:
        """Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
        self.use_compressed_image = convert_value_to_boolean(
            self.get_parameter("use_compressed_image").value,
            True,
        )
        self.font_path = os.path.expanduser(str(self.get_parameter("font_path").value))
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.frame_width = int(self.get_parameter("frame_width").value)
        self.frame_height = int(self.get_parameter("frame_height").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.show_debug_window = convert_value_to_boolean(
            self.get_parameter("show_debug_window").value,
            True,
        )
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.font_size = max(8, int(self.get_parameter("font_size").value))
        self.fps_font_scale = max(0.1, float(self.get_parameter("fps_font_scale").value))
        self.enable_horizontal_flip = convert_value_to_boolean(
            self.get_parameter("enable_horizontal_flip").value,
            False,
        )
        self.enable_vertical_flip = convert_value_to_boolean(
            self.get_parameter("enable_vertical_flip").value,
            False,
        )
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value).lower()
        self.print_decode_log = convert_value_to_boolean(
            self.get_parameter("print_decode_log").value,
            True,
        )
        self.decode_log_interval_sec = max(
            float(self.get_parameter("decode_log_interval_sec").value),
            0.05,
        )
        self.enable_rqt_dynamic_parameters = convert_value_to_boolean(
            self.get_parameter("enable_rqt_dynamic_parameters").value,
            True,
        )

    def _initialize_debug_window(self) -> None:
        """Initialize OpenCV window."""
        if not self.show_debug_window:
            return
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, self.window_width, self.window_height)

    def _keyboard_poll_timer_callback(self) -> None:
        """Poll OpenCV keyboard events independently."""
        if self.is_shutdown_requested or not self.show_debug_window:
            return
        try:
            key_code = cv.waitKey(1) & 0xFF
            self._handle_key_input(key_code)
        except Exception:
            pass

    def _decode_input_image_message(self, image_message: ImageMessageType) -> Optional[np.ndarray]:
        """
        Decode input image message.
        """
        try:
            if self.use_compressed_image:
                compressed_image_buffer = np.frombuffer(image_message.data, dtype=np.uint8)
                decoded_frame = cv.imdecode(compressed_image_buffer, cv.IMREAD_COLOR)
                if decoded_frame is None:
                    self.get_logger().warn("Compressed image decode failed.")
                    return None
                return decoded_frame

            return self.bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image decode failed: {exception_error}")
            return None

    def _apply_image_orientation(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Apply image orientation correction.
        """
        corrected_image_frame = image_frame

        if self.enable_horizontal_flip:
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

    def _resize_image_if_needed(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Resize image according to parameters.
        """
        if self.frame_width > 0 and self.frame_height > 0:
            return cv.resize(image_frame, (self.frame_width, self.frame_height), interpolation=cv.INTER_AREA)
        return image_frame

    def _load_text_font(self):
        """
        Load PIL font.
        """
        if self.font_path and os.path.exists(self.font_path):
            try:
                return ImageFont.truetype(self.font_path, size=self.font_size, encoding="UTF-8")
            except Exception as exception_error:  # pylint: disable=broad-except
                self.get_logger().warn(f"Load font failed, using default font: {exception_error}")

        return ImageFont.load_default()

    def decode_display(self, image: np.ndarray) -> Tuple[np.ndarray, int]:
        """
        Decode QR code and draw result.

            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            barcodes = pyzbar.decode(gray)
            PILImage.fromarray(...)
            ImageDraw.Draw(...)
        """
        if pyzbar is None:
            return image, 0

        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)

        rgb_image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        pilimg = PILImage.fromarray(rgb_image)
        draw = ImageDraw.Draw(pilimg)
        font_style = self._load_text_font()

        current_time_sec = time.monotonic()
        should_print_log = (
            self.print_decode_log
            and current_time_sec - self.last_decode_log_time_sec >= self.decode_log_interval_sec
        )

        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            encoding = "UTF-8"
            barcode_data = barcode.data.decode(encoding, errors="ignore")
            barcode_type = barcode.type

            draw.rectangle([(x, y), (x + w, y + h)], outline=(225, 0, 0), width=5)
            draw.text((x, max(0, y - 25)), str(barcode.data, encoding, errors="ignore"), fill=(255, 0, 0), font=font_style)

            if should_print_log:
                self.get_logger().info(f"Found {barcode_type} barcode: {barcode_data}")

        if should_print_log and barcodes:
            self.last_decode_log_time_sec = current_time_sec

        image = cv.cvtColor(np.array(pilimg), cv.COLOR_RGB2BGR)
        return image, len(barcodes)

    def _update_fps(self, start_time_sec: float) -> None:
        """
        Update FPS.
        """
        elapsed_time_sec = max(time.time() - start_time_sec, 1.0e-6)
        self.current_fps = 1.0 / elapsed_time_sec

    def image_callback(self, msg: ImageMessageType) -> None:
        """
        Image callback.
        """
        if self.is_shutdown_requested:
            return

        try:
            start = time.time()
            frame = self._decode_input_image_message(msg)
            if frame is None:
                return

            frame = self._apply_image_orientation(frame)
            frame = self._resize_image_if_needed(frame)
            frame, barcode_count = self.decode_display(frame)
            self.latest_barcode_count = barcode_count
            self._update_fps(start)

            text = "FPS : " + str(int(self.current_fps))
            cv.putText(frame, text, (30, 30), cv.FONT_HERSHEY_SIMPLEX, self.fps_font_scale, (100, 200, 200), 1)

            count_text = "QR : " + str(int(barcode_count))
            cv.putText(frame, count_text, (30, 55), cv.FONT_HERSHEY_SIMPLEX, self.fps_font_scale, (100, 200, 200), 1)

            if self.show_debug_window:
                cv.imshow(self.window_name, frame)

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def _handle_key_input(self, key_code: int) -> None:
        """
        Handle keyboard input.
        """
        if key_code in (255, -1):
            return

        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self._force_exit_process("q/ESC key pressed.")

    def _force_exit_process(self, reason: str) -> None:
        """
        Directly exit current process to avoid shutdown/destroy window freeze.
        """
        self.is_shutdown_requested = True
        try:
            self.get_logger().info(f"Force exiting parse_qrcode_node: {reason}")
        except Exception:
            pass

        os._exit(0)  # pylint: disable=protected-access

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        Dynamic parameter update callback.
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
            previous_show_debug_window = self.show_debug_window
            self._load_ros_parameters()

            if self.show_debug_window and not previous_show_debug_window:
                self._initialize_debug_window()

            if self.keyboard_poll_timer is None and self.show_debug_window:
                self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)
        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception_error))

        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        """
        Destroy node.

        """
        return super().destroy_node()


def main(args=None) -> None:
    """
    Node entry function.
    """
    rclpy.init(args=args)
    node = QRCodeDecoderNode()

    try:
        while rclpy.ok() and not node.is_shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
