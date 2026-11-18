#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_face.py


English Description:
    ROS2 Humble face detection node.
    This version removes PTZ control, PID, PTZ angle publishing, and target tracking state switching.
    It only keeps raw image subscription, face detection, visualization, and face-count logging.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image


# Current visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# Default node name.
DEFAULT_NODE_NAME = "detect_face_node"

# ESC key code.
KEY_CODE_ESCAPE = 27

# Lower bound of face detection scale.
MINIMUM_FACE_DETECTION_SCALE = 0.10

# Upper bound of face detection scale.
MAXIMUM_FACE_DETECTION_SCALE = 1.00

# Runtime dynamic parameters.
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
    "face_cascade_file",
    "face_detection_scale_factor",
    "face_min_neighbors",
    "face_min_width",
    "face_min_height",
    "face_detection_scale",
    "enable_gray_equalize",
    "print_detection_log",
    "detection_log_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
})

# Parameters that require restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "window_name",
})


@dataclass
class FaceDetectionResult:
    """Face detection result."""

    # Whether at least one face is detected.
    is_face_detected: bool

    # Number of detected faces.
    face_count: int = 0

    # Main face center x coordinate.
    main_face_center_x: float = 0.0

    # Main face center y coordinate.
    main_face_center_y: float = 0.0

    # Main face bounding rectangle.
    main_face_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)

    # All detected face rectangles.
    all_face_rectangles: Tuple[Tuple[int, int, int, int], ...] = ()


def clamp_numeric_value(current_value: float, minimum_value: float, maximum_value: float) -> float:
    """
    Clamp numeric value.

    Args:
        Current value.
        Minimum value.
        Maximum value.

    Returns:
        Clamped value.
    """
    return max(float(minimum_value), min(float(maximum_value), float(current_value)))


def convert_value_to_boolean(input_value: Any, default_boolean_value: bool) -> bool:
    """
    Safely convert value to boolean.

    Args:
        Input parameter value.
        Default boolean value.

    Returns:
        Converted boolean value.
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


def get_default_face_cascade_file() -> str:
    """
    Get default face cascade classifier file path.

    Returns:
        Default Haar face classifier path.
    """
    try:
        package_share_directory = get_package_share_directory(VISUAL_PACKAGE_NAME)
        package_cascade_file = os.path.join(
            package_share_directory,
            "config",
            "haarcascade_frontalface_default.xml",
        )
        if os.path.exists(package_cascade_file):
            return package_cascade_file
    except Exception:  # pylint: disable=broad-except
        print(f"Warning: Failed to get haarcascades for {VISUAL_PACKAGE_NAME}. ")


class DetectFaceNode(Node):
    """ROS2 face detection node."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # OpenCV face classifier.
        self.face_cascade_classifier: Optional[cv2.CascadeClassifier] = None

        # Whether shutdown is requested.
        self.is_shutdown_requested = False
        self.image_bridge = CvBridge()

        # Previous frame header timestamp.
        self.previous_header_stamp_sec = 0.0

        # Header stamp based FPS.
        self.header_based_frame_rate = 0.0

        # Receive time based FPS.
        self.receive_based_frame_rate = 0.0

        # Previous receive time.
        self.previous_receive_time_sec = 0.0

        # Image header-to-receive delay in ms.
        self.image_stamp_delay_ms = 0.0

        # Last detection log time.
        self.last_detection_log_time_sec = 0.0

        # Latest face detection result.
        self.latest_detection_result = FaceDetectionResult(False)

        self._declare_ros_parameters()
        self._load_ros_parameters()
        self._reload_face_cascade_classifier()
        self._initialize_debug_window()

        # Image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        # Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        self.get_logger().info(
            "Detect face node started. "
            "Only face detection is enabled; PTZ control has been removed. "
            "Press q or ESC in OpenCV window to quit. "
            f"image_topic={self.image_topic_name}, face_cascade={self.face_cascade_file}"
        )

    def _declare_ros_parameters(self) -> None:
        """Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("window_name", "detect_face_node")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.45)
        self.declare_parameter("overlay_line_height", 18)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 18)
        self.declare_parameter("overlay_show_background", False)

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
        self.declare_parameter("enable_gray_equalize", True)

        self.declare_parameter("print_detection_log", True)
        self.declare_parameter("detection_log_interval_sec", 1.0)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)

    def _load_ros_parameters(self) -> None:
        """Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
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
        self.enable_gray_equalize = convert_value_to_boolean(
            self.get_parameter("enable_gray_equalize").value,
            True,
        )

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

    def _reload_face_cascade_classifier(self) -> bool:
        """
        Reload face cascade classifier.

        Returns:
            Whether loading succeeded.
        """
        resolved_face_cascade_file = os.path.expanduser(self.face_cascade_file)

        face_cascade_classifier = cv2.CascadeClassifier(resolved_face_cascade_file)
        if face_cascade_classifier.empty():
            fallback_face_cascade_file = os.path.join(
                cv2.data.haarcascades,
                "haarcascade_frontalface_default.xml",
            )
            self.get_logger().warn(
                f"Failed to load face cascade file: {resolved_face_cascade_file}. "
                f"Trying OpenCV default: {fallback_face_cascade_file}"
            )
            resolved_face_cascade_file = fallback_face_cascade_file
            face_cascade_classifier = cv2.CascadeClassifier(resolved_face_cascade_file)

        if face_cascade_classifier.empty():
            self.face_cascade_classifier = None
            self.get_logger().error(
                "Failed to load Haar face cascade. "
                "Please check parameter 'face_cascade_file'."
            )
            return False

        self.face_cascade_file = resolved_face_cascade_file
        self.face_cascade_classifier = face_cascade_classifier
        self.get_logger().info(f"Face cascade loaded: {self.face_cascade_file}")
        return True

    def _initialize_debug_window(self) -> None:
        """Initialize OpenCV window."""
        if not self.show_debug_window:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

    def _update_debug_window(self) -> None:
        """Update OpenCV window."""
        if not self.show_debug_window:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:  # pylint: disable=broad-except
                pass
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)

    def image_callback(self, image_msg: Image) -> None:
        """
        Receive, decode, and detect faces.

        Args:
            Raw image message.
        """
        if self.is_shutdown_requested:
            return

        try:
            image_frame = self.image_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            self._update_image_timing_statistics(image_msg)
            image_frame = self._apply_image_orientation(image_frame)

            detection_result = self.detect_faces(image_frame)
            self.latest_detection_result = detection_result

            self._print_detection_log_if_needed(detection_result)

            if self.show_debug_window:
                self._draw_and_show_debug_image(image_frame, detection_result)

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

    def _apply_image_orientation(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Apply image orientation correction.

        Args:
            Input image.

        Returns:
            Orientation-corrected image.
        """
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
        """
        Update image FPS and delay statistics.

        Args:
            Raw image message.
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

    def detect_faces(self, image_frame: np.ndarray) -> FaceDetectionResult:
        """
        Detect faces in image.

        Args:
            Input BGR image.

        Returns:
            Face detection result.
        """
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

        main_face_rectangle = max(
            mapped_face_rectangles,
            key=lambda rectangle: int(rectangle[2]) * int(rectangle[3]),
        )
        main_face_x, main_face_y, main_face_width, main_face_height = main_face_rectangle
        main_face_center_x = float(main_face_x) + float(main_face_width) * 0.5
        main_face_center_y = float(main_face_y) + float(main_face_height) * 0.5

        return FaceDetectionResult(
            is_face_detected=True,
            face_count=len(mapped_face_rectangles),
            main_face_center_x=main_face_center_x,
            main_face_center_y=main_face_center_y,
            main_face_rectangle=main_face_rectangle,
            all_face_rectangles=tuple(mapped_face_rectangles),
        )

    def _print_detection_log_if_needed(self, detection_result: FaceDetectionResult) -> None:
        """
        Print detection result periodically.

        Args:
            Face detection result.
        """
        if not self.print_detection_log:
            return

        current_time_sec = time.monotonic()
        if current_time_sec - self.last_detection_log_time_sec < self.detection_log_interval_sec:
            return

        self.last_detection_log_time_sec = current_time_sec
        if detection_result.is_face_detected:
            self.get_logger().info(f"Detected faces: {detection_result.face_count}")
        else:
            self.get_logger().info("Detected faces: 0")

    def _draw_and_show_debug_image(
        self,
        image_frame: np.ndarray,
        detection_result: FaceDetectionResult,
    ) -> None:
        """
        Draw and show detection image.

        Args:
            Image frame to display.
            Face detection result.
        """
        display_image = image_frame.copy()

        for face_index, face_rectangle in enumerate(detection_result.all_face_rectangles, start=1):
            face_x, face_y, face_width, face_height = face_rectangle
            cv2.rectangle(
                display_image,
                (face_x, face_y),
                (face_x + face_width, face_y + face_height),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display_image,
                f"Face {face_index}",
                (face_x, max(face_y - 8, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        if detection_result.is_face_detected:
            cv2.circle(
                display_image,
                (
                    int(round(detection_result.main_face_center_x)),
                    int(round(detection_result.main_face_center_y)),
                ),
                4,
                (0, 0, 255),
                -1,
            )

        self._draw_overlay_text(display_image, detection_result)
        cv2.imshow(self.window_name, display_image)

        key_code = cv2.waitKey(1) & 0xFF
        self._handle_key_input(key_code)

    def _draw_overlay_text(
        self,
        display_image: np.ndarray,
        detection_result: FaceDetectionResult,
    ) -> None:
        """
        Draw overlay text.

        Args:
            Display image.
            Face detection result.
        """
        frame_rate_value = self.header_based_frame_rate if self.use_header_stamp_for_fps else self.receive_based_frame_rate

        overlay_lines = [
            "mode: face detection only",
            f"faces: {detection_result.face_count}",
            f"fps: {frame_rate_value:.1f}",
            "q/ESC: quit",
        ]
        if self.show_image_stamp_delay:
            overlay_lines.insert(3, f"delay: {self.image_stamp_delay_ms:.1f}ms")

        if self.overlay_show_background:
            background_height = max(self.overlay_line_height * len(overlay_lines) + 8, 1)
            cv2.rectangle(
                display_image,
                (0, 0),
                (min(display_image.shape[1], 360), min(display_image.shape[0], background_height)),
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
        Handle keyboard input.

        Args:
            OpenCV key code.
        """
        if key_code in (ord("q"), ord("Q"), KEY_CODE_ESCAPE):
            self.is_shutdown_requested = True
            rclpy.shutdown()

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        Dynamic parameter update callback.

        Args:
            Parameter list.

        Returns:
            Parameter update result.
        """
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

        previous_face_cascade_file = self.face_cascade_file

        try:
            self._load_ros_parameters()
            if previous_face_cascade_file != self.face_cascade_file:
                self._reload_face_cascade_classifier()
            self._update_debug_window()
        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception_error))

        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        """
        Destroy node.

        Returns:
            Parent destroy result.
        """
        try:
            cv2.destroyAllWindows()
        except Exception:  # pylint: disable=broad-except
            pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    Node entry function.

    Args:
        ROS2 arguments.
    """
    rclpy.init(args=args)

    detect_face_node = DetectFaceNode()
    try:
        rclpy.spin(detect_face_node)
    except KeyboardInterrupt:
        pass
    finally:
        detect_face_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
