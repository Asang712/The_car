#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_apriltag.py


English Description:
    ROS2 Humble AprilTag detection and 3D pose node.
    It uses dt_apriltags.Detector and loads camera calibration from config/camera.yaml.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any, List, Optional, Tuple, Union

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    from dt_apriltags import Detector
except Exception as import_exception:  # pylint: disable=broad-except
    Detector = None
    DETECTOR_IMPORT_EXCEPTION = import_exception
else:
    DETECTOR_IMPORT_EXCEPTION = None


# Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# Default node name.
DEFAULT_NODE_NAME = "detect_apriltag_node"

# ESC key code.
KEY_CODE_ESCAPE = 27

# Package share directory calibration path.
DEFAULT_SOURCE_CAMERA_YAML_PATH = os.path.join(
    get_package_share_directory(VISUAL_PACKAGE_NAME),
    "config",
    "camera.yaml",
)

# Image message type accepted by this node
ImageMessageType = Union[Image, CompressedImage]


@dataclass(frozen=True)
class CameraCalibrationInfo:
    """Camera calibration information."""

    is_valid: bool
    image_width: int = 0
    image_height: int = 0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    distortion_coefficients: Tuple[float, ...] = ()
    source_path: str = ""


@dataclass(frozen=True)
class AprilTagInfo:
    """AprilTag detection information."""

    tag_id: int
    center_x: float
    center_y: float
    corners: Tuple[Tuple[int, int], ...]
    hamming: int = -1
    decision_margin: float = 0.0
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_z: Optional[float] = None
    pose_roll_deg: Optional[float] = None
    pose_pitch_deg: Optional[float] = None
    pose_yaw_deg: Optional[float] = None
    pose_translation_vector: Optional[Tuple[float, float, float]] = None
    pose_rotation_matrix: Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]] = None


@dataclass(frozen=True)
class AprilTagDetectionResult:
    """AprilTag detection result."""

    is_tag_detected: bool
    tag_count: int = 0
    main_tag: Optional[AprilTagInfo] = None
    all_tags: Tuple[AprilTagInfo, ...] = ()


# Runtime dynamic parameters.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "publish_debug_image",
    "image_width",
    "image_height",
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
    "tag_family",
    "detector_nthreads",
    "quad_decimate",
    "quad_sigma",
    "refine_edges",
    "decode_sharpening",
    "detector_debug",
    "estimate_tag_pose",
    "tag_size",
    "camera_yaml_path",
    "use_camera_yaml",
    "use_undistort",
    "camera_fx",
    "camera_fy",
    "camera_cx",
    "camera_cy",
    "tag_select_strategy",
    "print_detection_log",
    "detection_log_interval_sec",
    "use_header_stamp_for_fps",
    "show_image_stamp_delay",
    "draw_tag_axes",
    "draw_axes_for_all_tags",
    "tag_axis_length",
    "tag_axis_thickness",
    "axis_label_font_scale",
    "draw_camera_axes",
    "camera_axis_pixel_length",
    "camera_axis_thickness",
    "camera_axis_label_font_scale",
    "enable_rqt_dynamic_parameters",
})

# Parameters that require restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "use_compressed_image",
    "debug_image_topic",
    "window_name",
})


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


def get_default_camera_yaml_path() -> str:
    """
    Get default camera.yaml path.

    Prefer install/share config/camera.yaml; otherwise use the user source package path.
    """
    try:
        package_share_directory = get_package_share_directory(VISUAL_PACKAGE_NAME)
        package_camera_yaml_path = os.path.join(package_share_directory, "config", "camera.yaml")
        if os.path.exists(package_camera_yaml_path):
            return package_camera_yaml_path
    except Exception:  # pylint: disable=broad-except
        pass

    return DEFAULT_SOURCE_CAMERA_YAML_PATH


def load_camera_calibration_from_yaml(camera_yaml_path: str) -> CameraCalibrationInfo:
    """
    Load camera intrinsics from camera.yaml.
    """
    expanded_camera_yaml_path = os.path.expanduser(str(camera_yaml_path))
    if not expanded_camera_yaml_path or not os.path.exists(expanded_camera_yaml_path):
        return CameraCalibrationInfo(False, source_path=expanded_camera_yaml_path)

    with open(expanded_camera_yaml_path, "r", encoding="utf-8") as yaml_file:
        camera_yaml_data = yaml.safe_load(yaml_file)

    image_width = int(camera_yaml_data.get("image_width", 0))
    image_height = int(camera_yaml_data.get("image_height", 0))

    camera_matrix_data = (
        camera_yaml_data.get("camera_matrix", {})
        .get("data", [])
    )

    if len(camera_matrix_data) < 9:
        return CameraCalibrationInfo(False, source_path=expanded_camera_yaml_path)

    fx = float(camera_matrix_data[0])
    fy = float(camera_matrix_data[4])
    cx = float(camera_matrix_data[2])
    cy = float(camera_matrix_data[5])

    distortion_data = (
        camera_yaml_data.get("distortion_coefficients", {})
        .get("data", [])
    )
    distortion_coefficients = tuple(float(value) for value in distortion_data)

    return CameraCalibrationInfo(
        is_valid=True,
        image_width=image_width,
        image_height=image_height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        distortion_coefficients=distortion_coefficients,
        source_path=expanded_camera_yaml_path,
    )


def rotation_matrix_to_euler_degrees(rotation_matrix: np.ndarray) -> Tuple[float, float, float]:
    """
    Convert rotation matrix to Euler angles.

    Returns:
        roll, pitch, yaw in degrees.
    """
    rotation_matrix = np.asarray(rotation_matrix, dtype=np.float64).reshape(3, 3)
    sy_value = math.sqrt(
        rotation_matrix[0, 0] * rotation_matrix[0, 0]
        + rotation_matrix[1, 0] * rotation_matrix[1, 0]
    )

    is_singular = sy_value < 1.0e-6
    if not is_singular:
        roll = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = math.atan2(-rotation_matrix[2, 0], sy_value)
        yaw = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    else:
        roll = math.atan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = math.atan2(-rotation_matrix[2, 0], sy_value)
        yaw = 0.0

    return (
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    )


class DetectAprilTagNode(Node):
    """ROS2 AprilTag detection and 3D pose node."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        self.cv_bridge = CvBridge()
        self.is_shutdown_requested = False

        self.previous_header_stamp_sec = 0.0
        self.header_based_frame_rate = 0.0
        self.receive_based_frame_rate = 0.0
        self.previous_receive_time_sec = 0.0
        self.image_stamp_delay_ms = 0.0
        self.last_detection_log_time_sec = 0.0
        self.last_pose_warning_time_sec = 0.0

        self.latest_detection_result = AprilTagDetectionResult(False)
        self.apriltag_detector = None
        self.camera_calibration = CameraCalibrationInfo(False)

        self._declare_ros_parameters()
        self._load_ros_parameters()
        self._load_camera_calibration_if_needed()
        self._recreate_apriltag_detector()

        subscribed_image_message_type = CompressedImage if self.use_compressed_image else Image
        self.image_subscription = self.create_subscription(
            subscribed_image_message_type,
            self.image_topic_name,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )

        self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic_name, 10)

        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        if self.show_debug_window:
            self._initialize_debug_window()
            self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)
        else:
            self.keyboard_poll_timer = None

        self.get_logger().info(
            "Detect AprilTag node started. "
            "Only AprilTag detection is enabled; servo/PID tracking has been removed. "
            "Press q or ESC to quit. "
            f"image_topic={self.image_topic_name}, use_compressed_image={self.use_compressed_image}, "
            f"tag_family={self.tag_family}, estimate_tag_pose={self.estimate_tag_pose}, "
            f"camera_yaml_path={self.camera_yaml_path}"
        )

    def _declare_ros_parameters(self) -> None:
        """Declare ROS parameters."""
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("use_compressed_image", False)
        self.declare_parameter("window_name", "detect_apriltag")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("debug_image_topic", "/detect_apriltag/debug_image")

        # Default matches the uploaded camera.yaml calibration size.
        self.declare_parameter("image_width", 320)
        self.declare_parameter("image_height", 240)

        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 14)
        self.declare_parameter("overlay_margin_x", 8)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)

        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        # AprilTag detector parameters
        self.declare_parameter("tag_family", "tag36h11")
        self.declare_parameter("detector_nthreads", 4)
        self.declare_parameter("quad_decimate", 2.0)
        self.declare_parameter("quad_sigma", 0.0)
        self.declare_parameter("refine_edges", True)
        self.declare_parameter("decode_sharpening", 0.25)
        self.declare_parameter("detector_debug", False)

        # Pose parameters
        self.declare_parameter("estimate_tag_pose", True)
        self.declare_parameter("tag_size", 0.025)
        self.declare_parameter("use_camera_yaml", True)
        self.declare_parameter("camera_yaml_path", get_default_camera_yaml_path())
        self.declare_parameter("use_undistort", False)

        # Manual intrinsics when not using camera_yaml.
        self.declare_parameter("camera_fx", 0.0)
        self.declare_parameter("camera_fy", 0.0)
        self.declare_parameter("camera_cx", 0.0)
        self.declare_parameter("camera_cy", 0.0)

        # Local coordinate frame drawing parameters.
        self.declare_parameter("draw_tag_axes", True)
        self.declare_parameter("draw_axes_for_all_tags", True)
        self.declare_parameter("tag_axis_length", 0.0)
        self.declare_parameter("tag_axis_thickness", 2)
        self.declare_parameter("axis_label_font_scale", 0.35)

        # Camera coordinate frame drawing parameters.
        self.declare_parameter("draw_camera_axes", True)
        self.declare_parameter("camera_axis_pixel_length", 38)
        self.declare_parameter("camera_axis_thickness", 2)
        self.declare_parameter("camera_axis_label_font_scale", 0.38)

        self.declare_parameter("tag_select_strategy", "smallest_id")
        self.declare_parameter("print_detection_log", True)
        self.declare_parameter("detection_log_interval_sec", 1.0)
        self.declare_parameter("use_header_stamp_for_fps", True)
        self.declare_parameter("show_image_stamp_delay", True)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_ros_parameters(self) -> None:
        """Load ROS parameters."""
        self.image_topic_name = str(self.get_parameter("image_topic").value)
        self.use_compressed_image = convert_value_to_boolean(
            self.get_parameter("use_compressed_image").value,
            True,
        )
        self.window_name = str(self.get_parameter("window_name").value)
        self.show_debug_window = convert_value_to_boolean(
            self.get_parameter("show_debug_window").value,
            True,
        )
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.publish_debug_image = convert_value_to_boolean(
            self.get_parameter("publish_debug_image").value,
            False,
        )
        self.debug_image_topic_name = str(self.get_parameter("debug_image_topic").value)

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

        self.tag_family = str(self.get_parameter("tag_family").value)
        self.detector_nthreads = max(1, int(self.get_parameter("detector_nthreads").value))
        self.quad_decimate = max(1.0, float(self.get_parameter("quad_decimate").value))
        self.quad_sigma = max(0.0, float(self.get_parameter("quad_sigma").value))
        self.refine_edges = convert_value_to_boolean(
            self.get_parameter("refine_edges").value,
            True,
        )
        self.decode_sharpening = max(0.0, float(self.get_parameter("decode_sharpening").value))
        self.detector_debug = convert_value_to_boolean(
            self.get_parameter("detector_debug").value,
            False,
        )

        self.estimate_tag_pose = convert_value_to_boolean(
            self.get_parameter("estimate_tag_pose").value,
            True,
        )
        self.tag_size = max(1.0e-6, float(self.get_parameter("tag_size").value))
        self.use_camera_yaml = convert_value_to_boolean(
            self.get_parameter("use_camera_yaml").value,
            True,
        )
        self.camera_yaml_path = os.path.expanduser(str(self.get_parameter("camera_yaml_path").value))
        self.use_undistort = convert_value_to_boolean(
            self.get_parameter("use_undistort").value,
            False,
        )

        self.camera_fx = float(self.get_parameter("camera_fx").value)
        self.camera_fy = float(self.get_parameter("camera_fy").value)
        self.camera_cx = float(self.get_parameter("camera_cx").value)
        self.camera_cy = float(self.get_parameter("camera_cy").value)

        self.draw_tag_axes = convert_value_to_boolean(
            self.get_parameter("draw_tag_axes").value,
            True,
        )
        self.draw_axes_for_all_tags = convert_value_to_boolean(
            self.get_parameter("draw_axes_for_all_tags").value,
            True,
        )
        self.tag_axis_length = max(0.0, float(self.get_parameter("tag_axis_length").value))
        self.tag_axis_thickness = max(1, int(self.get_parameter("tag_axis_thickness").value))
        self.axis_label_font_scale = max(0.1, float(self.get_parameter("axis_label_font_scale").value))

        self.draw_camera_axes = convert_value_to_boolean(
            self.get_parameter("draw_camera_axes").value,
            True,
        )
        self.camera_axis_pixel_length = max(10, int(self.get_parameter("camera_axis_pixel_length").value))
        self.camera_axis_thickness = max(1, int(self.get_parameter("camera_axis_thickness").value))
        self.camera_axis_label_font_scale = max(0.1, float(self.get_parameter("camera_axis_label_font_scale").value))

        self.tag_select_strategy = str(self.get_parameter("tag_select_strategy").value).lower()
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

    def _load_camera_calibration_if_needed(self) -> None:
        """Load camera calibration file."""
        if not self.use_camera_yaml:
            self.camera_calibration = CameraCalibrationInfo(False)
            return

        try:
            calibration_info = load_camera_calibration_from_yaml(self.camera_yaml_path)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.camera_calibration = CameraCalibrationInfo(False, source_path=self.camera_yaml_path)
            self.get_logger().error(f"Load camera yaml failed: {exception_error}")
            return

        self.camera_calibration = calibration_info
        if calibration_info.is_valid:
            self.get_logger().info(
                "Loaded camera calibration: "
                f"path={calibration_info.source_path}, "
                f"size={calibration_info.image_width}x{calibration_info.image_height}, "
                f"fx={calibration_info.fx:.5f}, fy={calibration_info.fy:.5f}, "
                f"cx={calibration_info.cx:.5f}, cy={calibration_info.cy:.5f}"
            )
        else:
            self.get_logger().warn(
                "Camera yaml is not valid or not found. "
                f"path={calibration_info.source_path}. "
                "Pose estimation will use manual camera_fx/camera_fy/camera_cx/camera_cy if valid."
            )

    def _recreate_apriltag_detector(self) -> None:
        """Create or recreate AprilTag detector."""
        if Detector is None:
            self.apriltag_detector = None
            self.get_logger().error(
                "dt_apriltags is unavailable. Please install dependency: python3 -m pip install dt-apriltags"
            )
            if DETECTOR_IMPORT_EXCEPTION is not None:
                self.get_logger().error(f"dt_apriltags import error: {DETECTOR_IMPORT_EXCEPTION}")
            return

        try:
            self.apriltag_detector = Detector(
                families=self.tag_family,
                nthreads=int(self.detector_nthreads),
                quad_decimate=float(self.quad_decimate),
                quad_sigma=float(self.quad_sigma),
                refine_edges=1 if self.refine_edges else 0,
                decode_sharpening=float(self.decode_sharpening),
                debug=1 if self.detector_debug else 0,
            )
            self.get_logger().info(
                "AprilTag detector ready: "
                f"families={self.tag_family}, nthreads={self.detector_nthreads}, "
                f"quad_decimate={self.quad_decimate}"
            )
        except Exception as exception_error:  # pylint: disable=broad-except
            self.apriltag_detector = None
            self.get_logger().error(f"Create AprilTag detector failed: {exception_error}")

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

    def image_callback(self, image_message: ImageMessageType) -> None:
        """
        Receive, decode, and detect AprilTags.
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

            if self.use_undistort:
                image_frame = self._undistort_image_if_possible(image_frame)

            detection_result = self.detect_apriltags(image_frame)
            self.latest_detection_result = detection_result

            display_image = image_frame.copy()
            self._draw_apriltag_detection_result(display_image, detection_result)
            self._draw_overlay_text(display_image, detection_result)
            self._publish_detection_log_if_needed(detection_result)
            self._publish_debug_image(display_image)

            if self.show_debug_window:
                cv.imshow(self.window_name, display_image)

        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().error(f"Image callback error: {exception_error}")

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

            return self.cv_bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
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

    def _update_image_timing_statistics(self, image_message: ImageMessageType) -> None:
        """
        Update image FPS and delay statistics.
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

    def _get_scaled_camera_params(self, frame_width: int, frame_height: int) -> Optional[Tuple[float, float, float, float]]:
        """
        Get camera intrinsics scaled to current processing image size.
        """
        if self.use_camera_yaml and self.camera_calibration.is_valid:
            source_width = max(int(self.camera_calibration.image_width), 1)
            source_height = max(int(self.camera_calibration.image_height), 1)

            scale_x = float(frame_width) / float(source_width)
            scale_y = float(frame_height) / float(source_height)

            return (
                self.camera_calibration.fx * scale_x,
                self.camera_calibration.fy * scale_y,
                self.camera_calibration.cx * scale_x,
                self.camera_calibration.cy * scale_y,
            )

        if self.camera_fx > 0.0 and self.camera_fy > 0.0:
            return (
                float(self.camera_fx),
                float(self.camera_fy),
                float(self.camera_cx),
                float(self.camera_cy),
            )

        return None

    def _get_scaled_camera_matrix_and_distortion(self, frame_width: int, frame_height: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get scaled camera matrix and distortion coefficients.
        """
        if not (self.use_camera_yaml and self.camera_calibration.is_valid):
            return None

        camera_params = self._get_scaled_camera_params(frame_width, frame_height)
        if camera_params is None:
            return None

        fx, fy, cx, cy = camera_params
        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        distortion_coefficients = np.array(
            self.camera_calibration.distortion_coefficients,
            dtype=np.float64,
        ).reshape(-1, 1)
        return camera_matrix, distortion_coefficients

    def _get_camera_matrix_and_distortion_for_projection(
        self,
        frame_width: int,
        frame_height: int,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get camera matrix and distortion coefficients for local-axis projection.
        """
        camera_params = self._get_scaled_camera_params(frame_width, frame_height)
        if camera_params is None:
            return None

        fx, fy, cx, cy = camera_params
        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        # If image has already been undistorted, do not apply distortion again during projection.
        if self.use_undistort:
            return camera_matrix, np.zeros((5, 1), dtype=np.float64)

        if self.use_camera_yaml and self.camera_calibration.is_valid and self.camera_calibration.distortion_coefficients:
            distortion_coefficients = np.array(
                self.camera_calibration.distortion_coefficients,
                dtype=np.float64,
            ).reshape(-1, 1)
        else:
            distortion_coefficients = np.zeros((5, 1), dtype=np.float64)

        return camera_matrix, distortion_coefficients

    def _undistort_image_if_possible(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Undistort image when distortion coefficients are available.
        """
        frame_height, frame_width = image_frame.shape[:2]
        matrix_and_distortion = self._get_scaled_camera_matrix_and_distortion(frame_width, frame_height)
        if matrix_and_distortion is None:
            return image_frame

        camera_matrix, distortion_coefficients = matrix_and_distortion
        if distortion_coefficients.size <= 0:
            return image_frame

        try:
            return cv.undistort(image_frame, camera_matrix, distortion_coefficients)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Undistort image failed: {exception_error}")
            return image_frame

    def detect_apriltags(self, image_frame: np.ndarray) -> AprilTagDetectionResult:
        """
        Detect AprilTags.
        """
        if self.apriltag_detector is None:
            return AprilTagDetectionResult(False)

        frame_height, frame_width = image_frame.shape[:2]
        gray_image = cv.cvtColor(image_frame, cv.COLOR_BGR2GRAY)

        camera_params = self._get_scaled_camera_params(frame_width, frame_height)
        should_estimate_pose = self.estimate_tag_pose and camera_params is not None

        if self.estimate_tag_pose and camera_params is None:
            current_time_sec = time.monotonic()
            if current_time_sec - self.last_pose_warning_time_sec >= 2.0:
                self.get_logger().warn(
                    "Pose estimation is enabled, but no valid camera intrinsics are available. "
                    "Check camera_yaml_path or manual camera_fx/camera_fy/camera_cx/camera_cy."
                )
                self.last_pose_warning_time_sec = current_time_sec

        try:
            detected_tags = self.apriltag_detector.detect(
                gray_image,
                estimate_tag_pose=should_estimate_pose,
                camera_params=camera_params,
                tag_size=float(self.tag_size),
            )
        except TypeError:
            detected_tags = self.apriltag_detector.detect(
                gray_image,
                should_estimate_pose,
                camera_params,
                float(self.tag_size),
            )
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"AprilTag detection failed: {exception_error}")
            return AprilTagDetectionResult(False)

        if not detected_tags:
            return AprilTagDetectionResult(False)

        tag_infos: List[AprilTagInfo] = []
        for detected_tag in detected_tags:
            center_x, center_y = detected_tag.center
            corners = tuple(
                (int(round(corner[0])), int(round(corner[1])))
                for corner in np.array(detected_tag.corners).reshape(-1, 2)
            )

            pose_x = pose_y = pose_z = None
            pose_roll_deg = pose_pitch_deg = pose_yaw_deg = None
            pose_translation_vector = None
            pose_rotation_matrix = None
            pose_t = getattr(detected_tag, "pose_t", None)
            pose_R = getattr(detected_tag, "pose_R", None)

            if pose_t is not None:
                pose_t_array = np.asarray(pose_t, dtype=np.float64).reshape(-1)
                if pose_t_array.size >= 3:
                    pose_x = float(pose_t_array[0])
                    pose_y = float(pose_t_array[1])
                    pose_z = float(pose_t_array[2])
                    pose_translation_vector = (pose_x, pose_y, pose_z)

            if pose_R is not None:
                try:
                    pose_R_array = np.asarray(pose_R, dtype=np.float64).reshape(3, 3)
                    pose_roll_deg, pose_pitch_deg, pose_yaw_deg = rotation_matrix_to_euler_degrees(pose_R_array)
                    pose_rotation_matrix = tuple(
                        tuple(float(pose_R_array[row_index, column_index]) for column_index in range(3))
                        for row_index in range(3)
                    )
                except Exception:
                    pass

            tag_infos.append(
                AprilTagInfo(
                    tag_id=int(detected_tag.tag_id),
                    center_x=float(center_x),
                    center_y=float(center_y),
                    corners=corners,
                    hamming=int(getattr(detected_tag, "hamming", -1)),
                    decision_margin=float(getattr(detected_tag, "decision_margin", 0.0)),
                    pose_x=pose_x,
                    pose_y=pose_y,
                    pose_z=pose_z,
                    pose_roll_deg=pose_roll_deg,
                    pose_pitch_deg=pose_pitch_deg,
                    pose_yaw_deg=pose_yaw_deg,
                    pose_translation_vector=pose_translation_vector,
                    pose_rotation_matrix=pose_rotation_matrix,
                )
            )

        tag_infos.sort(key=lambda tag_info: tag_info.tag_id)
        main_tag = self._select_main_tag(tag_infos, frame_width, frame_height)
        return AprilTagDetectionResult(
            is_tag_detected=True,
            tag_count=len(tag_infos),
            main_tag=main_tag,
            all_tags=tuple(tag_infos),
        )

    def _select_main_tag(
        self,
        tag_infos: List[AprilTagInfo],
        frame_width: int,
        frame_height: int,
    ) -> Optional[AprilTagInfo]:
        """
        Select main AprilTag.
        """
        if not tag_infos:
            return None

        if self.tag_select_strategy == "nearest_center":
            image_center_x = frame_width * 0.5
            image_center_y = frame_height * 0.5
            return min(
                tag_infos,
                key=lambda tag_info: (
                    (tag_info.center_x - image_center_x) ** 2
                    + (tag_info.center_y - image_center_y) ** 2
                ),
            )

        if self.tag_select_strategy == "largest":
            return max(tag_infos, key=self._calculate_tag_polygon_area)

        return min(tag_infos, key=lambda tag_info: tag_info.tag_id)

    @staticmethod
    def _calculate_tag_polygon_area(tag_info: AprilTagInfo) -> float:
        """
        Calculate tag polygon area.
        """
        if len(tag_info.corners) < 4:
            return 0.0
        points = np.array(tag_info.corners, dtype=np.float32)
        return float(abs(cv.contourArea(points)))

    def _draw_apriltag_detection_result(
        self,
        display_image: np.ndarray,
        detection_result: AprilTagDetectionResult,
    ) -> None:
        """
        Draw AprilTag detection result.
        """
        self._draw_camera_coordinate_axes(display_image)

        if not detection_result.is_tag_detected:
            return

        for tag_info in detection_result.all_tags:
            if len(tag_info.corners) >= 4:
                polygon_points = np.array(tag_info.corners, dtype=np.int32).reshape((-1, 1, 2))
                line_color = (0, 255, 0)
                if detection_result.main_tag is not None and tag_info.tag_id == detection_result.main_tag.tag_id:
                    line_color = (255, 0, 255)
                cv.polylines(display_image, [polygon_points], True, line_color, 2)

                for corner_index, corner in enumerate(tag_info.corners):
                    cv.circle(display_image, corner, 3, (0, 0, 255), -1)
                    cv.putText(
                        display_image,
                        str(corner_index),
                        (corner[0] + 3, corner[1] + 10),
                        cv.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                        cv.LINE_AA,
                    )

            center_point = (int(round(tag_info.center_x)), int(round(tag_info.center_y)))
            cv.circle(display_image, center_point, 4, (0, 255, 255), -1)
            cv.putText(
                display_image,
                f"id:{tag_info.tag_id}",
                (center_point[0] + 6, max(15, center_point[1] - 8)),
                cv.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 255, 255),
                1,
                cv.LINE_AA,
            )

            if tag_info.pose_z is not None:
                cv.putText(
                    display_image,
                    f"Z:{tag_info.pose_z:.2f}m",
                    (center_point[0] + 6, center_point[1] + 14),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (0, 255, 255),
                    1,
                    cv.LINE_AA,
                )

            should_draw_this_tag_axes = (
                self.draw_tag_axes
                and (
                    self.draw_axes_for_all_tags
                    or (
                        detection_result.main_tag is not None
                        and tag_info.tag_id == detection_result.main_tag.tag_id
                    )
                )
            )
            if should_draw_this_tag_axes:
                self._draw_tag_local_coordinate_axes(display_image, tag_info)

    def _get_camera_axes_origin_pixel(self, frame_width: int, frame_height: int) -> Tuple[int, int]:
        """
        Get the camera coordinate frame origin in image pixels.

        Prefer principal point cx/cy from camera.yaml or manual intrinsics; otherwise use image center.
        """
        camera_params = self._get_scaled_camera_params(frame_width, frame_height)
        if camera_params is not None:
            _, _, cx, cy = camera_params
            origin_x = int(round(float(cx)))
            origin_y = int(round(float(cy)))
        else:
            origin_x = frame_width // 2
            origin_y = frame_height // 2

        origin_x = max(0, min(frame_width - 1, origin_x))
        origin_y = max(0, min(frame_height - 1, origin_y))
        return origin_x, origin_y

    def _draw_camera_coordinate_axes(self, display_image: np.ndarray) -> None:
        """
        Draw the camera coordinate frame on the image, replacing the old center cross.

        Camera frame convention:
            positive to image right, red
            positive to image down, green
            positive forward from camera, blue dot/ring
        """
        if not self.draw_camera_axes:
            return

        frame_height, frame_width = display_image.shape[:2]
        origin_x, origin_y = self._get_camera_axes_origin_pixel(frame_width, frame_height)

        axis_length = max(10, int(self.camera_axis_pixel_length))
        axis_thickness = max(1, int(self.camera_axis_thickness))
        label_font_scale = max(0.1, float(self.camera_axis_label_font_scale))
        label_thickness = max(1, axis_thickness - 1)

        # Keep arrows inside the image as much as possible.
        x_axis_end = (min(frame_width - 1, origin_x + axis_length), origin_y)
        y_axis_end = (origin_x, min(frame_height - 1, origin_y + axis_length))

        # X/Y are the camera axes projected on the image plane.
        cv.arrowedLine(
            display_image,
            (origin_x, origin_y),
            x_axis_end,
            (0, 0, 255),
            axis_thickness,
            tipLength=0.25,
        )
        cv.arrowedLine(
            display_image,
            (origin_x, origin_y),
            y_axis_end,
            (0, 255, 0),
            axis_thickness,
            tipLength=0.25,
        )

        # The Z axis points forward along the optical axis, projected to the principal point.
        z_radius = max(4, axis_thickness * 3)
        cv.circle(display_image, (origin_x, origin_y), z_radius, (255, 0, 0), axis_thickness)
        cv.circle(display_image, (origin_x, origin_y), max(2, axis_thickness), (255, 0, 0), -1)

        cv.putText(
            display_image,
            "Cam",
            (max(0, origin_x + 4), max(12, origin_y - 8)),
            cv.FONT_HERSHEY_SIMPLEX,
            label_font_scale,
            (255, 255, 0),
            label_thickness,
            cv.LINE_AA,
        )
        cv.putText(
            display_image,
            "X+",
            (x_axis_end[0] + 4 if x_axis_end[0] + 25 < frame_width else max(0, x_axis_end[0] - 28), x_axis_end[1] - 4),
            cv.FONT_HERSHEY_SIMPLEX,
            label_font_scale,
            (0, 0, 255),
            label_thickness,
            cv.LINE_AA,
        )
        cv.putText(
            display_image,
            "Y+",
            (y_axis_end[0] + 4, y_axis_end[1] + 14 if y_axis_end[1] + 18 < frame_height else max(12, y_axis_end[1] - 6)),
            cv.FONT_HERSHEY_SIMPLEX,
            label_font_scale,
            (0, 255, 0),
            label_thickness,
            cv.LINE_AA,
        )
        cv.putText(
            display_image,
            "Z+",
            (max(0, origin_x - 18), min(frame_height - 4, origin_y + z_radius + 16)),
            cv.FONT_HERSHEY_SIMPLEX,
            label_font_scale,
            (255, 0, 0),
            label_thickness,
            cv.LINE_AA,
        )

    def _draw_tag_local_coordinate_axes(self, display_image: np.ndarray, tag_info: AprilTagInfo) -> None:
        """
        Draw the local coordinate frame of an AprilTag on the image.

        Color convention:
            X: red
            Y: green
            Z: blue
        """
        if not self.draw_tag_axes:
            return
        if tag_info.pose_translation_vector is None or tag_info.pose_rotation_matrix is None:
            return

        frame_height, frame_width = display_image.shape[:2]
        matrix_and_distortion = self._get_camera_matrix_and_distortion_for_projection(frame_width, frame_height)
        if matrix_and_distortion is None:
            return

        camera_matrix, distortion_coefficients = matrix_and_distortion
        rotation_matrix = np.array(tag_info.pose_rotation_matrix, dtype=np.float64).reshape(3, 3)
        translation_vector = np.array(tag_info.pose_translation_vector, dtype=np.float64).reshape(3, 1)

        try:
            rotation_vector, _ = cv.Rodrigues(rotation_matrix)
        except Exception:
            return

        axis_length = self.tag_axis_length
        if axis_length <= 0.0:
            # Default axis length is 60% of tag size.
            axis_length = float(self.tag_size) * 0.60

        object_points = np.array(
            [
                [0.0, 0.0, 0.0],
                [axis_length, 0.0, 0.0],
                [0.0, axis_length, 0.0],
                [0.0, 0.0, axis_length],
            ],
            dtype=np.float64,
        )

        try:
            projected_points, _ = cv.projectPoints(
                object_points,
                rotation_vector,
                translation_vector,
                camera_matrix,
                distortion_coefficients,
            )
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Project AprilTag local axes failed: {exception_error}")
            return

        projected_points = projected_points.reshape(-1, 2)
        if projected_points.shape[0] < 4:
            return

        origin_point = self._point_to_int_tuple(projected_points[0])
        x_axis_point = self._point_to_int_tuple(projected_points[1])
        y_axis_point = self._point_to_int_tuple(projected_points[2])
        z_axis_point = self._point_to_int_tuple(projected_points[3])

        axis_thickness = max(1, int(self.tag_axis_thickness))
        cv.arrowedLine(display_image, origin_point, x_axis_point, (0, 0, 255), axis_thickness, tipLength=0.25)
        cv.arrowedLine(display_image, origin_point, y_axis_point, (0, 255, 0), axis_thickness, tipLength=0.25)
        cv.arrowedLine(display_image, origin_point, z_axis_point, (255, 0, 0), axis_thickness, tipLength=0.25)

        label_thickness = max(1, axis_thickness - 1)
        cv.putText(display_image, "X", x_axis_point, cv.FONT_HERSHEY_SIMPLEX, self.axis_label_font_scale, (0, 0, 255), label_thickness, cv.LINE_AA)
        cv.putText(display_image, "Y", y_axis_point, cv.FONT_HERSHEY_SIMPLEX, self.axis_label_font_scale, (0, 255, 0), label_thickness, cv.LINE_AA)
        cv.putText(display_image, "Z", z_axis_point, cv.FONT_HERSHEY_SIMPLEX, self.axis_label_font_scale, (255, 0, 0), label_thickness, cv.LINE_AA)

    @staticmethod
    def _point_to_int_tuple(point_value: np.ndarray) -> Tuple[int, int]:
        """
        Convert a projected point to integer pixel coordinates.
        """
        return (int(round(float(point_value[0]))), int(round(float(point_value[1]))))

    def _draw_overlay_text(
        self,
        display_image: np.ndarray,
        detection_result: AprilTagDetectionResult,
    ) -> None:
        """
        Draw overlay text.
        """
        frame_rate_value = self.header_based_frame_rate if self.use_header_stamp_for_fps else self.receive_based_frame_rate

        main_tag_text = "main: None"
        pose_text = "pose: None"
        if detection_result.main_tag is not None:
            main_tag_text = (
                f"main id:{detection_result.main_tag.tag_id} "
                f"u:{detection_result.main_tag.center_x:.0f} "
                f"v:{detection_result.main_tag.center_y:.0f}"
            )
            if detection_result.main_tag.pose_x is not None:
                pose_text = (
                    f"XYZ: {detection_result.main_tag.pose_x:.3f}, "
                    f"{detection_result.main_tag.pose_y:.3f}, "
                    f"{detection_result.main_tag.pose_z:.3f} m"
                )

        calibration_text = "calib: manual/none"
        if self.use_camera_yaml and self.camera_calibration.is_valid:
            calibration_text = (
                f"calib: {self.camera_calibration.image_width}x{self.camera_calibration.image_height}"
            )

        overlay_lines = [
            "mode: apriltag detection + 3D pose",
            f"tags: {detection_result.tag_count}",
            f"fps: {frame_rate_value:.1f}",
            main_tag_text,
            pose_text,
            f"tag_size: {self.tag_size:.3f}m | family: {self.tag_family}",
            f"tag axes: {'on' if self.draw_tag_axes else 'off'} | len: {self.tag_axis_length if self.tag_axis_length > 0.0 else self.tag_size * 0.60:.3f}m",
            f"camera axes: {'on' if self.draw_camera_axes else 'off'}",
            calibration_text,
            "q/ESC quit",
        ]

        if self.show_image_stamp_delay:
            overlay_lines.insert(3, f"delay: {self.image_stamp_delay_ms:.1f}ms")

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

    def _publish_detection_log_if_needed(self, detection_result: AprilTagDetectionResult) -> None:
        """
        Print detection log periodically.
        """
        if not self.print_detection_log:
            return

        current_time_sec = time.monotonic()
        if current_time_sec - self.last_detection_log_time_sec < self.detection_log_interval_sec:
            return

        self.last_detection_log_time_sec = current_time_sec

        if detection_result.main_tag is not None:
            pose_text = ""
            if detection_result.main_tag.pose_x is not None:
                pose_text = (
                    f", xyz=({detection_result.main_tag.pose_x:.3f}, "
                    f"{detection_result.main_tag.pose_y:.3f}, "
                    f"{detection_result.main_tag.pose_z:.3f})m"
                )
                if detection_result.main_tag.pose_yaw_deg is not None:
                    pose_text += (
                        f", rpy=({detection_result.main_tag.pose_roll_deg:.1f}, "
                        f"{detection_result.main_tag.pose_pitch_deg:.1f}, "
                        f"{detection_result.main_tag.pose_yaw_deg:.1f})deg"
                    )
            self.get_logger().info(
                f"Detected AprilTags: {detection_result.tag_count}, "
                f"main_id={detection_result.main_tag.tag_id}, "
                f"center=({detection_result.main_tag.center_x:.1f}, {detection_result.main_tag.center_y:.1f})"
                f"{pose_text}, family={self.tag_family}"
            )
        else:
            self.get_logger().info(f"Detected AprilTags: 0, family={self.tag_family}")

    def _publish_debug_image(self, display_image: np.ndarray) -> None:
        """
        Publish debug image.
        """
        if not self.publish_debug_image:
            return

        try:
            debug_image_message = self.cv_bridge.cv2_to_imgmsg(display_image, encoding="bgr8")
            self.debug_image_publisher.publish(debug_image_message)
        except Exception as exception_error:  # pylint: disable=broad-except
            self.get_logger().warn(f"Publish debug image failed: {exception_error}")

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
            self.get_logger().info(f"Force exiting detect_apriltag_node: {reason}")
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

        detector_parameter_names = {
            "tag_family",
            "detector_nthreads",
            "quad_decimate",
            "quad_sigma",
            "refine_edges",
            "decode_sharpening",
            "detector_debug",
        }

        calibration_parameter_names = {
            "camera_yaml_path",
            "use_camera_yaml",
        }

        try:
            old_show_debug_window = self.show_debug_window
            self._load_ros_parameters()

            if parameter_names & calibration_parameter_names:
                self._load_camera_calibration_if_needed()

            if parameter_names & detector_parameter_names:
                self._recreate_apriltag_detector()

            if self.show_debug_window and not old_show_debug_window:
                self._initialize_debug_window()
                if self.keyboard_poll_timer is None:
                    self.keyboard_poll_timer = self.create_timer(0.05, self._keyboard_poll_timer_callback)
        except Exception as exception_error:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception_error))

        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        """
        Destroy node.

        """
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    Node entry function.
    """
    rclpy.init(args=args)
    detect_apriltag_node = DetectAprilTagNode()

    try:
        while rclpy.ok() and not detect_apriltag_node.is_shutdown_requested:
            rclpy.spin_once(detect_apriltag_node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            detect_apriltag_node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
