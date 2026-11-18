#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Standalone follower for bbox feedback published on /tracking_bbox.

This node does not start OpenCV KCF tracking and does not talk to Dify. It only
subscribes to a bbox topic, keeps the target centered, and optionally follows
target size with chassis linear velocity.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Bool


DEFAULT_NODE_NAME = "tracking_bbox_follow_node"
MINIMUM_TIMER_RATE_HZ = 1.0

DYNAMIC_PARAMETER_NAMES = frozenset(
    {
        "enable_follow",
        "enable_ptz_follow",
        "enable_body_turn_follow",
        "enable_distance_follow",
        "enable_joy_override",
        "image_width",
        "image_height",
        "tracking_confidence_threshold",
        "target_stale_timeout_sec",
        "follow_rate_hz",
        "motion_log_interval_sec",
        "pixel_tolerance",
        "kp_x",
        "kd_x",
        "kp_y",
        "kd_y",
        "max_step_xy",
        "sign_x",
        "sign_y",
        "horizontal_min_deg",
        "horizontal_max_deg",
        "pitch_min_deg",
        "pitch_max_deg",
        "initial_horizontal_deg",
        "initial_pitch_deg",
        "publish_initial_ptz_command",
        "initial_publish_count",
        "initial_publish_interval_sec",
        "body_kp",
        "body_kd",
        "body_tolerance_deg",
        "servo1_engage_deg",
        "max_angular_z",
        "body_dir",
        "body_ff_gain",
        "fixed_camera_pixel_tolerance",
        "fixed_camera_engage_px",
        "fixed_camera_body_kp",
        "fixed_camera_body_kd",
        "fixed_camera_body_dir",
        "target_bbox_size_px",
        "auto_set_target_bbox_size",
        "distance_tolerance_ratio",
        "distance_kp",
        "distance_ki",
        "distance_kd",
        "distance_integral_limit",
        "max_linear_x",
        "linear_dir",
        "print_cmd_vel",
        "print_interval_sec",
        "enable_rqt_dynamic_parameters",
    }
)

RESTART_REQUIRED_PARAMETER_NAMES = frozenset(
    {
        "tracking_bbox_topic",
        "tracking_bbox_msg_type",
        "cmd_vel_topic",
        "ptz_command_topic",
        "joy_state_topic",
    }
)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a numeric value."""
    return max(float(lower), min(float(upper), float(value)))


def to_bool(value: Any, default: bool) -> bool:
    """Convert ROS launch string/bool/int values to bool."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "y", "on"):
            return True
        if normalized in ("false", "0", "no", "n", "off"):
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    return default


class TrackingBboxFollowNode(Node):
    """Follow a target from /tracking_bbox without Dify or tracker lifecycle."""

    def __init__(self) -> None:
        super().__init__(DEFAULT_NODE_NAME)

        self._bbox_lock = threading.Lock()
        self._latest_bbox = self._empty_bbox_state()

        self._bbox_subscription = None
        self._bbox_wait_logged = False
        self._subscription_retry_timer = None

        self._joy_is_active = False
        self._previous_joy_is_active = False

        self._servo1_angle = 0.0
        self._servo2_angle = 0.0
        self._previous_err_x = 0.0
        self._previous_err_y = 0.0
        self._previous_control_time = 0.0
        self._last_body_angular_z = 0.0
        self._body_correcting = False
        self._previous_body_error = 0.0
        self._previous_distance_error = 0.0
        self._distance_integral = 0.0
        self._target_bbox_size = 0.0
        self._target_bbox_size_initialized = False
        self._last_motion_log_time = 0.0
        self._last_target_lost_log_time = 0.0

        self._declare_parameters()
        self._load_parameters()

        self._servo1_angle = self.initial_horizontal_deg
        self._servo2_angle = self.initial_pitch_deg
        self._target_bbox_size = max(0.0, self.target_bbox_size_px)
        self._target_bbox_size_initialized = self._target_bbox_size > 0.0

        self._cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._bbox_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, self._cmd_vel_qos
        )
        self._ptz_pub = self.create_publisher(Vector3, self.ptz_command_topic, 10)
        self._joy_sub = self.create_subscription(
            Bool, self.joy_state_topic, self._joy_state_callback, 10
        )

        self._control_timer = None
        self._rebuild_control_timer(log=False)
        self._subscription_retry_timer = self.create_timer(
            1.0, self._ensure_tracking_bbox_subscription
        )
        self._ensure_tracking_bbox_subscription()

        self._initial_ptz_timer = None
        self._initial_ptz_remaining = 0
        if self.enable_ptz_follow and self.publish_initial_ptz_command:
            self._start_initial_ptz_publish()

        self._parameter_callback = self.add_on_set_parameters_callback(
            self._on_parameter_update
        )
        self.get_logger().info(
            "Tracking bbox follower started. bbox_topic=%s, cmd_vel_topic=%s, "
            "ptz=%s, body_turn=%s, distance_follow=%s"
            % (
                self.tracking_bbox_topic,
                self.cmd_vel_topic,
                self.enable_ptz_follow,
                self.enable_body_turn_follow,
                self.enable_distance_follow,
            )
        )

    @staticmethod
    def _empty_bbox_state() -> dict:
        return {
            "valid": False,
            "stamp": 0.0,
            "x1": 0,
            "y1": 0,
            "x2": 0,
            "y2": 0,
            "cx": 0.0,
            "cy": 0.0,
            "width": 0.0,
            "height": 0.0,
            "image_width": 640,
            "image_height": 480,
            "confidence": 0.0,
        }

    def _declare_parameters(self) -> None:
        self.declare_parameter("tracking_bbox_topic", "/tracking_bbox")
        self.declare_parameter("tracking_bbox_msg_type", "auto")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("joy_state_topic", "/JoyState")

        self.declare_parameter("enable_follow", True)
        self.declare_parameter("enable_ptz_follow", True)
        self.declare_parameter("enable_body_turn_follow", True)
        self.declare_parameter("enable_distance_follow", True)
        self.declare_parameter("enable_joy_override", True)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("tracking_confidence_threshold", 0.1)
        self.declare_parameter("target_stale_timeout_sec", 1.2)
        self.declare_parameter("follow_rate_hz", 10.0)
        self.declare_parameter("motion_log_interval_sec", 0.0)

        self.declare_parameter("horizontal_min_deg", -90.0)
        self.declare_parameter("horizontal_max_deg", 90.0)
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 20.0)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)

        self.declare_parameter("pixel_tolerance", 15.0)
        self.declare_parameter("kp_x", 0.6)
        self.declare_parameter("kd_x", 0.15)
        self.declare_parameter("kp_y", 0.6)
        self.declare_parameter("kd_y", 0.15)
        self.declare_parameter("max_step_xy", 5.0)
        self.declare_parameter("sign_x", 1.0)
        self.declare_parameter("sign_y", -1.0)

        self.declare_parameter("body_kp", 0.03)
        self.declare_parameter("body_kd", 0.0)
        self.declare_parameter("body_tolerance_deg", 3.0)
        self.declare_parameter("servo1_engage_deg", 5.0)
        self.declare_parameter("max_angular_z", 0.6)
        self.declare_parameter("body_dir", -1.0)
        self.declare_parameter("body_ff_gain", 1.0)

        self.declare_parameter("fixed_camera_pixel_tolerance", 15.0)
        self.declare_parameter("fixed_camera_engage_px", 30.0)
        self.declare_parameter("fixed_camera_body_kp", 0.6)
        self.declare_parameter("fixed_camera_body_kd", 0.15)
        self.declare_parameter("fixed_camera_body_dir", -1.0)

        self.declare_parameter("target_bbox_size_px", 0.0)
        self.declare_parameter("auto_set_target_bbox_size", True)
        self.declare_parameter("distance_tolerance_ratio", 0.06)
        self.declare_parameter("distance_kp", 0.18)
        self.declare_parameter("distance_ki", 0.0)
        self.declare_parameter("distance_kd", 0.02)
        self.declare_parameter("distance_integral_limit", 0.5)
        self.declare_parameter("max_linear_x", 0.12)
        self.declare_parameter("linear_dir", 1.0)

        self.declare_parameter("print_cmd_vel", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

    def _load_parameters(self, overrides: Optional[dict[str, Any]] = None) -> None:
        overrides = overrides or {}

        def get_param(parameter_name: str) -> Any:
            if parameter_name in overrides:
                return overrides[parameter_name]
            return self.get_parameter(parameter_name).value

        self.tracking_bbox_topic = str(
            get_param("tracking_bbox_topic")
        )
        self.tracking_bbox_msg_type = str(
            get_param("tracking_bbox_msg_type")
        )
        self.cmd_vel_topic = str(get_param("cmd_vel_topic"))
        self.ptz_command_topic = str(get_param("ptz_command_topic"))
        self.joy_state_topic = str(get_param("joy_state_topic"))

        self.enable_follow = to_bool(get_param("enable_follow"), True)
        self.enable_ptz_follow = to_bool(
            get_param("enable_ptz_follow"), True
        )
        self.enable_body_turn_follow = to_bool(
            get_param("enable_body_turn_follow"), True
        )
        self.enable_distance_follow = to_bool(
            get_param("enable_distance_follow"), True
        )
        self.enable_joy_override = to_bool(
            get_param("enable_joy_override"), True
        )
        self.image_width = max(1, int(get_param("image_width")))
        self.image_height = max(1, int(get_param("image_height")))
        self.tracking_confidence_threshold = float(
            get_param("tracking_confidence_threshold")
        )
        self.target_stale_timeout_sec = max(
            0.05, float(get_param("target_stale_timeout_sec"))
        )
        self.follow_rate_hz = max(
            MINIMUM_TIMER_RATE_HZ,
            float(get_param("follow_rate_hz")),
        )
        self.motion_log_interval_sec = max(
            0.0, float(get_param("motion_log_interval_sec"))
        )

        self.horizontal_min_deg = float(
            get_param("horizontal_min_deg")
        )
        self.horizontal_max_deg = float(
            get_param("horizontal_max_deg")
        )
        self.pitch_min_deg = float(get_param("pitch_min_deg"))
        self.pitch_max_deg = float(get_param("pitch_max_deg"))
        self.initial_horizontal_deg = float(
            get_param("initial_horizontal_deg")
        )
        self.initial_pitch_deg = float(get_param("initial_pitch_deg"))
        self.publish_initial_ptz_command = to_bool(
            get_param("publish_initial_ptz_command"), True
        )
        self.initial_publish_count = max(
            0, int(get_param("initial_publish_count"))
        )
        self.initial_publish_interval_sec = max(
            0.01, float(get_param("initial_publish_interval_sec"))
        )

        self.pixel_tolerance = max(
            0.0, float(get_param("pixel_tolerance"))
        )
        self.ptz_kp_x = max(0.0, float(get_param("kp_x")))
        self.ptz_kd_x = max(0.0, float(get_param("kd_x")))
        self.ptz_kp_y = max(0.0, float(get_param("kp_y")))
        self.ptz_kd_y = max(0.0, float(get_param("kd_y")))
        self.max_ptz_step = max(0.0, float(get_param("max_step_xy")))
        self.ptz_sign_x = clamp(float(get_param("sign_x")), -1.0, 1.0)
        self.ptz_sign_y = clamp(float(get_param("sign_y")), -1.0, 1.0)

        self.body_kp = max(0.0, float(get_param("body_kp")))
        self.body_kd = max(0.0, float(get_param("body_kd")))
        self.body_tolerance_deg = max(
            0.0, float(get_param("body_tolerance_deg"))
        )
        self.servo1_engage_deg = max(
            0.0, float(get_param("servo1_engage_deg"))
        )
        self.max_angular_z = max(
            0.0, float(get_param("max_angular_z"))
        )
        self.body_dir = clamp(float(get_param("body_dir")), -1.0, 1.0)
        self.body_ff_gain = clamp(
            float(get_param("body_ff_gain")), 0.0, 3.0
        )

        self.fixed_camera_pixel_tolerance = max(
            0.0, float(get_param("fixed_camera_pixel_tolerance"))
        )
        self.fixed_camera_engage_px = max(
            0.0, float(get_param("fixed_camera_engage_px"))
        )
        self.fixed_camera_body_kp = max(
            0.0, float(get_param("fixed_camera_body_kp"))
        )
        self.fixed_camera_body_kd = max(
            0.0, float(get_param("fixed_camera_body_kd"))
        )
        self.fixed_camera_body_dir = clamp(
            float(get_param("fixed_camera_body_dir")), -1.0, 1.0
        )

        self.target_bbox_size_px = max(
            0.0, float(get_param("target_bbox_size_px"))
        )
        self.auto_set_target_bbox_size = to_bool(
            get_param("auto_set_target_bbox_size"), True
        )
        self.distance_tolerance_ratio = max(
            0.0, float(get_param("distance_tolerance_ratio"))
        )
        self.distance_kp = max(0.0, float(get_param("distance_kp")))
        self.distance_ki = max(0.0, float(get_param("distance_ki")))
        self.distance_kd = max(0.0, float(get_param("distance_kd")))
        self.distance_integral_limit = max(
            0.0, float(get_param("distance_integral_limit"))
        )
        self.max_linear_x = max(0.0, float(get_param("max_linear_x")))
        self.linear_dir = clamp(
            float(get_param("linear_dir")), -1.0, 1.0
        )

        self.print_cmd_vel = to_bool(get_param("print_cmd_vel"), False)
        self.print_interval_sec = max(
            0.0, float(get_param("print_interval_sec"))
        )
        self.enable_rqt_dynamic_parameters = to_bool(
            get_param("enable_rqt_dynamic_parameters"), True
        )

    def _discover_topic_types(self, topic_name: str) -> list[str]:
        try:
            for name, types in self.get_topic_names_and_types():
                if name == topic_name:
                    return list(types)
        except Exception as error:
            self.get_logger().warn("Failed to query topic types for %s: %s" % (topic_name, error))
        return []

    def _candidate_bbox_msg_types(self) -> list[str]:
        configured_type = self.tracking_bbox_msg_type.strip()
        if configured_type and configured_type.lower() not in ("auto", "none"):
            return [configured_type]

        discovered = self._discover_topic_types(self.tracking_bbox_topic)
        preferred = [
            "microros_v2_interface_pkg/msg/TrackingBbox",
            "smart_tracker/msg/TrackBox",
            "std_msgs/msg/Int32MultiArray",
        ]
        ordered = []
        for type_name in preferred + discovered:
            if type_name in discovered and type_name not in ordered:
                ordered.append(type_name)
        for type_name in discovered:
            if type_name not in ordered:
                ordered.append(type_name)
        return ordered

    def _ensure_tracking_bbox_subscription(self) -> None:
        if self._bbox_subscription is not None:
            return

        candidate_types = self._candidate_bbox_msg_types()
        if not candidate_types:
            if not self._bbox_wait_logged:
                self.get_logger().info(
                    "Waiting for %s to discover bbox message type. "
                    "Set tracking_bbox_msg_type if the publisher starts later."
                    % self.tracking_bbox_topic
                )
                self._bbox_wait_logged = True
            return

        last_error = ""
        for type_name in candidate_types:
            try:
                message_type = get_message(type_name)
                self._bbox_subscription = self.create_subscription(
                    message_type,
                    self.tracking_bbox_topic,
                    self._tracking_bbox_callback,
                    self._bbox_qos,
                )
                self.tracking_bbox_msg_type = type_name
                self._bbox_wait_logged = False
                self.get_logger().info(
                    "Subscribed %s, type=%s"
                    % (self.tracking_bbox_topic, self.tracking_bbox_msg_type)
                )
                return
            except Exception as error:
                last_error = "%s: %s" % (type_name, error)

        self.get_logger().warn(
            "Failed to subscribe %s. candidates=%s, last_error=%s"
            % (self.tracking_bbox_topic, candidate_types, last_error or "none")
        )

    def _tracking_bbox_callback(self, message: Any) -> None:
        parsed_state = self._parse_tracking_bbox_message(message)
        if parsed_state is None:
            return

        with self._bbox_lock:
            self._latest_bbox.update(parsed_state)

        if parsed_state["valid"] and self.auto_set_target_bbox_size:
            current_size = self._get_bbox_size(
                parsed_state["width"], parsed_state["height"]
            )
            if not self._target_bbox_size_initialized and current_size > 0.0:
                self._target_bbox_size = current_size
                self._target_bbox_size_initialized = True
                self.get_logger().info(
                    "Target bbox size initialized from first valid bbox: %.1f px"
                    % self._target_bbox_size
                )

    def _parse_tracking_bbox_message(self, message: Any) -> Optional[dict]:
        image_width = self.image_width
        image_height = self.image_height
        confidence = 1.0
        explicit_valid = True
        bbox = None

        if hasattr(message, "bbox"):
            bbox = list(message.bbox)
            confidence = float(getattr(message, "confidence", 1.0))
            image_width = int(getattr(message, "image_width", image_width) or image_width)
            image_height = int(getattr(message, "image_height", image_height) or image_height)
        elif hasattr(message, "data"):
            data = list(message.data)
            if len(data) >= 11:
                explicit_valid = bool(data[0])
                bbox = [data[1], data[2], data[3], data[4]]
                confidence = 1.0 if explicit_valid else 0.0
                image_width = int(data[9]) if int(data[9]) > 0 else image_width
                image_height = int(data[10]) if int(data[10]) > 0 else image_height
            elif len(data) >= 4:
                bbox = data[:4]
            else:
                self.get_logger().warn(
                    "Ignored bbox message with invalid Int32MultiArray length: %d"
                    % len(data)
                )
                return None
        elif all(hasattr(message, name) for name in ("x1", "y1", "x2", "y2")):
            bbox = [message.x1, message.y1, message.x2, message.y2]
            confidence = float(getattr(message, "confidence", 1.0))
            image_width = int(getattr(message, "image_width", image_width) or image_width)
            image_height = int(getattr(message, "image_height", image_height) or image_height)
        else:
            self.get_logger().warn(
                "Ignored bbox message without bbox/data/x1-y1-x2-y2 fields: %s"
                % type(message).__name__
            )
            return None

        if bbox is None or len(bbox) != 4:
            self.get_logger().warn("Ignored bbox message with invalid bbox=%s" % (bbox,))
            return None

        x1, y1, x2, y2 = [int(value) for value in bbox]
        x1, x2 = sorted((x1, x2))
        y1, y2 = sorted((y1, y2))
        width = max(0, x2 - x1)
        height = max(0, y2 - y1)
        valid = (
            explicit_valid
            and confidence >= self.tracking_confidence_threshold
            and width > 0
            and height > 0
        )

        return {
            "valid": valid,
            "stamp": time.monotonic(),
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "cx": float(x1 + width / 2.0),
            "cy": float(y1 + height / 2.0),
            "width": float(width),
            "height": float(height),
            "image_width": max(1, int(image_width)),
            "image_height": max(1, int(image_height)),
            "confidence": confidence,
        }

    def _joy_state_callback(self, joy_message: Bool) -> None:
        self._joy_is_active = bool(joy_message.data)
        if self._joy_is_active and not self._previous_joy_is_active:
            self._reset_follow_controller()
            self._publish_zero_twist()
            self.get_logger().info("Joy override active; autonomous follow paused.")
        self._previous_joy_is_active = self._joy_is_active

    def _read_bbox_state(self) -> dict:
        with self._bbox_lock:
            return dict(self._latest_bbox)

    @staticmethod
    def _get_bbox_size(width: float, height: float) -> float:
        return math.sqrt(max(1.0, float(width)) * max(1.0, float(height)))

    def _reset_follow_controller(self) -> None:
        self._previous_err_x = 0.0
        self._previous_err_y = 0.0
        self._previous_control_time = 0.0
        self._last_body_angular_z = 0.0
        self._body_correcting = False
        self._previous_body_error = 0.0
        self._previous_distance_error = 0.0
        self._distance_integral = 0.0

    def _control_tick(self) -> None:
        if not self.enable_follow:
            self._reset_follow_controller()
            self._publish_zero_twist()
            return

        if self.enable_joy_override and self._joy_is_active:
            self._reset_follow_controller()
            return

        state = self._read_bbox_state()
        now = time.monotonic()
        fresh = (now - float(state["stamp"])) <= self.target_stale_timeout_sec
        if not state["valid"] or not fresh:
            self._reset_follow_controller()
            self._publish_zero_twist()
            self._log_target_lost_if_needed(state, fresh)
            return

        if state["image_width"] <= 0:
            state["image_width"] = self.image_width
        if state["image_height"] <= 0:
            state["image_height"] = self.image_height

        dt = (
            now - self._previous_control_time
            if self._previous_control_time > 0.0
            else 1.0 / max(self.follow_rate_hz, MINIMUM_TIMER_RATE_HZ)
        )
        dt = max(dt, 1e-3)

        if self.enable_ptz_follow:
            self._apply_ptz_follow(state, dt)

        twist = self._compute_chassis_twist(state, dt)
        self._cmd_vel_pub.publish(twist)
        self._log_motion_state(twist, state)
        self._previous_control_time = now

    def _log_target_lost_if_needed(self, state: dict, fresh: bool) -> None:
        now = time.monotonic()
        if now - self._last_target_lost_log_time < 2.0:
            return
        self._last_target_lost_log_time = now
        self.get_logger().warn(
            "No valid bbox; stopping. valid=%s fresh=%s confidence=%.3f"
            % (state["valid"], fresh, float(state.get("confidence", 0.0)))
        )

    def _publish_zero_twist(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _publish_ptz_angle(self, horizontal_angle: float, pitch_angle: float) -> None:
        message = Vector3()
        message.x = float(horizontal_angle)
        message.y = float(pitch_angle)
        message.z = 0.0
        self._ptz_pub.publish(message)

    def _start_initial_ptz_publish(self) -> None:
        if self.initial_publish_count <= 0:
            return
        self._initial_ptz_remaining = self.initial_publish_count
        self._initial_ptz_timer = self.create_timer(
            self.initial_publish_interval_sec, self._publish_initial_ptz_tick
        )

    def _publish_initial_ptz_tick(self) -> None:
        if self._initial_ptz_remaining <= 0:
            if self._initial_ptz_timer is not None:
                self.destroy_timer(self._initial_ptz_timer)
                self._initial_ptz_timer = None
            return
        self._publish_ptz_angle(self._servo1_angle, self._servo2_angle)
        self._initial_ptz_remaining -= 1

    def _apply_ptz_follow(self, state: dict, dt: float) -> None:
        if abs(self._last_body_angular_z) > 1e-6:
            body_yaw_deg = math.degrees(self._last_body_angular_z * dt)
            vor_deg = self.body_ff_gain * self.ptz_sign_x * body_yaw_deg
            self._servo1_angle = clamp(
                self._servo1_angle + vor_deg,
                self.horizontal_min_deg,
                self.horizontal_max_deg,
            )

        image_width = max(1.0, float(state["image_width"]))
        image_height = max(1.0, float(state["image_height"]))
        half_width = image_width * 0.5
        half_height = image_height * 0.5

        error_x = float(state["cx"]) - half_width
        if abs(error_x) <= self.pixel_tolerance:
            self._previous_err_x = 0.0
            step_x = 0.0
        else:
            normalized_x = clamp(error_x / half_width, -1.0, 1.0)
            derivative_x = normalized_x - self._previous_err_x
            output_x = clamp(
                self.ptz_kp_x * normalized_x + self.ptz_kd_x * derivative_x,
                -1.0,
                1.0,
            )
            step_x = output_x * self.max_ptz_step
            self._previous_err_x = normalized_x

        error_y = float(state["cy"]) - half_height
        if abs(error_y) <= self.pixel_tolerance:
            self._previous_err_y = 0.0
            step_y = 0.0
        else:
            normalized_y = clamp(error_y / half_height, -1.0, 1.0)
            derivative_y = normalized_y - self._previous_err_y
            output_y = clamp(
                self.ptz_kp_y * normalized_y + self.ptz_kd_y * derivative_y,
                -1.0,
                1.0,
            )
            step_y = output_y * self.max_ptz_step
            self._previous_err_y = normalized_y

        self._servo1_angle = clamp(
            self._servo1_angle + self.ptz_sign_x * step_x,
            self.horizontal_min_deg,
            self.horizontal_max_deg,
        )
        self._servo2_angle = clamp(
            self._servo2_angle + self.ptz_sign_y * step_y,
            self.pitch_min_deg,
            self.pitch_max_deg,
        )
        self._publish_ptz_angle(self._servo1_angle, self._servo2_angle)

    def _get_pan_error_deg(self) -> float:
        return self._servo1_angle - self.initial_horizontal_deg

    def _compute_ptz_body_turn(self, dt: float) -> float:
        servo_error = self._get_pan_error_deg()

        if abs(servo_error) >= self.servo1_engage_deg:
            self._body_correcting = True
        elif abs(servo_error) <= self.body_tolerance_deg:
            self._body_correcting = False

        if not self._body_correcting:
            self._previous_body_error = 0.0
            return 0.0

        derivative = (servo_error - self._previous_body_error) / max(dt, 1e-3)
        angular_z = self.body_dir * (
            self.body_kp * servo_error + self.body_kd * derivative
        )
        angular_z = clamp(angular_z, -self.max_angular_z, self.max_angular_z)
        self._previous_body_error = servo_error
        return angular_z

    def _compute_fixed_camera_body_turn(self, state: dict) -> float:
        image_width = max(1.0, float(state["image_width"]))
        half_width = image_width * 0.5
        pixel_error = float(state["cx"]) - half_width

        if abs(pixel_error) <= self.fixed_camera_pixel_tolerance:
            self._body_correcting = False
        elif abs(pixel_error) >= self.fixed_camera_engage_px:
            self._body_correcting = True

        if not self._body_correcting:
            self._previous_body_error = 0.0
            return 0.0

        normalized_error = clamp(pixel_error / half_width, -1.0, 1.0)
        derivative = normalized_error - self._previous_body_error
        angular_z = self.fixed_camera_body_dir * (
            self.fixed_camera_body_kp * normalized_error
            + self.fixed_camera_body_kd * derivative
        )
        self._previous_body_error = normalized_error
        return clamp(angular_z, -self.max_angular_z, self.max_angular_z)

    def _compute_distance_follow(self, state: dict, dt: float) -> float:
        if not self.enable_distance_follow:
            self._previous_distance_error = 0.0
            self._distance_integral = 0.0
            return 0.0

        if not self._target_bbox_size_initialized:
            return 0.0

        target_size = max(1.0, float(self._target_bbox_size))
        current_size = self._get_bbox_size(state["width"], state["height"])
        size_error = (target_size - current_size) / target_size
        if abs(size_error) <= self.distance_tolerance_ratio:
            self._previous_distance_error = 0.0
            self._distance_integral = 0.0
            return 0.0

        dt = max(dt, 1e-3)
        self._distance_integral += size_error * dt
        if self.distance_integral_limit > 0.0:
            self._distance_integral = clamp(
                self._distance_integral,
                -self.distance_integral_limit,
                self.distance_integral_limit,
            )

        derivative = (size_error - self._previous_distance_error) / dt
        linear_x = self.linear_dir * (
            self.distance_kp * size_error
            + self.distance_ki * self._distance_integral
            + self.distance_kd * derivative
        )
        self._previous_distance_error = size_error
        return clamp(linear_x, -self.max_linear_x, self.max_linear_x)

    def _compute_chassis_twist(self, state: dict, dt: float) -> Twist:
        twist = Twist()

        if self.enable_body_turn_follow:
            if self.enable_ptz_follow:
                twist.angular.z = self._compute_ptz_body_turn(dt)
            else:
                twist.angular.z = self._compute_fixed_camera_body_turn(state)
            self._last_body_angular_z = float(twist.angular.z)
        else:
            self._last_body_angular_z = 0.0

        twist.linear.x = self._compute_distance_follow(state, dt)
        return twist

    def _log_motion_state(self, twist: Twist, state: dict) -> None:
        if not self.print_cmd_vel and self.motion_log_interval_sec <= 0.0:
            return

        interval = self.print_interval_sec if self.print_cmd_vel else self.motion_log_interval_sec
        if interval <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_motion_log_time < interval:
            return
        self._last_motion_log_time = now
        self.get_logger().info(
            "cmd_vel linear.x=%.3f angular.z=%.3f bbox=(%d,%d)-(%d,%d) "
            "conf=%.3f ptz=(%.1f,%.1f) target_size=%.1f"
            % (
                twist.linear.x,
                twist.angular.z,
                state["x1"],
                state["y1"],
                state["x2"],
                state["y2"],
                float(state.get("confidence", 0.0)),
                self._servo1_angle,
                self._servo2_angle,
                self._target_bbox_size,
            )
        )

    def _rebuild_control_timer(self, log: bool = True) -> None:
        if self._control_timer is not None:
            self.destroy_timer(self._control_timer)
        self._control_timer = self.create_timer(
            1.0 / max(self.follow_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self._control_tick,
        )
        if log:
            self.get_logger().info("Control timer rebuilt: %.1f Hz" % self.follow_rate_hz)

    def _on_parameter_update(self, parameters: list[Parameter]) -> SetParametersResult:
        if not self.enable_rqt_dynamic_parameters:
            for parameter in parameters:
                if parameter.name != "enable_rqt_dynamic_parameters":
                    return SetParametersResult(
                        successful=False,
                        reason="Dynamic parameter updates are disabled.",
                    )

        current_rate = self.follow_rate_hz
        target_size_before = self.target_bbox_size_px
        parameter_overrides = {parameter.name: parameter.value for parameter in parameters}

        for parameter in parameters:
            if parameter.name in RESTART_REQUIRED_PARAMETER_NAMES:
                return SetParametersResult(
                    successful=False,
                    reason="%s requires node restart." % parameter.name,
                )
            if parameter.name not in DYNAMIC_PARAMETER_NAMES:
                continue

        try:
            self._load_parameters(parameter_overrides)
            if abs(self.follow_rate_hz - current_rate) > 1e-6:
                self._rebuild_control_timer(log=True)
            if abs(self.target_bbox_size_px - target_size_before) > 1e-6:
                self._target_bbox_size = self.target_bbox_size_px
                self._target_bbox_size_initialized = self._target_bbox_size > 0.0
                if not self._target_bbox_size_initialized:
                    self.get_logger().info(
                        "Target bbox size reset; next valid bbox will become distance baseline."
                    )
            self._servo1_angle = clamp(
                self._servo1_angle, self.horizontal_min_deg, self.horizontal_max_deg
            )
            self._servo2_angle = clamp(
                self._servo2_angle, self.pitch_min_deg, self.pitch_max_deg
            )
        except Exception as error:
            return SetParametersResult(successful=False, reason=str(error))

        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        try:
            if rclpy.ok():
                self._publish_zero_twist()
        except Exception:
            pass
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = TrackingBboxFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
