#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KCF bbox follower control shared by visual detector nodes."""

from __future__ import annotations

import math
import time
from typing import Any, Callable, Dict

from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool


MINIMUM_TIMER_RATE_HZ = 1.0

CONTROL_DYNAMIC_PARAMETER_NAMES = frozenset(
    {
        "enable_follow",
        "enable_ptz_follow",
        "enable_body_turn_follow",
        "enable_distance_follow",
        "enable_joy_override",
        "image_width",
        "image_height",
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

CONTROL_RESTART_REQUIRED_PARAMETER_NAMES = frozenset(
    {"cmd_vel_topic", "ptz_command_topic", "joy_state_topic"}
)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(float(lower), min(float(upper), float(value)))


def to_bool(value: Any, default: bool) -> bool:
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


class KcfStyleFollowController:
    """Apply tracking_bbox_follow_node control to a detector-provided bbox."""

    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter("cmd_vel_topic", "/cmd_vel")
        node.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        node.declare_parameter("joy_state_topic", "/JoyState")

        node.declare_parameter("enable_follow", True)
        node.declare_parameter("enable_ptz_follow", True)
        node.declare_parameter("enable_body_turn_follow", True)
        node.declare_parameter("enable_distance_follow", True)
        node.declare_parameter("enable_joy_override", True)
        node.declare_parameter("image_width", 640)
        node.declare_parameter("image_height", 480)
        node.declare_parameter("target_stale_timeout_sec", 1.2)
        node.declare_parameter("follow_rate_hz", 10.0)
        node.declare_parameter("motion_log_interval_sec", 0.0)

        node.declare_parameter("horizontal_min_deg", -90.0)
        node.declare_parameter("horizontal_max_deg", 90.0)
        node.declare_parameter("pitch_min_deg", -90.0)
        node.declare_parameter("pitch_max_deg", 20.0)
        node.declare_parameter("initial_horizontal_deg", 0.0)
        node.declare_parameter("initial_pitch_deg", -45.0)
        node.declare_parameter("publish_initial_ptz_command", True)
        node.declare_parameter("initial_publish_count", 20)
        node.declare_parameter("initial_publish_interval_sec", 0.2)

        node.declare_parameter("pixel_tolerance", 15.0)
        node.declare_parameter("kp_x", 0.6)
        node.declare_parameter("kd_x", 0.15)
        node.declare_parameter("kp_y", 0.6)
        node.declare_parameter("kd_y", 0.15)
        node.declare_parameter("max_step_xy", 5.0)
        node.declare_parameter("sign_x", 1.0)
        node.declare_parameter("sign_y", -1.0)

        node.declare_parameter("body_kp", 0.03)
        node.declare_parameter("body_kd", 0.0)
        node.declare_parameter("body_tolerance_deg", 3.0)
        node.declare_parameter("servo1_engage_deg", 5.0)
        node.declare_parameter("max_angular_z", 0.6)
        node.declare_parameter("body_dir", -1.0)
        node.declare_parameter("body_ff_gain", 1.0)

        node.declare_parameter("fixed_camera_pixel_tolerance", 15.0)
        node.declare_parameter("fixed_camera_engage_px", 30.0)
        node.declare_parameter("fixed_camera_body_kp", 0.6)
        node.declare_parameter("fixed_camera_body_kd", 0.15)
        node.declare_parameter("fixed_camera_body_dir", -1.0)

        node.declare_parameter("target_bbox_size_px", 0.0)
        node.declare_parameter("auto_set_target_bbox_size", True)
        node.declare_parameter("distance_tolerance_ratio", 0.06)
        node.declare_parameter("distance_kp", 0.18)
        node.declare_parameter("distance_ki", 0.0)
        node.declare_parameter("distance_kd", 0.02)
        node.declare_parameter("distance_integral_limit", 0.5)
        node.declare_parameter("max_linear_x", 0.12)
        node.declare_parameter("linear_dir", 1.0)

        node.declare_parameter("print_cmd_vel", False)
        node.declare_parameter("print_interval_sec", 0.5)
        node.declare_parameter("enable_rqt_dynamic_parameters", True)

    def __init__(
        self,
        node: Node,
        target_provider: Callable[[], Dict[str, Any]],
        active_provider: Callable[[], bool],
    ) -> None:
        self._node = node
        self._target_provider = target_provider
        self._active_provider = active_provider

        self.horizontal_angle = 0.0
        self.pitch_angle = 0.0
        self.last_angular_z = 0.0
        self.target_bbox_size = 0.0
        self.target_bbox_size_initialized = False

        self._previous_err_x = 0.0
        self._previous_err_y = 0.0
        self._previous_control_time = 0.0
        self._body_correcting = False
        self._previous_body_error = 0.0
        self._previous_distance_error = 0.0
        self._distance_integral = 0.0
        self._last_motion_log_time = 0.0
        self._last_target_lost_log_time = 0.0
        self._joy_is_active = False
        self._previous_joy_is_active = False

        self._load_parameters()
        self.horizontal_angle = self.initial_horizontal_deg
        self.pitch_angle = self.initial_pitch_deg
        self.target_bbox_size = max(0.0, self.target_bbox_size_px)
        self.target_bbox_size_initialized = self.target_bbox_size > 0.0

        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = node.create_publisher(Twist, self.cmd_vel_topic, cmd_qos)
        self.ptz_pub = node.create_publisher(Vector3, self.ptz_command_topic, 10)
        self._joy_sub = node.create_subscription(
            Bool, self.joy_state_topic, self._joy_state_callback, 10
        )

        self._control_timer = None
        self._rebuild_control_timer(log=False)
        self._initial_ptz_timer = None
        self._initial_ptz_remaining = 0
        if self.enable_ptz_follow and self.publish_initial_ptz_command:
            self._start_initial_ptz_publish()

    def _parameter_values(self, overrides: Dict[str, Any] | None = None) -> Dict[str, Any]:
        overrides = overrides or {}

        def value(name: str) -> Any:
            if name in overrides:
                return overrides[name]
            return self._node.get_parameter(name).value

        return {name: value(name) for name in (
            CONTROL_DYNAMIC_PARAMETER_NAMES | CONTROL_RESTART_REQUIRED_PARAMETER_NAMES
        )}

    def _load_parameters(self, overrides: Dict[str, Any] | None = None) -> None:
        values = self._parameter_values(overrides)
        self.cmd_vel_topic = str(values["cmd_vel_topic"])
        self.ptz_command_topic = str(values["ptz_command_topic"])
        self.joy_state_topic = str(values["joy_state_topic"])
        self.enable_follow = to_bool(values["enable_follow"], True)
        self.enable_ptz_follow = to_bool(values["enable_ptz_follow"], True)
        self.enable_body_turn_follow = to_bool(
            values["enable_body_turn_follow"], True
        )
        self.enable_distance_follow = to_bool(
            values["enable_distance_follow"], True
        )
        self.enable_joy_override = to_bool(values["enable_joy_override"], True)
        self.image_width = max(1, int(values["image_width"]))
        self.image_height = max(1, int(values["image_height"]))
        self.target_stale_timeout_sec = max(
            0.05, float(values["target_stale_timeout_sec"])
        )
        self.follow_rate_hz = max(
            MINIMUM_TIMER_RATE_HZ, float(values["follow_rate_hz"])
        )
        self.motion_log_interval_sec = max(
            0.0, float(values["motion_log_interval_sec"])
        )

        self.horizontal_min_deg = float(values["horizontal_min_deg"])
        self.horizontal_max_deg = float(values["horizontal_max_deg"])
        self.pitch_min_deg = float(values["pitch_min_deg"])
        self.pitch_max_deg = float(values["pitch_max_deg"])
        self.initial_horizontal_deg = float(values["initial_horizontal_deg"])
        self.initial_pitch_deg = float(values["initial_pitch_deg"])
        self.publish_initial_ptz_command = to_bool(
            values["publish_initial_ptz_command"], True
        )
        self.initial_publish_count = max(0, int(values["initial_publish_count"]))
        self.initial_publish_interval_sec = max(
            0.01, float(values["initial_publish_interval_sec"])
        )

        self.pixel_tolerance = max(0.0, float(values["pixel_tolerance"]))
        self.ptz_kp_x = max(0.0, float(values["kp_x"]))
        self.ptz_kd_x = max(0.0, float(values["kd_x"]))
        self.ptz_kp_y = max(0.0, float(values["kp_y"]))
        self.ptz_kd_y = max(0.0, float(values["kd_y"]))
        self.max_ptz_step = max(0.0, float(values["max_step_xy"]))
        self.ptz_sign_x = clamp(float(values["sign_x"]), -1.0, 1.0)
        self.ptz_sign_y = clamp(float(values["sign_y"]), -1.0, 1.0)

        self.body_kp = max(0.0, float(values["body_kp"]))
        self.body_kd = max(0.0, float(values["body_kd"]))
        self.body_tolerance_deg = max(0.0, float(values["body_tolerance_deg"]))
        self.servo1_engage_deg = max(0.0, float(values["servo1_engage_deg"]))
        self.max_angular_z = max(0.0, float(values["max_angular_z"]))
        self.body_dir = clamp(float(values["body_dir"]), -1.0, 1.0)
        self.body_ff_gain = clamp(float(values["body_ff_gain"]), 0.0, 3.0)

        self.fixed_camera_pixel_tolerance = max(
            0.0, float(values["fixed_camera_pixel_tolerance"])
        )
        self.fixed_camera_engage_px = max(
            0.0, float(values["fixed_camera_engage_px"])
        )
        self.fixed_camera_body_kp = max(
            0.0, float(values["fixed_camera_body_kp"])
        )
        self.fixed_camera_body_kd = max(
            0.0, float(values["fixed_camera_body_kd"])
        )
        self.fixed_camera_body_dir = clamp(
            float(values["fixed_camera_body_dir"]), -1.0, 1.0
        )

        self.target_bbox_size_px = max(0.0, float(values["target_bbox_size_px"]))
        self.auto_set_target_bbox_size = to_bool(
            values["auto_set_target_bbox_size"], True
        )
        self.distance_tolerance_ratio = max(
            0.0, float(values["distance_tolerance_ratio"])
        )
        self.distance_kp = max(0.0, float(values["distance_kp"]))
        self.distance_ki = max(0.0, float(values["distance_ki"]))
        self.distance_kd = max(0.0, float(values["distance_kd"]))
        self.distance_integral_limit = max(
            0.0, float(values["distance_integral_limit"])
        )
        self.max_linear_x = max(0.0, float(values["max_linear_x"]))
        self.linear_dir = clamp(float(values["linear_dir"]), -1.0, 1.0)
        self.print_cmd_vel = to_bool(values["print_cmd_vel"], False)
        self.print_interval_sec = max(0.0, float(values["print_interval_sec"]))
        self.enable_rqt_dynamic_parameters = to_bool(
            values["enable_rqt_dynamic_parameters"], True
        )

    def validate_parameter_overrides(self, overrides: Dict[str, Any]) -> str:
        try:
            values = self._parameter_values(overrides)
            if int(values["image_width"]) <= 0 or int(values["image_height"]) <= 0:
                return "image_width/image_height must be > 0"
            if float(values["target_stale_timeout_sec"]) < 0.05:
                return "target_stale_timeout_sec must be >= 0.05"
            if float(values["follow_rate_hz"]) < MINIMUM_TIMER_RATE_HZ:
                return "follow_rate_hz must be >= 1.0"
            if float(values["horizontal_min_deg"]) > float(values["horizontal_max_deg"]):
                return "horizontal_min_deg must be <= horizontal_max_deg"
            if float(values["pitch_min_deg"]) > float(values["pitch_max_deg"]):
                return "pitch_min_deg must be <= pitch_max_deg"
            nonnegative = (
                "pixel_tolerance", "kp_x", "kd_x", "kp_y", "kd_y",
                "max_step_xy", "body_kp", "body_kd", "body_tolerance_deg",
                "servo1_engage_deg", "max_angular_z",
                "fixed_camera_pixel_tolerance", "fixed_camera_engage_px",
                "fixed_camera_body_kp", "fixed_camera_body_kd",
                "target_bbox_size_px", "distance_tolerance_ratio",
                "distance_kp", "distance_ki", "distance_kd",
                "distance_integral_limit", "max_linear_x",
                "motion_log_interval_sec", "print_interval_sec",
            )
            if any(float(values[name]) < 0.0 for name in nonnegative):
                return "KCF control gains, tolerances, limits, and sizes must be >= 0"
            if int(values["initial_publish_count"]) < 0:
                return "initial_publish_count must be >= 0"
            if float(values["initial_publish_interval_sec"]) <= 0.0:
                return "initial_publish_interval_sec must be > 0"
            for name in ("sign_x", "sign_y", "body_dir", "fixed_camera_body_dir", "linear_dir"):
                if not -1.0 <= float(values[name]) <= 1.0:
                    return f"{name} must be in [-1, 1]"
            if not 0.0 <= float(values["body_ff_gain"]) <= 3.0:
                return "body_ff_gain must be in [0, 3]"
        except Exception as error:
            return str(error)
        return ""

    def apply_parameter_overrides(self, overrides: Dict[str, Any]) -> None:
        overrides = {
            name: value
            for name, value in overrides.items()
            if name in CONTROL_DYNAMIC_PARAMETER_NAMES
        }
        if not overrides:
            return
        old_rate = self.follow_rate_hz
        old_target_size = self.target_bbox_size_px
        self._load_parameters(overrides)
        if abs(self.follow_rate_hz - old_rate) > 1e-6:
            self._rebuild_control_timer(log=True)
        if abs(self.target_bbox_size_px - old_target_size) > 1e-6:
            self.target_bbox_size = self.target_bbox_size_px
            self.target_bbox_size_initialized = self.target_bbox_size > 0.0
            if not self.target_bbox_size_initialized:
                self._node.get_logger().info(
                    "Target bbox size reset; next valid target becomes distance baseline."
                )
        self.horizontal_angle = clamp(
            self.horizontal_angle, self.horizontal_min_deg, self.horizontal_max_deg
        )
        self.pitch_angle = clamp(
            self.pitch_angle, self.pitch_min_deg, self.pitch_max_deg
        )
        self.reset()

    def _joy_state_callback(self, message: Bool) -> None:
        self._joy_is_active = bool(message.data)
        if self._joy_is_active and not self._previous_joy_is_active:
            self.reset()
            self.publish_zero_twist()
            self._node.get_logger().info(
                "Joy override active; autonomous follow paused."
            )
        self._previous_joy_is_active = self._joy_is_active

    @staticmethod
    def bbox_size(width: float, height: float) -> float:
        return math.sqrt(max(1.0, float(width)) * max(1.0, float(height)))

    def reset(self) -> None:
        self._previous_err_x = 0.0
        self._previous_err_y = 0.0
        self._previous_control_time = 0.0
        self.last_angular_z = 0.0
        self._body_correcting = False
        self._previous_body_error = 0.0
        self._previous_distance_error = 0.0
        self._distance_integral = 0.0

    def _control_tick(self) -> None:
        if not self.enable_follow or not self._active_provider():
            self.reset()
            self.publish_zero_twist()
            return

        if self.enable_joy_override and self._joy_is_active:
            self.reset()
            return

        state = self._target_provider()
        now = time.monotonic()
        fresh = (now - float(state.get("stamp", 0.0))) <= self.target_stale_timeout_sec
        if not state.get("valid", False) or not fresh:
            self.reset()
            self.publish_zero_twist()
            self._log_target_lost_if_needed(state, fresh)
            return

        state["image_width"] = max(
            1, int(state.get("image_width", self.image_width) or self.image_width)
        )
        state["image_height"] = max(
            1, int(state.get("image_height", self.image_height) or self.image_height)
        )
        if self.auto_set_target_bbox_size and not self.target_bbox_size_initialized:
            current_size = self.bbox_size(state["width"], state["height"])
            if current_size > 0.0:
                self.target_bbox_size = current_size
                self.target_bbox_size_initialized = True
                self._node.get_logger().info(
                    "Target bbox size initialized from first valid target: %.1f px"
                    % self.target_bbox_size
                )

        dt = (
            now - self._previous_control_time
            if self._previous_control_time > 0.0
            else 1.0 / max(self.follow_rate_hz, MINIMUM_TIMER_RATE_HZ)
        )
        dt = max(dt, 1e-3)
        if self.enable_ptz_follow:
            self._apply_ptz_follow(state, dt)

        twist = self._compute_chassis_twist(state, dt)
        self.cmd_pub.publish(twist)
        self._log_motion_state(twist, state)
        self._previous_control_time = now

    def _log_target_lost_if_needed(self, state: Dict[str, Any], fresh: bool) -> None:
        now = time.monotonic()
        if now - self._last_target_lost_log_time < 2.0:
            return
        self._last_target_lost_log_time = now
        self._node.get_logger().warn(
            "No valid visual target; stopping. valid=%s fresh=%s"
            % (state.get("valid", False), fresh)
        )

    def publish_zero_twist(self) -> None:
        self.cmd_pub.publish(Twist())

    def publish_ptz_angle(self) -> None:
        message = Vector3()
        message.x = float(self.horizontal_angle)
        message.y = float(self.pitch_angle)
        message.z = 0.0
        self.ptz_pub.publish(message)

    def _start_initial_ptz_publish(self) -> None:
        if self.initial_publish_count <= 0:
            return
        self._initial_ptz_remaining = self.initial_publish_count
        self._initial_ptz_timer = self._node.create_timer(
            self.initial_publish_interval_sec, self._publish_initial_ptz_tick
        )

    def _publish_initial_ptz_tick(self) -> None:
        if self._initial_ptz_remaining <= 0:
            if self._initial_ptz_timer is not None:
                self._node.destroy_timer(self._initial_ptz_timer)
                self._initial_ptz_timer = None
            return
        self.publish_ptz_angle()
        self._initial_ptz_remaining -= 1

    def _apply_ptz_follow(self, state: Dict[str, Any], dt: float) -> None:
        if abs(self.last_angular_z) > 1e-6:
            body_yaw_deg = math.degrees(self.last_angular_z * dt)
            feedforward_deg = self.body_ff_gain * self.ptz_sign_x * body_yaw_deg
            self.horizontal_angle = clamp(
                self.horizontal_angle + feedforward_deg,
                self.horizontal_min_deg,
                self.horizontal_max_deg,
            )

        half_width = max(1.0, float(state["image_width"])) * 0.5
        half_height = max(1.0, float(state["image_height"])) * 0.5
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

        self.horizontal_angle = clamp(
            self.horizontal_angle + self.ptz_sign_x * step_x,
            self.horizontal_min_deg,
            self.horizontal_max_deg,
        )
        self.pitch_angle = clamp(
            self.pitch_angle + self.ptz_sign_y * step_y,
            self.pitch_min_deg,
            self.pitch_max_deg,
        )
        self.publish_ptz_angle()

    def _compute_ptz_body_turn(self, dt: float) -> float:
        servo_error = self.horizontal_angle - self.initial_horizontal_deg
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
        self._previous_body_error = servo_error
        return clamp(angular_z, -self.max_angular_z, self.max_angular_z)

    def _compute_fixed_camera_body_turn(self, state: Dict[str, Any]) -> float:
        half_width = max(1.0, float(state["image_width"])) * 0.5
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

    def _compute_distance_follow(self, state: Dict[str, Any], dt: float) -> float:
        if not self.enable_distance_follow:
            self._previous_distance_error = 0.0
            self._distance_integral = 0.0
            return 0.0
        if not self.target_bbox_size_initialized:
            return 0.0

        target_size = max(1.0, float(self.target_bbox_size))
        current_size = self.bbox_size(state["width"], state["height"])
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

    def _compute_chassis_twist(self, state: Dict[str, Any], dt: float) -> Twist:
        twist = Twist()
        if self.enable_body_turn_follow:
            if self.enable_ptz_follow:
                twist.angular.z = self._compute_ptz_body_turn(dt)
            else:
                twist.angular.z = self._compute_fixed_camera_body_turn(state)
            self.last_angular_z = float(twist.angular.z)
        else:
            self.last_angular_z = 0.0
        twist.linear.x = self._compute_distance_follow(state, dt)
        return twist

    def _log_motion_state(self, twist: Twist, state: Dict[str, Any]) -> None:
        if not self.print_cmd_vel and self.motion_log_interval_sec <= 0.0:
            return
        interval = (
            self.print_interval_sec
            if self.print_cmd_vel
            else self.motion_log_interval_sec
        )
        if interval <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_motion_log_time < interval:
            return
        self._last_motion_log_time = now
        self._node.get_logger().info(
            "cmd_vel linear.x=%.3f angular.z=%.3f target=(%.1f,%.1f) "
            "ptz=(%.1f,%.1f) size=%.1f/%.1f"
            % (
                twist.linear.x,
                twist.angular.z,
                float(state["cx"]),
                float(state["cy"]),
                self.horizontal_angle,
                self.pitch_angle,
                self.bbox_size(state["width"], state["height"]),
                self.target_bbox_size,
            )
        )

    def _rebuild_control_timer(self, log: bool = True) -> None:
        if self._control_timer is not None:
            self._node.destroy_timer(self._control_timer)
        self._control_timer = self._node.create_timer(
            1.0 / max(self.follow_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self._control_tick,
        )
        if log:
            self._node.get_logger().info(
                "Control timer rebuilt: %.1f Hz" % self.follow_rate_hz
            )

    def shutdown(self) -> None:
        try:
            self.publish_zero_twist()
        except Exception:
            pass
