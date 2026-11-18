#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_line_ptz_node.py

中文说明:
    ROS2 Humble 巡线底盘控制节点。
    本节点参考物体跟踪/人脸跟踪节点的低帧率稳定控制思路，将图像处理与底盘控制解耦：
    图像回调只缓存最新图像，图像处理定时器负责 HSV 巡线识别，底盘控制定时器固定频率发布 /cmd_vel。
    启动时可发布云台初始角度，默认 pitch 为 -90 deg，便于摄像头朝向地面巡线。

English Description:
    ROS2 Humble line following control node.
    This node follows the low-FPS stable control idea used by object/face tracking nodes:
    the image callback only caches the latest image, the image processing timer detects the line,
    and a fixed-rate control timer publishes /cmd_vel. At startup it can publish an initial PTZ angle,
    with pitch defaulting to -90 deg for ground-facing line following.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, UInt16
from .image_conversion import image_to_bgr


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "follow_line_ptz_node"

# TRACKING_STATE_SELECTING: HSV 采样选择状态 / HSV sampling selection state.
TRACKING_STATE_SELECTING = "selecting"

# TRACKING_STATE_IDENTIFY: 只识别路线但不控制底盘状态 / Identify line without chassis control state.
TRACKING_STATE_IDENTIFY = "identify"

# TRACKING_STATE_TRACKING: 巡线控制状态 / Active line following state.
TRACKING_STATE_TRACKING = "tracking"

# TRACKING_STATE_LOST: 路线丢失状态 / Line lost state.
TRACKING_STATE_LOST = "lost"

# TRACKING_STATE_PAUSED: 暂停状态 / Paused state.
TRACKING_STATE_PAUSED = "paused"

# KEY_CODE_SPACE: 空格键编码 / Space key code.
KEY_CODE_SPACE = 32

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# MINIMUM_TIMER_RATE_HZ: 定时器最小频率 / Minimum timer frequency.
MINIMUM_TIMER_RATE_HZ = 1.0

# MINIMUM_ALPHA_VALUE: 平滑系数最小值 / Minimum smoothing alpha value.
MINIMUM_ALPHA_VALUE = 0.01

# MAXIMUM_ALPHA_VALUE: 平滑系数最大值 / Maximum smoothing alpha value.
MAXIMUM_ALPHA_VALUE = 1.0

# MINIMUM_LINE_LOST_TIMEOUT_SEC: 路线丢失超时时间下限 / Lower bound of line lost timeout.
MINIMUM_LINE_LOST_TIMEOUT_SEC = 0.05

# MINIMUM_ROI_SIZE_PIXEL: 鼠标框选 ROI 最小尺寸 / Minimum mouse ROI size in pixels.
MINIMUM_ROI_SIZE_PIXEL = 8

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
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
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
    "line_roi_top_ratio",
    "line_roi_bottom_ratio",
    "line_min_area",
    "line_lost_timeout_sec",
    "target_smooth_alpha",
    "line_pid_kp",
    "line_pid_ki",
    "line_pid_kd",
    "pid_scale",
    "pid_integral_limit",
    "line_error_divisor",
    "line_linear_speed",
    "line_center_deadband_pixel",
    "max_angular_speed",
    "reverse_angular_direction",
    "control_rate_hz",
    "image_processing_rate_hz",
    "enable_obstacle_stop",
    "obstacle_min_valid_distance",
    "obstacle_response_distance",
    "obstacle_check_angle_deg",
    "obstacle_warning_count_threshold",
    "enable_beep",
    "auto_start_tracking",
    "lost_line_stop",
    "publish_initial_ptz_command",
    "initial_horizontal_deg",
    "initial_pitch_deg",
    "initial_publish_count",
    "initial_publish_interval_sec",
    "debug_log_interval_sec",
    "print_cmd_vel",
    "print_interval_sec",
    "enable_rqt_dynamic_parameters",
    "hsv_config_file",
    "save_hsv_to_file",
    "load_hsv_from_file",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require node restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "image_topic",
    "cmd_vel_topic",
    "scan_topic",
    "beep_topic",
    "joy_state_topic",
    "ptz_command_topic",
    "window_name",
})


@dataclass
class LineTrackingResult:
    """路线识别结果 / Line detection result."""

    # is_line_found: 是否识别到路线 / Whether a line is found.
    is_line_found: bool

    # line_center_x: 路线中心 x 像素坐标 / Line center x coordinate in pixels.
    line_center_x: float = 0.0

    # line_center_y: 路线中心 y 像素坐标 / Line center y coordinate in pixels.
    line_center_y: float = 0.0

    # line_area: 路线轮廓面积 / Line contour area.
    line_area: float = 0.0

    # line_radius: 路线等效圆半径 / Equivalent line contour radius.
    line_radius: float = 0.0

    # line_rectangle: 路线外接矩形 / Line bounding rectangle.
    line_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)

    # line_contour: 路线轮廓 / Line contour.
    line_contour: Optional[np.ndarray] = None

    # binary_mask: 二值掩膜 / Binary mask.
    binary_mask: Optional[np.ndarray] = None


class DiscretePidController:
    """离散 PID 控制器 / Discrete PID controller."""

    def __init__(self, proportional_gain: float, integral_gain: float, derivative_gain: float, integral_limit: float) -> None:
        """
        初始化离散 PID 控制器 / Initialize a discrete PID controller.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            integral_gain: 积分增益 / Integral gain.
            derivative_gain: 微分增益 / Derivative gain.
            integral_limit: 积分限幅 / Integral clamp limit.
        """
        # proportional_gain: 比例增益 / Proportional gain.
        self.proportional_gain = float(proportional_gain)
        # integral_gain: 积分增益 / Integral gain.
        self.integral_gain = float(integral_gain)
        # derivative_gain: 微分增益 / Derivative gain.
        self.derivative_gain = float(derivative_gain)
        # integral_limit: 积分限幅 / Integral clamp limit.
        self.integral_limit = max(0.0, float(integral_limit))
        # previous_error: 上一次误差 / Previous error.
        self.previous_error = 0.0
        # integral_error: 积分误差 / Integral error.
        self.integral_error = 0.0
        # previous_time_sec: 上一次更新时间 / Previous update time.
        self.previous_time_sec: Optional[float] = None

    def update_gains(self, proportional_gain: float, integral_gain: float, derivative_gain: float, integral_limit: float) -> None:
        """
        更新 PID 参数 / Update PID gains.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            integral_gain: 积分增益 / Integral gain.
            derivative_gain: 微分增益 / Derivative gain.
            integral_limit: 积分限幅 / Integral clamp limit.
        """
        self.proportional_gain = float(proportional_gain)
        self.integral_gain = float(integral_gain)
        self.derivative_gain = float(derivative_gain)
        self.integral_limit = max(0.0, float(integral_limit))

    def reset(self) -> None:
        """重置 PID 内部状态 / Reset PID internal state."""
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.previous_time_sec = None

    def update(self, current_value: float, target_value: float = 0.0) -> float:
        """
        根据当前值更新 PID 输出 / Update PID output from current value.

        Args:
            current_value: 当前输入值 / Current input value.
            target_value: 目标输入值 / Target input value.

        Returns:
            float: PID 输出 / PID output.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        # error_value: 当前误差，保持和旧巡线程序 simplePID 的 target-current 逻辑一致 / Current error.
        error_value = float(target_value) - float(current_value)

        if self.previous_time_sec is None:
            self.previous_time_sec = current_time_sec
            self.previous_error = error_value
            return 0.0

        # delta_time_sec: 控制周期 / Control period.
        delta_time_sec = max(current_time_sec - self.previous_time_sec, 1e-6)
        self.previous_time_sec = current_time_sec

        self.integral_error += error_value * delta_time_sec
        if self.integral_limit > 0.0:
            self.integral_error = float(np.clip(self.integral_error, -self.integral_limit, self.integral_limit))

        # derivative_error: 微分误差 / Derivative error.
        derivative_error = (error_value - self.previous_error) / delta_time_sec
        self.previous_error = error_value

        return (
            self.proportional_gain * error_value
            + self.integral_gain * self.integral_error
            + self.derivative_gain * derivative_error
        )


class FollowLinePtzNode(Node):
    """巡线底盘控制节点 / Line following chassis control node."""

    def __init__(self) -> None:
        """初始化巡线节点 / Initialize the line following node."""
        super().__init__(DEFAULT_NODE_NAME)

        # image_lock: 图像缓存互斥锁 / Image cache mutex.
        self.image_lock = threading.Lock()
        # line_state_lock: 路线状态互斥锁 / Line state mutex.
        self.line_state_lock = threading.Lock()
        # scan_state_lock: 激光状态互斥锁 / Scan state mutex.
        self.scan_state_lock = threading.Lock()

        # latest_image_msg: 最新图像消息 / Latest image message.
        self.latest_image_msg: Optional[Image] = None
        # latest_image_sequence_id: 最新图像序号 / Latest image sequence ID.
        self.latest_image_sequence_id = 0
        # processed_image_sequence_id: 已处理图像序号 / Processed image sequence ID.
        self.processed_image_sequence_id = 0

        # tracking_state: 当前巡线状态 / Current line following state.
        self.tracking_state = TRACKING_STATE_IDENTIFY
        # is_node_running: 节点运行标志 / Node running flag.
        self.is_node_running = True
        # joy_is_active: 手柄接管标志 / Joystick override flag.
        self.joy_is_active = False

        # line_is_valid: 路线有效标志 / Line valid flag.
        self.line_is_valid = False
        # raw_line_center: 原始路线中心 / Raw line center.
        self.raw_line_center: Optional[Tuple[float, float]] = None
        # filtered_line_center: 滤波路线中心 / Filtered line center.
        self.filtered_line_center: Optional[Tuple[float, float]] = None
        # line_rectangle: 路线外接矩形 / Line bounding rectangle.
        self.line_rectangle: Tuple[int, int, int, int] = (0, 0, 0, 0)
        # line_area: 路线面积 / Line area.
        self.line_area = 0.0
        # line_radius: 路线等效半径 / Line equivalent radius.
        self.line_radius = 0.0
        # latest_line_update_time_sec: 最近路线更新时间 / Latest line update time.
        self.latest_line_update_time_sec = 0.0
        # line_lost_start_time_sec: 路线开始丢失时间 / Line lost start time.
        self.line_lost_start_time_sec: Optional[float] = None
        # line_lost_timeout_reported: 丢线超时日志是否已打印 / Whether line lost timeout was reported.
        self.line_lost_timeout_reported = False
        # image_width: 图像宽度 / Image width.
        self.image_width = 0
        # image_height: 图像高度 / Image height.
        self.image_height = 0
        # latest_display_frame: 最新显示图像 / Latest display frame.
        self.latest_display_frame: Optional[np.ndarray] = None
        # latest_binary_mask: 最新二值图 / Latest binary mask.
        self.latest_binary_mask: Optional[np.ndarray] = None

        # obstacle_warning_count: 障碍物告警点数 / Obstacle warning point count.
        self.obstacle_warning_count = 0
        # obstacle_is_active: 障碍物触发标志 / Obstacle active flag.
        self.obstacle_is_active = False
        # beep_is_active: 蜂鸣器状态 / Buzzer state.
        self.beep_is_active = False

        # mouse_selecting: 鼠标正在选择 ROI 标志 / Mouse ROI selecting flag.
        self.mouse_selecting = False
        # roi_selection_start: ROI 起点 / ROI start point.
        self.roi_selection_start: Tuple[int, int] = (0, 0)
        # roi_selection_end: ROI 终点 / ROI end point.
        self.roi_selection_end: Tuple[int, int] = (0, 0)
        # roi_selection_pending: ROI HSV 更新待处理标志 / ROI HSV update pending flag.
        self.roi_selection_pending = False
        # debug_window_initialized: 调试窗口初始化标志 / Debug window initialized flag.
        self.debug_window_initialized = False

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
        # shutdown_requested: 是否已经请求关闭 / Whether shutdown has already been requested.
        self.shutdown_requested = False

        self.declare_and_get_parameters()
        self.load_hsv_parameters_from_file_if_needed()

        # line_pid_controller: 巡线角速度 PID 控制器 / Line angular velocity PID controller.
        self.line_pid_controller = DiscretePidController(
            self.line_pid_kp / self.pid_scale,
            self.line_pid_ki / self.pid_scale,
            self.line_pid_kd / self.pid_scale,
            self.pid_integral_limit,
        )

        # image_subscription: 图像订阅器 / Image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE),
        )
        # scan_subscription: 激光雷达订阅器 / Laser scan subscriber.
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
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
        # ptz_command_publisher: 云台角度发布器 / PTZ angle publisher.
        self.ptz_command_publisher = self.create_publisher(Vector3, self.ptz_command_topic, 10)
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

        # initial_ptz_publish_counter: 初始云台角度发布计数 / Initial PTZ publish counter.
        self.initial_ptz_publish_counter = 0
        # initial_ptz_timer: 初始云台角度发布定时器 / Initial PTZ publish timer.
        self.initial_ptz_timer = None
        if self.publish_initial_ptz_command:
            self.initial_ptz_timer = self.create_timer(
                max(self.initial_publish_interval_sec, 0.02),
                self.publish_initial_ptz_command_callback,
            )

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(
            self.on_dynamic_parameter_update
        )

        if self.auto_start_tracking:
            self.tracking_state = TRACKING_STATE_TRACKING

        self.get_logger().info(
            "Follow line PTZ node started. image_topic=%s, cmd_vel_topic=%s, "
            "beep_topic=%s, enable_beep=%s, initial_pitch_deg=%.1f, "
            "line_linear_speed=%.2f"
            % (
                self.image_topic,
                self.cmd_vel_topic,
                self.beep_topic,
                self.enable_beep,
                self.initial_pitch_deg,
                self.line_linear_speed,
            )
        )

    def declare_and_get_parameters(self) -> None:
        """声明并读取参数 / Declare and read parameters."""
        # 图像和话题参数 / Image and topic parameters.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("beep_topic", "/beep")
        self.declare_parameter("joy_state_topic", "/JoyState")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "follow_line_ptz")

        # 显示参数 / Display parameters.
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 15)
        self.declare_parameter("overlay_margin_x", 6)
        self.declare_parameter("overlay_margin_y", 16)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("show_binary_window", True)

        # 图像方向参数 / Image orientation parameters.
        self.declare_parameter("enable_horizontal_flip", True)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        # HSV 参数 / HSV parameters.
        self.declare_parameter("Hmin", 0)
        self.declare_parameter("Smin", 85)
        self.declare_parameter("Vmin", 126)
        self.declare_parameter("Hmax", 9)
        self.declare_parameter("Smax", 253)
        self.declare_parameter("Vmax", 253)
        self.declare_parameter("h_expand", 8)
        self.declare_parameter("s_expand", 40)
        self.declare_parameter("v_expand", 40)
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("hsv_config_file", "")
        self.declare_parameter("save_hsv_to_file", False)
        self.declare_parameter("load_hsv_from_file", False)

        # 路线检测参数 / Line detection parameters.
        self.declare_parameter("line_roi_top_ratio", 0.55)
        self.declare_parameter("line_roi_bottom_ratio", 1.00)
        self.declare_parameter("line_min_area", 150.0)
        self.declare_parameter("line_lost_timeout_sec", 0.5)
        self.declare_parameter("target_smooth_alpha", 0.55)

        # PID 和底盘控制参数 / PID and chassis control parameters.
        self.declare_parameter("line_pid_kp", 50.0)
        self.declare_parameter("line_pid_ki", 0.0)
        self.declare_parameter("line_pid_kd", 5.0)
        self.declare_parameter("pid_scale", 1000.0)
        self.declare_parameter("pid_integral_limit", 100.0)
        self.declare_parameter("line_error_divisor", 16.0)
        self.declare_parameter("line_linear_speed", 0.30)
        self.declare_parameter("line_center_deadband_pixel", 40.0)
        self.declare_parameter("max_angular_speed", 1.5)
        self.declare_parameter("reverse_angular_direction", True)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)

        # 避障参数 / Obstacle avoidance parameters.
        self.declare_parameter("enable_obstacle_stop", True)
        self.declare_parameter("obstacle_min_valid_distance", 0.20)
        self.declare_parameter("obstacle_response_distance", 0.55)
        self.declare_parameter("obstacle_check_angle_deg", 60.0)
        self.declare_parameter("obstacle_warning_count_threshold", 10)
        self.declare_parameter("enable_beep", True)

        # 云台初始化参数 / PTZ initialization parameters.
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -90.0)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)

        # 状态和调试参数 / State and debug parameters.
        self.declare_parameter("auto_start_tracking", False)
        self.declare_parameter("lost_line_stop", True)
        self.declare_parameter("debug_log_interval_sec", 2.0)
        self.declare_parameter("print_cmd_vel", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

        self.refresh_parameters_from_ros()

    def refresh_parameters_from_ros(self) -> None:
        """从 ROS 参数服务器刷新参数 / Refresh parameters from ROS parameter server."""
        # image_topic: 图像输入话题 / Image input topic.
        self.image_topic = self.get_parameter("image_topic").value
        # cmd_vel_topic: 底盘速度输出话题 / Chassis velocity output topic.
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        # scan_topic: 激光雷达输入话题 / Laser scan input topic.
        self.scan_topic = self.get_parameter("scan_topic").value
        # beep_topic: 蜂鸣器输出话题 / Buzzer output topic.
        self.beep_topic = self.get_parameter("beep_topic").value
        # joy_state_topic: 手柄状态输入话题 / Joystick state input topic.
        self.joy_state_topic = self.get_parameter("joy_state_topic").value
        # ptz_command_topic: 云台角度输出话题 / PTZ angle output topic.
        self.ptz_command_topic = self.get_parameter("ptz_command_topic").value
        # window_name: 调试窗口名称 / Debug window name.
        self.window_name = self.get_parameter("window_name").value

        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.window_width = int(self.get_parameter("window_width").value)
        self.window_height = int(self.get_parameter("window_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_margin_x = int(self.get_parameter("overlay_margin_x").value)
        self.overlay_margin_y = int(self.get_parameter("overlay_margin_y").value)
        self.overlay_show_background = bool(self.get_parameter("overlay_show_background").value)
        self.show_binary_window = bool(self.get_parameter("show_binary_window").value)

        self.enable_horizontal_flip = bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)

        self.Hmin = int(self.get_parameter("Hmin").value)
        self.Smin = int(self.get_parameter("Smin").value)
        self.Vmin = int(self.get_parameter("Vmin").value)
        self.Hmax = int(self.get_parameter("Hmax").value)
        self.Smax = int(self.get_parameter("Smax").value)
        self.Vmax = int(self.get_parameter("Vmax").value)
        self.h_expand = int(self.get_parameter("h_expand").value)
        self.s_expand = int(self.get_parameter("s_expand").value)
        self.v_expand = int(self.get_parameter("v_expand").value)
        self.morph_kernel_size = int(self.get_parameter("morph_kernel_size").value)
        self.hsv_config_file = str(self.get_parameter("hsv_config_file").value)
        self.save_hsv_to_file = bool(self.get_parameter("save_hsv_to_file").value)
        self.load_hsv_from_file = bool(self.get_parameter("load_hsv_from_file").value)

        self.line_roi_top_ratio = float(self.get_parameter("line_roi_top_ratio").value)
        self.line_roi_bottom_ratio = float(self.get_parameter("line_roi_bottom_ratio").value)
        self.line_min_area = float(self.get_parameter("line_min_area").value)
        self.line_lost_timeout_sec = max(float(self.get_parameter("line_lost_timeout_sec").value), MINIMUM_LINE_LOST_TIMEOUT_SEC)
        self.target_smooth_alpha = self.clamp_alpha(float(self.get_parameter("target_smooth_alpha").value))

        self.line_pid_kp = float(self.get_parameter("line_pid_kp").value)
        self.line_pid_ki = float(self.get_parameter("line_pid_ki").value)
        self.line_pid_kd = float(self.get_parameter("line_pid_kd").value)
        self.pid_scale = max(float(self.get_parameter("pid_scale").value), 1e-6)
        self.pid_integral_limit = float(self.get_parameter("pid_integral_limit").value)
        self.line_error_divisor = max(float(self.get_parameter("line_error_divisor").value), 1e-6)
        self.line_linear_speed = float(self.get_parameter("line_linear_speed").value)
        self.line_center_deadband_pixel = float(self.get_parameter("line_center_deadband_pixel").value)
        self.max_angular_speed = abs(float(self.get_parameter("max_angular_speed").value))
        self.reverse_angular_direction = bool(self.get_parameter("reverse_angular_direction").value)
        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self.image_processing_rate_hz = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)

        self.enable_obstacle_stop = bool(self.get_parameter("enable_obstacle_stop").value)
        self.obstacle_min_valid_distance = float(self.get_parameter("obstacle_min_valid_distance").value)
        self.obstacle_response_distance = float(self.get_parameter("obstacle_response_distance").value)
        self.obstacle_check_angle_deg = abs(float(self.get_parameter("obstacle_check_angle_deg").value))
        self.obstacle_warning_count_threshold = int(self.get_parameter("obstacle_warning_count_threshold").value)
        self.enable_beep = bool(self.get_parameter("enable_beep").value)

        self.publish_initial_ptz_command = bool(self.get_parameter("publish_initial_ptz_command").value)
        self.initial_horizontal_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self.initial_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)

        self.auto_start_tracking = bool(self.get_parameter("auto_start_tracking").value)
        self.lost_line_stop = bool(self.get_parameter("lost_line_stop").value)
        self.debug_log_interval_sec = float(self.get_parameter("debug_log_interval_sec").value)
        self.print_cmd_vel = bool(self.get_parameter("print_cmd_vel").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
        self.enable_rqt_dynamic_parameters = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

    @staticmethod
    def clamp_alpha(alpha_value: float) -> float:
        """
        限制平滑系数范围 / Clamp smoothing alpha range.

        Args:
            alpha_value: 原始平滑系数 / Raw smoothing alpha.

        Returns:
            float: 限幅后的平滑系数 / Clamped smoothing alpha.
        """
        return float(np.clip(alpha_value, MINIMUM_ALPHA_VALUE, MAXIMUM_ALPHA_VALUE))

    def load_hsv_parameters_from_file_if_needed(self) -> None:
        """按需从文件加载 HSV 参数 / Load HSV parameters from file if needed."""
        if not self.load_hsv_from_file or not self.hsv_config_file:
            return
        if not os.path.exists(self.hsv_config_file):
            self.get_logger().warn("HSV config file does not exist: %s" % self.hsv_config_file)
            return
        try:
            with open(self.hsv_config_file, "r", encoding="utf-8") as hsv_file:
                # hsv_values: HSV 参数列表 / HSV value list.
                hsv_values = [int(value) for value in hsv_file.read().replace(",", " ").split()[:6]]
            if len(hsv_values) == 6:
                self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax = hsv_values
                self.set_parameters([
                    Parameter("Hmin", Parameter.Type.INTEGER, self.Hmin),
                    Parameter("Smin", Parameter.Type.INTEGER, self.Smin),
                    Parameter("Vmin", Parameter.Type.INTEGER, self.Vmin),
                    Parameter("Hmax", Parameter.Type.INTEGER, self.Hmax),
                    Parameter("Smax", Parameter.Type.INTEGER, self.Smax),
                    Parameter("Vmax", Parameter.Type.INTEGER, self.Vmax),
                ])
                self.get_logger().info("Loaded HSV config from %s" % self.hsv_config_file)
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().warn("Failed to load HSV config: %s" % exception)

    def save_hsv_parameters_to_file_if_needed(self) -> None:
        """按需保存 HSV 参数到文件 / Save HSV parameters to file if needed."""
        if not self.save_hsv_to_file or not self.hsv_config_file:
            return
        try:
            # hsv_directory: HSV 文件目录 / HSV file directory.
            hsv_directory = os.path.dirname(self.hsv_config_file)
            if hsv_directory:
                os.makedirs(hsv_directory, exist_ok=True)
            with open(self.hsv_config_file, "w", encoding="utf-8") as hsv_file:
                hsv_file.write("%d %d %d %d %d %d\n" % (self.Hmin, self.Smin, self.Vmin, self.Hmax, self.Smax, self.Vmax))
            self.get_logger().info("Saved HSV config to %s" % self.hsv_config_file)
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().warn("Failed to save HSV config: %s" % exception)

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
        if self.joy_is_active:
            self.line_pid_controller.reset()
            self.publish_zero_twist()

    def scan_callback(self, scan_msg: LaserScan) -> None:
        """
        处理激光雷达避障数据 / Process laser scan obstacle data.

        Args:
            scan_msg: 激光雷达扫描消息 / Laser scan message.
        """
        # warning_count: 障碍物告警点数量 / Obstacle warning point count.
        warning_count = 0
        if not scan_msg.ranges:
            return
        # obstacle_check_angle_deg: 正前方检测总角度，60 deg 表示 -30..+30 deg / Total front sector.
        half_check_angle_deg = min(self.obstacle_check_angle_deg * 0.5, 180.0)
        for range_index, range_value in enumerate(scan_msg.ranges):
            if not math.isfinite(range_value):
                continue
            # scan_angle_deg: 当前扫描角度 / Current scan angle.
            scan_angle_deg = math.degrees(scan_msg.angle_min + scan_msg.angle_increment * range_index)
            normalized_scan_angle_deg = (scan_angle_deg + 180.0) % 360.0 - 180.0
            if abs(normalized_scan_angle_deg) > half_check_angle_deg:
                continue
            if self.obstacle_min_valid_distance < range_value < self.obstacle_response_distance:
                warning_count += 1

        with self.scan_state_lock:
            self.obstacle_warning_count = warning_count
            self.obstacle_is_active = warning_count > self.obstacle_warning_count_threshold

    def publish_initial_ptz_command_callback(self) -> None:
        """发布初始云台角度 / Publish initial PTZ angle."""
        if self.initial_ptz_publish_counter >= self.initial_publish_count:
            if self.initial_ptz_timer is not None:
                self.destroy_timer(self.initial_ptz_timer)
                self.initial_ptz_timer = None
            return
        self.publish_ptz_angle(self.initial_horizontal_deg, self.initial_pitch_deg)
        self.initial_ptz_publish_counter += 1

    def publish_ptz_angle(self, horizontal_angle_deg: float, pitch_angle_deg: float) -> None:
        """
        发布云台角度 / Publish PTZ angle.

        Args:
            horizontal_angle_deg: 水平角度 / Horizontal angle.
            pitch_angle_deg: 俯仰角度 / Pitch angle.
        """
        # ptz_command_msg: 云台角度消息 / PTZ angle message.
        ptz_command_msg = Vector3()
        ptz_command_msg.x = float(horizontal_angle_deg)
        ptz_command_msg.y = float(pitch_angle_deg)
        ptz_command_msg.z = 0.0
        self.ptz_command_publisher.publish(ptz_command_msg)

    def process_latest_image(self) -> None:
        """处理最新图像并更新路线状态 / Process latest image and update line state."""
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

        if self.roi_selection_pending:
            self.update_hsv_from_selected_roi(processed_frame)

        # line_result: 路线识别结果 / Line detection result.
        line_result = self.detect_line(processed_frame)
        self.update_line_state(line_result)

        # display_frame: 调试显示图像 / Debug display frame.
        display_frame = processed_frame.copy()
        self.draw_line_detection(display_frame, line_result)
        self.draw_roi_selection(display_frame)
        self.draw_overlay_text(display_frame)

        self.latest_display_frame = display_frame
        self.latest_binary_mask = line_result.binary_mask
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

    def detect_line(self, image_frame: np.ndarray) -> LineTrackingResult:
        """
        使用 HSV 阈值检测路线 / Detect line using HSV threshold.

        Args:
            image_frame: BGR 图像 / BGR frame.

        Returns:
            LineTrackingResult: 路线识别结果 / Line detection result.
        """
        # image_height: 图像高度 / Image height.
        image_height = image_frame.shape[0]
        # image_width: 图像宽度 / Image width.
        image_width = image_frame.shape[1]
        # roi_top_y: ROI 顶部 y 坐标 / ROI top y coordinate.
        roi_top_y = int(np.clip(self.line_roi_top_ratio, 0.0, 1.0) * image_height)
        # roi_bottom_y: ROI 底部 y 坐标 / ROI bottom y coordinate.
        roi_bottom_y = int(np.clip(self.line_roi_bottom_ratio, 0.0, 1.0) * image_height)
        if roi_bottom_y <= roi_top_y:
            roi_top_y = int(0.55 * image_height)
            roi_bottom_y = image_height

        # roi_frame: 巡线感兴趣区域 / Line following region of interest.
        roi_frame = image_frame[roi_top_y:roi_bottom_y, 0:image_width]
        # hsv_frame: HSV 图像 / HSV frame.
        hsv_frame = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        # binary_roi_mask: ROI 二值掩膜 / ROI binary mask.
        binary_roi_mask = self.create_hsv_mask(hsv_frame)
        binary_roi_mask = self.apply_morphology(binary_roi_mask)

        # full_binary_mask: 全图二值掩膜 / Full image binary mask.
        full_binary_mask = np.zeros((image_height, image_width), dtype=np.uint8)
        full_binary_mask[roi_top_y:roi_bottom_y, 0:image_width] = binary_roi_mask

        # contours: 轮廓列表 / Contour list.
        contours, _ = cv2.findContours(binary_roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return LineTrackingResult(False, binary_mask=full_binary_mask)

        # largest_contour: 最大轮廓 / Largest contour.
        largest_contour = max(contours, key=cv2.contourArea)
        # line_area: 路线轮廓面积 / Line contour area.
        line_area = float(cv2.contourArea(largest_contour))
        if line_area < self.line_min_area:
            return LineTrackingResult(False, binary_mask=full_binary_mask)

        # contour_moments: 轮廓矩 / Contour moments.
        contour_moments = cv2.moments(largest_contour)
        if abs(contour_moments["m00"]) < 1e-6:
            return LineTrackingResult(False, binary_mask=full_binary_mask)

        # center_x_roi: ROI 内中心 x / Center x inside ROI.
        center_x_roi = float(contour_moments["m10"] / contour_moments["m00"])
        # center_y_roi: ROI 内中心 y / Center y inside ROI.
        center_y_roi = float(contour_moments["m01"] / contour_moments["m00"])
        # center_x: 全图中心 x / Full frame center x.
        center_x = center_x_roi
        # center_y: 全图中心 y / Full frame center y.
        center_y = center_y_roi + roi_top_y

        # bounding_x, bounding_y, bounding_w, bounding_h: 外接矩形 / Bounding rectangle.
        bounding_x, bounding_y, bounding_w, bounding_h = cv2.boundingRect(largest_contour)
        # adjusted_rectangle: 映射到全图的外接矩形 / Bounding rectangle mapped to full frame.
        adjusted_rectangle = (int(bounding_x), int(bounding_y + roi_top_y), int(bounding_w), int(bounding_h))
        # enclosing_circle: 最小外接圆 / Minimum enclosing circle.
        (_, _), line_radius = cv2.minEnclosingCircle(largest_contour)
        # adjusted_contour: 映射到全图的轮廓 / Contour mapped to full frame.
        adjusted_contour = largest_contour.copy()
        adjusted_contour[:, :, 1] += roi_top_y

        return LineTrackingResult(
            True,
            center_x,
            center_y,
            line_area,
            float(line_radius),
            adjusted_rectangle,
            adjusted_contour,
            full_binary_mask,
        )

    def create_hsv_mask(self, hsv_frame: np.ndarray) -> np.ndarray:
        """
        根据 HSV 阈值创建掩膜 / Create mask from HSV thresholds.

        Args:
            hsv_frame: HSV 图像 / HSV frame.

        Returns:
            np.ndarray: 二值掩膜 / Binary mask.
        """
        # lower_bound: HSV 下限 / HSV lower bound.
        lower_bound = np.array([self.Hmin, self.Smin, self.Vmin], dtype=np.uint8)
        # upper_bound: HSV 上限 / HSV upper bound.
        upper_bound = np.array([self.Hmax, self.Smax, self.Vmax], dtype=np.uint8)

        if self.Hmin <= self.Hmax:
            return cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # lower_mask: 色相回绕时的低段掩膜 / Low hue range mask for hue wrap-around.
        lower_mask = cv2.inRange(hsv_frame, np.array([0, self.Smin, self.Vmin], dtype=np.uint8), upper_bound)
        # upper_mask: 色相回绕时的高段掩膜 / High hue range mask for hue wrap-around.
        upper_mask = cv2.inRange(hsv_frame, lower_bound, np.array([179, self.Smax, self.Vmax], dtype=np.uint8))
        return cv2.bitwise_or(lower_mask, upper_mask)

    def apply_morphology(self, binary_mask: np.ndarray) -> np.ndarray:
        """
        对二值图做形态学滤波 / Apply morphology filtering to binary mask.

        Args:
            binary_mask: 原始二值图 / Raw binary mask.

        Returns:
            np.ndarray: 滤波后的二值图 / Filtered binary mask.
        """
        # kernel_size: 形态学核尺寸 / Morphology kernel size.
        kernel_size = max(1, int(self.morph_kernel_size))
        if kernel_size % 2 == 0:
            kernel_size += 1
        # morphology_kernel: 形态学核 / Morphology kernel.
        morphology_kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_OPEN, morphology_kernel)
        binary_mask = cv2.morphologyEx(binary_mask, cv2.MORPH_CLOSE, morphology_kernel)
        return binary_mask

    def update_line_state(self, line_result: LineTrackingResult) -> None:
        """
        更新路线状态 / Update line state.

        Args:
            line_result: 路线识别结果 / Line detection result.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        with self.line_state_lock:
            if line_result.is_line_found:
                self.line_lost_start_time_sec = None
                self.line_lost_timeout_reported = False
                # raw_center: 原始路线中心 / Raw line center.
                raw_center = (line_result.line_center_x, line_result.line_center_y)
                self.raw_line_center = raw_center
                if self.filtered_line_center is None:
                    self.filtered_line_center = raw_center
                else:
                    # old_x, old_y: 上一次滤波中心 / Previous filtered center.
                    old_x, old_y = self.filtered_line_center
                    # new_x, new_y: 当前检测中心 / Current detected center.
                    new_x, new_y = raw_center
                    self.filtered_line_center = (
                        old_x * (1.0 - self.target_smooth_alpha) + new_x * self.target_smooth_alpha,
                        old_y * (1.0 - self.target_smooth_alpha) + new_y * self.target_smooth_alpha,
                    )
                self.line_is_valid = True
                self.latest_line_update_time_sec = current_time_sec
                self.line_rectangle = line_result.line_rectangle
                self.line_area = line_result.line_area
                self.line_radius = line_result.line_radius
                if self.tracking_state == TRACKING_STATE_LOST:
                    self.tracking_state = TRACKING_STATE_TRACKING if self.auto_start_tracking else TRACKING_STATE_IDENTIFY
            else:
                if self.line_lost_start_time_sec is None:
                    self.line_lost_start_time_sec = current_time_sec
                # line_age_sec: 距离上一次有效路线的时间 / Age since last valid line.
                line_age_sec = current_time_sec - self.line_lost_start_time_sec
                if line_age_sec > self.line_lost_timeout_sec:
                    self.line_is_valid = False
                    self.raw_line_center = None
                    self.filtered_line_center = None
                    self.line_rectangle = (0, 0, 0, 0)
                    self.line_area = 0.0
                    self.line_radius = 0.0
                    if self.tracking_state == TRACKING_STATE_TRACKING:
                        self.tracking_state = TRACKING_STATE_LOST
                        self.line_pid_controller.reset()
                    if not self.line_lost_timeout_reported:
                        self.line_lost_timeout_reported = True
                        self.publish_zero_twist()
                        self.publish_beep(False)
                        self.get_logger().info(
                            "Line lost for %.2fs; stopping chassis and keeping follow_line_ptz_node active."
                            % line_age_sec
                        )

    def publish_chassis_control_command(self) -> None:
        """固定频率发布底盘控制命令 / Publish chassis control command at a fixed rate."""
        if not self.is_node_running:
            return

        if self.joy_is_active:
            self.publish_zero_twist()
            return

        with self.scan_state_lock:
            # local_obstacle_is_active: 本地障碍物触发标志 / Local obstacle active flag.
            local_obstacle_is_active = self.obstacle_is_active
            # local_obstacle_warning_count: 本地障碍物点数 / Local obstacle warning count.
            local_obstacle_warning_count = self.obstacle_warning_count

        if self.enable_obstacle_stop and local_obstacle_is_active:
            self.publish_zero_twist()
            self.publish_beep(True)
            self.throttled_debug_log("Obstacle stop active. warning_count=%d" % local_obstacle_warning_count)
            return
        self.publish_beep(False)

        if self.tracking_state != TRACKING_STATE_TRACKING:
            if self.tracking_state in (TRACKING_STATE_LOST, TRACKING_STATE_PAUSED) or self.lost_line_stop:
                self.publish_zero_twist()
            return

        with self.line_state_lock:
            # local_line_is_valid: 本地路线有效标志 / Local line valid flag.
            local_line_is_valid = self.line_is_valid
            # local_filtered_center: 本地滤波路线中心 / Local filtered line center.
            local_filtered_center = self.filtered_line_center
            # local_image_width: 本地图像宽度 / Local image width.
            local_image_width = self.image_width
            # local_latest_update_time_sec: 本地最近更新时间 / Local latest update time.
            local_latest_update_time_sec = self.latest_line_update_time_sec

        if (
            not local_line_is_valid
            or local_filtered_center is None
            or local_image_width <= 0
            or (time.perf_counter() - local_latest_update_time_sec) > self.line_lost_timeout_sec
        ):
            self.line_pid_controller.reset()
            self.publish_zero_twist()
            return

        # line_center_x: 路线中心 x / Line center x.
        line_center_x = local_filtered_center[0]
        # image_center_x: 图像中心 x / Image center x.
        image_center_x = local_image_width * 0.5
        # pixel_error_x: 像素误差 / Pixel error.
        pixel_error_x = line_center_x - image_center_x

        # twist_command: 底盘速度命令 / Chassis velocity command.
        twist_command = Twist()
        twist_command.linear.x = float(self.line_linear_speed)

        if abs(pixel_error_x) <= self.line_center_deadband_pixel:
            angular_command = 0.0
            self.line_pid_controller.reset()
        else:
            # pid_input_value: PID 输入值，保持旧程序除以 16 的默认比例 / PID input value.
            pid_input_value = pixel_error_x / self.line_error_divisor
            # pid_output_value: PID 输出值 / PID output value.
            pid_output_value = self.line_pid_controller.update(pid_input_value, 0.0)
            angular_command = -pid_output_value if self.reverse_angular_direction else pid_output_value
            angular_command = float(np.clip(angular_command, -self.max_angular_speed, self.max_angular_speed))

        twist_command.angular.z = angular_command
        self.cmd_vel_publisher.publish(twist_command)
        self.previous_twist_command = twist_command
        self.print_cmd_vel_if_needed(twist_command, pixel_error_x)

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

    def update_hsv_from_selected_roi(self, image_frame: np.ndarray) -> None:
        """
        根据鼠标框选 ROI 更新 HSV 阈值 / Update HSV thresholds from selected ROI.

        Args:
            image_frame: 当前图像 / Current frame.
        """
        self.roi_selection_pending = False
        # x1, y1: ROI 左上角 / ROI top-left.
        x1 = min(self.roi_selection_start[0], self.roi_selection_end[0])
        y1 = min(self.roi_selection_start[1], self.roi_selection_end[1])
        # x2, y2: ROI 右下角 / ROI bottom-right.
        x2 = max(self.roi_selection_start[0], self.roi_selection_end[0])
        y2 = max(self.roi_selection_start[1], self.roi_selection_end[1])
        x1 = int(np.clip(x1, 0, image_frame.shape[1] - 1))
        x2 = int(np.clip(x2, 0, image_frame.shape[1] - 1))
        y1 = int(np.clip(y1, 0, image_frame.shape[0] - 1))
        y2 = int(np.clip(y2, 0, image_frame.shape[0] - 1))
        if (x2 - x1) < MINIMUM_ROI_SIZE_PIXEL or (y2 - y1) < MINIMUM_ROI_SIZE_PIXEL:
            self.get_logger().warn("Selected ROI is too small; HSV update skipped.")
            return

        # roi_frame: 选中的 ROI 图像 / Selected ROI frame.
        roi_frame = image_frame[y1:y2, x1:x2]
        # hsv_roi: ROI HSV 图像 / ROI HSV frame.
        hsv_roi = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
        # h_channel, s_channel, v_channel: HSV 通道 / HSV channels.
        h_channel, s_channel, v_channel = cv2.split(hsv_roi)

        # h_low, h_high: H 通道范围 / H channel range.
        h_low = int(np.clip(np.percentile(h_channel, 5) - self.h_expand, 0, 179))
        h_high = int(np.clip(np.percentile(h_channel, 95) + self.h_expand, 0, 179))
        # s_low, s_high: S 通道范围 / S channel range.
        s_low = int(np.clip(np.percentile(s_channel, 5) - self.s_expand, 0, 255))
        s_high = int(np.clip(np.percentile(s_channel, 95) + self.s_expand, 0, 255))
        # v_low, v_high: V 通道范围 / V channel range.
        v_low = int(np.clip(np.percentile(v_channel, 5) - self.v_expand, 0, 255))
        v_high = int(np.clip(np.percentile(v_channel, 95) + self.v_expand, 0, 255))

        self.Hmin, self.Smin, self.Vmin = h_low, s_low, v_low
        self.Hmax, self.Smax, self.Vmax = h_high, s_high, v_high
        self.set_parameters([
            Parameter("Hmin", Parameter.Type.INTEGER, self.Hmin),
            Parameter("Smin", Parameter.Type.INTEGER, self.Smin),
            Parameter("Vmin", Parameter.Type.INTEGER, self.Vmin),
            Parameter("Hmax", Parameter.Type.INTEGER, self.Hmax),
            Parameter("Smax", Parameter.Type.INTEGER, self.Smax),
            Parameter("Vmax", Parameter.Type.INTEGER, self.Vmax),
        ])
        self.save_hsv_parameters_to_file_if_needed()
        self.tracking_state = TRACKING_STATE_IDENTIFY
        self.get_logger().info(
            "HSV updated from ROI: H[%d,%d] S[%d,%d] V[%d,%d]"
            % (self.Hmin, self.Hmax, self.Smin, self.Smax, self.Vmin, self.Vmax)
        )

    def initialize_debug_window_if_needed(self) -> None:
        """按需初始化调试窗口 / Initialize debug window if needed."""
        if not self.show_debug_window:
            return
        if self.debug_window_initialized:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, int(self.window_width), int(self.window_height))
        cv2.setMouseCallback(self.window_name, self.on_mouse_event)
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

        # display_image: 最终显示图像 / Final display image.
        display_image = self.latest_display_frame
        if self.show_binary_window and self.latest_binary_mask is not None:
            # binary_bgr: BGR 二值图 / BGR binary image.
            binary_bgr = cv2.cvtColor(self.latest_binary_mask, cv2.COLOR_GRAY2BGR)
            display_image = np.hstack((self.latest_display_frame, binary_bgr))
        cv2.imshow(self.window_name, display_image)
        # key_code: 键盘输入 / Keyboard input.
        key_code = cv2.waitKey(1) & 0xFF
        self.handle_keyboard_input(key_code)

    def on_mouse_event(self, event: int, x_coordinate: int, y_coordinate: int, flags: int, parameter: Any) -> None:
        """
        OpenCV 鼠标事件回调 / OpenCV mouse event callback.

        Args:
            event: 鼠标事件 / Mouse event.
            x_coordinate: 鼠标 x 坐标 / Mouse x coordinate.
            y_coordinate: 鼠标 y 坐标 / Mouse y coordinate.
            flags: 鼠标标志 / Mouse flags.
            parameter: 回调参数 / Callback parameter.
        """
        del flags, parameter
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_selecting = True
            self.tracking_state = TRACKING_STATE_SELECTING
            self.roi_selection_start = (x_coordinate, y_coordinate)
            self.roi_selection_end = (x_coordinate, y_coordinate)
        elif event == cv2.EVENT_MOUSEMOVE and self.mouse_selecting:
            self.roi_selection_end = (x_coordinate, y_coordinate)
        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_selecting = False
            self.roi_selection_end = (x_coordinate, y_coordinate)
            self.roi_selection_pending = True

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
            self.reset_line_following_state()
        elif key_code in (ord("i"), ord("I")):
            self.tracking_state = TRACKING_STATE_IDENTIFY
            self.publish_zero_twist()
            self.get_logger().info("Line following identify mode.")
        elif key_code == KEY_CODE_SPACE:
            if self.tracking_state == TRACKING_STATE_TRACKING:
                self.tracking_state = TRACKING_STATE_PAUSED
                self.publish_zero_twist()
                self.get_logger().info("Line following paused.")
            else:
                self.tracking_state = TRACKING_STATE_TRACKING
                self.line_pid_controller.reset()
                self.get_logger().info("Line following tracking enabled.")

    def reset_line_following_state(self) -> None:
        """重置巡线状态 / Reset line following state."""
        with self.line_state_lock:
            self.line_is_valid = False
            self.raw_line_center = None
            self.filtered_line_center = None
            self.line_rectangle = (0, 0, 0, 0)
            self.line_area = 0.0
            self.line_radius = 0.0
        self.tracking_state = TRACKING_STATE_IDENTIFY
        self.line_pid_controller.reset()
        self.publish_zero_twist()
        self.get_logger().info("Line following state reset. Drag ROI to update HSV or press SPACE to track.")

    def cancel_node(self) -> None:
        """取消节点运行 / Cancel node running."""
        self.is_node_running = False
        self.publish_zero_twist()
        self.publish_beep(False)
        self.get_logger().info("Follow line node requested to stop from keyboard.")
        # 参照 ptz-tracking-rules SKILL: q/ESC 使用 os._exit(0) 立即退出
        os._exit(0)

    def draw_line_detection(self, display_frame: np.ndarray, line_result: LineTrackingResult) -> None:
        """
        绘制路线识别结果 / Draw line detection result.

        Args:
            display_frame: 显示图像 / Display frame.
            line_result: 路线识别结果 / Line detection result.
        """
        # image_center_x: 图像中心 x / Image center x.
        image_center_x = int(display_frame.shape[1] * 0.5)
        cv2.line(display_frame, (image_center_x, 0), (image_center_x, display_frame.shape[0]), (255, 255, 0), 1)

        # roi_top_y: ROI 顶部 / ROI top.
        roi_top_y = int(np.clip(self.line_roi_top_ratio, 0.0, 1.0) * display_frame.shape[0])
        # roi_bottom_y: ROI 底部 / ROI bottom.
        roi_bottom_y = int(np.clip(self.line_roi_bottom_ratio, 0.0, 1.0) * display_frame.shape[0])
        cv2.rectangle(display_frame, (0, roi_top_y), (display_frame.shape[1] - 1, roi_bottom_y), (80, 80, 80), 1)

        if line_result.is_line_found:
            x_coord, y_coord, width, height = line_result.line_rectangle
            cv2.rectangle(display_frame, (x_coord, y_coord), (x_coord + width, y_coord + height), (0, 255, 0), 2)
            cv2.circle(display_frame, (int(line_result.line_center_x), int(line_result.line_center_y)), 5, (0, 0, 255), -1)
            if line_result.line_contour is not None:
                cv2.drawContours(display_frame, [line_result.line_contour], -1, (0, 180, 255), 2)

        with self.line_state_lock:
            if self.filtered_line_center is not None:
                cv2.circle(
                    display_frame,
                    (int(self.filtered_line_center[0]), int(self.filtered_line_center[1])),
                    7,
                    (255, 0, 255),
                    2,
                )

    def draw_roi_selection(self, display_frame: np.ndarray) -> None:
        """
        绘制鼠标选择 ROI / Draw mouse selection ROI.

        Args:
            display_frame: 显示图像 / Display frame.
        """
        if not self.mouse_selecting:
            return
        cv2.rectangle(display_frame, self.roi_selection_start, self.roi_selection_end, (255, 0, 0), 2)

    def draw_overlay_text(self, display_frame: np.ndarray) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            display_frame: 显示图像 / Display frame.
        """
        with self.line_state_lock:
            # center_text: 中心点文本 / Center text.
            center_text = "None" if self.filtered_line_center is None else "%.1f, %.1f" % self.filtered_line_center
            # area_text: 面积文本 / Area text.
            area_text = "%.1f" % self.line_area
        with self.scan_state_lock:
            # obstacle_text: 障碍物文本 / Obstacle text.
            obstacle_text = "ON(%d)" % self.obstacle_warning_count if self.obstacle_is_active else "OFF(%d)" % self.obstacle_warning_count

        # overlay_lines: 叠加文字行 / Overlay text lines.
        overlay_lines = [
            "State: %s | FPS: %.1f" % (self.tracking_state, self.measured_fps),
            "Line center: %s | area: %s" % (center_text, area_text),
            "HSV: H[%d,%d] S[%d,%d] V[%d,%d]" % (self.Hmin, self.Hmax, self.Smin, self.Smax, self.Vmin, self.Vmax),
            "Speed: %.2f m/s | obstacle: %s" % (self.line_linear_speed, obstacle_text),
            "Keys: SPACE track/pause, i identify, r reset, q quit",
        ]

        if self.overlay_show_background:
            # background_height: 背景高度 / Background height.
            background_height = self.overlay_line_height * len(overlay_lines) + 8
            cv2.rectangle(
                display_frame,
                (0, 0),
                (min(display_frame.shape[1], 640), background_height),
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

    def print_cmd_vel_if_needed(self, twist_command: Twist, pixel_error_x: float) -> None:
        """
        按需打印速度命令 / Print velocity command if needed.

        Args:
            twist_command: 速度命令 / Velocity command.
            pixel_error_x: 路线像素误差 / Line pixel error.
        """
        if not self.print_cmd_vel:
            return
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_cmd_print_time_sec < self.print_interval_sec:
            return
        self.get_logger().info(
            "cmd_vel linear.x=%.3f angular.z=%.3f error_x=%.1f"
            % (twist_command.linear.x, twist_command.angular.z, pixel_error_x)
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

            # 先让 ROS 更新参数，再从参数服务器刷新 / Refresh from ROS parameter server after accepting update.
            # 注意: rclpy 回调内参数值尚未写入，因此这里也直接根据 parameters 更新属性。
            self.apply_dynamic_parameters_to_attributes(parameters)

            if any(parameter.name in {
                "line_pid_kp", "line_pid_ki", "line_pid_kd", "pid_scale", "pid_integral_limit"
            } for parameter in parameters):
                self.line_pid_controller.update_gains(
                    self.line_pid_kp / self.pid_scale,
                    self.line_pid_ki / self.pid_scale,
                    self.line_pid_kd / self.pid_scale,
                    self.pid_integral_limit,
                )

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
            elif parameter_name == "show_binary_window":
                self.show_binary_window = bool(parameter_value)
            elif parameter_name == "enable_horizontal_flip":
                self.enable_horizontal_flip = bool(parameter_value)
            elif parameter_name == "enable_vertical_flip":
                self.enable_vertical_flip = bool(parameter_value)
            elif parameter_name == "rotate_image_code":
                self.rotate_image_code = str(parameter_value)
            elif parameter_name in {"Hmin", "Smin", "Vmin", "Hmax", "Smax", "Vmax"}:
                setattr(self, parameter_name, int(parameter_value))
            elif parameter_name in {"h_expand", "s_expand", "v_expand", "morph_kernel_size"}:
                setattr(self, parameter_name, int(parameter_value))
            elif parameter_name in {"line_roi_top_ratio", "line_roi_bottom_ratio", "line_min_area", "line_error_divisor", "line_linear_speed", "line_center_deadband_pixel", "max_angular_speed", "debug_log_interval_sec", "print_interval_sec"}:
                setattr(self, parameter_name, float(parameter_value))
            elif parameter_name == "line_lost_timeout_sec":
                self.line_lost_timeout_sec = max(float(parameter_value), MINIMUM_LINE_LOST_TIMEOUT_SEC)
            elif parameter_name == "target_smooth_alpha":
                self.target_smooth_alpha = self.clamp_alpha(float(parameter_value))
            elif parameter_name in {"line_pid_kp", "line_pid_ki", "line_pid_kd", "pid_integral_limit"}:
                setattr(self, parameter_name, float(parameter_value))
            elif parameter_name == "pid_scale":
                self.pid_scale = max(float(parameter_value), 1e-6)
            elif parameter_name == "control_rate_hz":
                self.control_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name == "image_processing_rate_hz":
                self.image_processing_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name in {"enable_obstacle_stop", "enable_beep", "reverse_angular_direction", "auto_start_tracking", "lost_line_stop", "publish_initial_ptz_command", "print_cmd_vel", "enable_rqt_dynamic_parameters", "save_hsv_to_file", "load_hsv_from_file"}:
                setattr(self, parameter_name, bool(parameter_value))
            elif parameter_name in {"obstacle_min_valid_distance", "obstacle_response_distance", "obstacle_check_angle_deg", "initial_horizontal_deg", "initial_pitch_deg", "initial_publish_interval_sec"}:
                setattr(self, parameter_name, float(parameter_value))
            elif parameter_name in {"obstacle_warning_count_threshold", "initial_publish_count"}:
                setattr(self, parameter_name, int(parameter_value))
            elif parameter_name == "hsv_config_file":
                self.hsv_config_file = str(parameter_value)

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            bool: 销毁结果 / Destroy result.
        """
        self.is_node_running = False
        if not self.shutdown_requested:
            # 正常退出时主动停车；已 shutdown 时 context 可能已失效，跳过
            try:
                self.publish_zero_twist()
            except Exception:
                pass
            try:
                self.publish_beep(False)
            except Exception:
                pass
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
    # follow_line_node: 巡线节点 / Line following node.
    follow_line_node = FollowLinePtzNode()
    try:
        rclpy.spin(follow_line_node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        follow_line_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
