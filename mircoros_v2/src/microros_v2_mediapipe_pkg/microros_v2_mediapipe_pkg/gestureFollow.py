#!/usr/bin/env python3
# encoding: utf-8
"""
gestureFollow.py

中文说明:
    ROS2 Humble 手势中心车体 PD 跟随节点。
    本版本只订阅原始图像话题 /camera/image_raw，不使用深度相机。
    图像会先缩放到固定计算尺寸，再进行手势检测和控制计算；显示尺寸只用于 OpenCV 窗口显示。
    云台水平角和俯仰角固定发布，手掌中心在画面中的水平/竖直误差只通过车体 /cmd_vel 的 PD 控制调整。

English Description:
    ROS2 Humble gesture-center chassis PD following node.
    This version subscribes only to /camera/image_raw and does not use a depth camera.
    The image is resized to a fixed processing size before gesture detection and control computation;
    the display size only affects the OpenCV window. The PTZ horizontal and pitch angles are fixed,
    and only 4-finger or 5-finger gestures enable chassis PD following from hand-center image errors.
"""

from __future__ import annotations

import os

# MEDIAPIPE_DISABLE_TF: 禁用 MediaPipe TensorFlow 初始化 / Disable MediaPipe TensorFlow initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
# MEDIAPIPE_DISABLE_GPU: 禁用 MediaPipe GPU 初始化 / Disable MediaPipe GPU initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .media_common import handDetector


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "gesture_follow_node"

# CONTROL_STATE_IDENTIFY: 只识别不控制状态 / Identify-only state.
CONTROL_STATE_IDENTIFY = "identify"

# CONTROL_STATE_TRACKING: 手势跟随控制状态 / Active gesture following state.
CONTROL_STATE_TRACKING = "tracking"

# CONTROL_STATE_LOST: 手势丢失状态 / Gesture lost state.
CONTROL_STATE_LOST = "lost"

# CONTROL_STATE_PAUSED: 暂停状态 / Paused state.
CONTROL_STATE_PAUSED = "paused"

# CONTROL_STATE_GESTURE_STOP: 非跟随手势停车状态 / Stop state for non-follow gestures.
CONTROL_STATE_GESTURE_STOP = "gesture_stop"

# KEY_CODE_SPACE: 空格键编码 / Space key code.
KEY_CODE_SPACE = 32

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# MINIMUM_TIMER_RATE_HZ: 定时器最小频率 / Minimum timer frequency.
MINIMUM_TIMER_RATE_HZ = 1.0

# MINIMUM_PROCESSING_SIZE_PIXEL: 最小计算图像尺寸 / Minimum processing image size.
MINIMUM_PROCESSING_SIZE_PIXEL = 160

# DYNAMIC_PARAMETER_NAMES: 支持动态修改的参数 / Parameters that can be updated dynamically.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "processing_width",
    "processing_height",
    "display_width",
    "display_height",
    "overlay_font_scale",
    "overlay_line_height",
    "overlay_show_background",
    "overlay_show_legend",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "fixed_horizontal_deg",
    "fixed_pitch_deg",
    "publish_initial_ptz_command",
    "initial_publish_count",
    "initial_publish_interval_sec",
    "publish_ptz_hold_command",
    "ptz_hold_rate_hz",
    "horizontal_deadband_pixel",
    "vertical_deadband_pixel",
    "horizontal_pd_kp",
    "horizontal_pd_kd",
    "vertical_pd_kp",
    "vertical_pd_kd",
    "max_linear_x",
    "max_angular_z",
    "linear_velocity_deadband",
    "angular_velocity_deadband",
    "reverse_body_angular_direction",
    "reverse_linear_from_vertical",
    "enable_body_angular_control",
    "enable_body_linear_control",
    "control_rate_hz",
    "image_processing_rate_hz",
    "hand_lost_timeout_sec",
    "stop_when_hand_lost",
    "auto_start_tracking",
    "print_cmd_vel",
    "print_interval_sec",
    "enable_rqt_dynamic_parameters",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters that require node restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "rgb_topic",
    "cmd_vel_topic",
    "joy_state_topic",
    "ptz_command_topic",
    "window_name",
})


@dataclass
class HandGestureState:
    """手势识别状态 / Hand gesture detection state."""

    # is_hand_valid: 是否检测到手 / Whether a hand is detected.
    is_hand_valid: bool = False
    # hand_center_x: 手势中心 x 像素坐标 / Gesture center x coordinate.
    hand_center_x: float = 0.0
    # hand_center_y: 手势中心 y 像素坐标 / Gesture center y coordinate.
    hand_center_y: float = 0.0
    # hand_width: 手部框宽度 / Hand bounding width.
    hand_width: float = 0.0
    # hand_height: 手部框高度 / Hand bounding height.
    hand_height: float = 0.0
    # gesture_name: 手势名称 / Gesture name.
    gesture_name: str = "None"
    # finger_code: 手指状态代码 / Finger state code.
    finger_code: str = "None"
    # finger_count: 伸出手指数 / Number of raised fingers.
    finger_count: int = 0
    # is_follow_gesture: 是否为跟随手势，4 指或 5 指为 True / Whether this is a follow gesture, true for 4 or 5 fingers.
    is_follow_gesture: bool = False
    # update_time_sec: 更新时间 / Update timestamp.
    update_time_sec: float = 0.0


class DiscretePdController:
    """离散 PD 控制器 / Discrete PD controller."""

    def __init__(self, proportional_gain: float, derivative_gain: float) -> None:
        """
        初始化 PD 控制器 / Initialize PD controller.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            derivative_gain: 微分增益 / Derivative gain.
        """
        # proportional_gain: 比例增益 / Proportional gain.
        self.proportional_gain = float(proportional_gain)
        # derivative_gain: 微分增益 / Derivative gain.
        self.derivative_gain = float(derivative_gain)
        # previous_error: 上一次误差 / Previous error.
        self.previous_error = 0.0
        # previous_time_sec: 上一次计算时间 / Previous update time.
        self.previous_time_sec: Optional[float] = None

    def update_gains(self, proportional_gain: float, derivative_gain: float) -> None:
        """
        更新 PD 参数 / Update PD gains.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            derivative_gain: 微分增益 / Derivative gain.
        """
        self.proportional_gain = float(proportional_gain)
        self.derivative_gain = float(derivative_gain)

    def reset(self) -> None:
        """重置 PD 状态 / Reset PD state."""
        self.previous_error = 0.0
        self.previous_time_sec = None

    def update(self, error_value: float) -> float:
        """
        根据误差计算 PD 输出 / Compute PD output from error.

        Args:
            error_value: 当前误差 / Current error.

        Returns:
            float: PD 输出 / PD output.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if self.previous_time_sec is None:
            self.previous_time_sec = current_time_sec
            self.previous_error = float(error_value)
            return self.proportional_gain * float(error_value)

        # delta_time_sec: 控制周期 / Control period.
        delta_time_sec = max(current_time_sec - self.previous_time_sec, 1e-6)
        # derivative_error: 误差变化率 / Error derivative.
        derivative_error = (float(error_value) - self.previous_error) / delta_time_sec
        self.previous_time_sec = current_time_sec
        self.previous_error = float(error_value)
        return self.proportional_gain * float(error_value) + self.derivative_gain * derivative_error


class GestureFollowNode(Node):
    """手势中心车体 PD 跟随节点 / Gesture-center chassis PD following node."""

    def __init__(self) -> None:
        """初始化节点 / Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # image_lock: 图像缓存互斥锁 / Image cache mutex.
        self.image_lock = threading.Lock()
        # gesture_state_lock: 手势状态互斥锁 / Gesture state mutex.
        self.gesture_state_lock = threading.Lock()

        # latest_image_msg: 最新原始图像 / Latest raw image.
        self.latest_image_msg: Optional[Image] = None
        # latest_image_sequence_id: 最新图像序号 / Latest image sequence ID.
        self.latest_image_sequence_id = 0
        # processed_image_sequence_id: 已处理图像序号 / Processed image sequence ID.
        self.processed_image_sequence_id = 0

        # control_state: 当前控制状态 / Current control state.
        self.control_state = CONTROL_STATE_IDENTIFY
        # control_is_enabled: 是否启用自动控制 / Whether automatic control is enabled.
        self.control_is_enabled = False
        # joy_is_active: 手柄接管状态 / Joystick override state.
        self.joy_is_active = False
        # is_node_running: 节点运行标志 / Node running flag.
        self.is_node_running = True

        # hand_state: 当前手势状态 / Current hand gesture state.
        self.hand_state = HandGestureState()
        # latest_display_frame: 最新显示图像 / Latest display frame.
        self.latest_display_frame: Optional[np.ndarray] = None
        # measured_fps: 测量 FPS / Measured FPS.
        self.measured_fps = 0.0
        # fps_last_time_sec: FPS 上一次统计时间 / Previous FPS statistics time.
        self.fps_last_time_sec = time.perf_counter()
        # fps_frame_count: FPS 帧数 / FPS frame count.
        self.fps_frame_count = 0
        # display_window_initialized: 显示窗口初始化标志 / Display window initialized flag.
        self.display_window_initialized = False
        # previous_twist_command: 上一次速度命令 / Previous velocity command.
        self.previous_twist_command = Twist()
        # last_cmd_print_time_sec: 上一次速度打印时间 / Previous command print time.
        self.last_cmd_print_time_sec = 0.0

        self.declare_and_get_parameters()
        self.control_is_enabled = self.auto_start_tracking
        self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY

        # horizontal_pd_controller: 水平误差车体转向 PD 控制器 / PD controller for angular.z from horizontal error.
        self.horizontal_pd_controller = DiscretePdController(self.horizontal_pd_kp, self.horizontal_pd_kd)
        # vertical_pd_controller: 竖直误差前后移动 PD 控制器 / PD controller for linear.x from vertical error.
        self.vertical_pd_controller = DiscretePdController(self.vertical_pd_kp, self.vertical_pd_kd)

        # qos_profile: 图像订阅 QoS / QoS profile for image subscription.
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # image_bridge: 图像转换工具 / Image conversion helper.
        self.image_bridge = CvBridge()
        # hand_detector: 手势检测器 / Hand detector.
        self.hand_detector = handDetector(detectorCon=0.8)

        # image_subscription: 原始图像订阅器 / Raw image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            self.qos_profile,
        )
        # joy_state_subscription: 手柄状态订阅器 / Joystick state subscriber.
        self.joy_state_subscription = self.create_subscription(Bool, self.joy_state_topic, self.joy_state_callback, 10)
        # cmd_vel_qos: 底盘速度 QoS（与 ESP32 固件端 BEST_EFFORT 对齐） / Chassis velocity QoS.
        self.cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # cmd_vel_publisher: 底盘速度发布器 / Chassis velocity publisher.
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, self.cmd_vel_qos)
        # ptz_command_publisher: 云台角度发布器 / PTZ angle publisher.
        self.ptz_command_publisher = self.create_publisher(Vector3, self.ptz_command_topic, 10)

        # image_processing_timer: 图像处理定时器 / Image processing timer.
        self.image_processing_timer = self.create_timer(
            1.0 / max(self.image_processing_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self.process_latest_image,
        )
        # control_timer: 控制定时器 / Control timer.
        self.control_timer = self.create_timer(
            1.0 / max(self.control_rate_hz, MINIMUM_TIMER_RATE_HZ),
            self.publish_control_command,
        )

        # initial_ptz_publish_counter: 初始云台发布计数 / Initial PTZ publish counter.
        self.initial_ptz_publish_counter = 0
        # initial_ptz_timer: 初始云台发布定时器 / Initial PTZ publish timer.
        self.initial_ptz_timer = None
        if self.publish_initial_ptz_command:
            self.initial_ptz_timer = self.create_timer(
                max(self.initial_publish_interval_sec, 0.02),
                self.publish_initial_ptz_command_callback,
            )
        # ptz_hold_timer: 云台固定角保持定时器 / Fixed PTZ hold timer.
        self.ptz_hold_timer = None
        if self.publish_ptz_hold_command:
            self.ptz_hold_timer = self.create_timer(
                1.0 / max(self.ptz_hold_rate_hz, MINIMUM_TIMER_RATE_HZ),
                self.publish_fixed_ptz_angle,
            )

        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(self.on_dynamic_parameter_update)

        self.get_logger().info(
            "Gesture follow node started. rgb_topic=%s, cmd_vel_topic=%s, fixed_ptz=(%.1f, %.1f), PD h=(%.4f, %.4f), v=(%.4f, %.4f)"
            % (
                self.rgb_topic,
                self.cmd_vel_topic,
                self.fixed_horizontal_deg,
                self.fixed_pitch_deg,
                self.horizontal_pd_kp,
                self.horizontal_pd_kd,
                self.vertical_pd_kp,
                self.vertical_pd_kd,
            )
        )

    def declare_and_get_parameters(self) -> None:
        """声明并读取参数 / Declare and get parameters."""
        # 话题参数 / Topic parameters.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_state_topic", "/JoyState")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "gesture_follow")

        # 图像和显示参数 / Image and display parameters.
        self.declare_parameter("processing_width", 640)
        self.declare_parameter("processing_height", 480)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 13)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("overlay_show_legend", False)
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        # 固定云台参数 / Fixed PTZ parameters.
        self.declare_parameter("fixed_horizontal_deg", 0.0)
        self.declare_parameter("fixed_pitch_deg", 0.0)
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("publish_ptz_hold_command", True)
        self.declare_parameter("ptz_hold_rate_hz", 2.0)

        # 车体 PD 控制参数 / Chassis PD control parameters.
        self.declare_parameter("horizontal_deadband_pixel", 20.0)
        self.declare_parameter("vertical_deadband_pixel", 20.0)
        self.declare_parameter("horizontal_pd_kp", 0.006)
        self.declare_parameter("horizontal_pd_kd", 0.0015)
        self.declare_parameter("vertical_pd_kp", 0.002)
        self.declare_parameter("vertical_pd_kd", 0.0005)
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_angular_z", 1.2)
        self.declare_parameter("linear_velocity_deadband", 0.02)
        self.declare_parameter("angular_velocity_deadband", 0.03)
        self.declare_parameter("reverse_body_angular_direction", False)
        self.declare_parameter("reverse_linear_from_vertical", True)
        self.declare_parameter("enable_body_angular_control", True)
        self.declare_parameter("enable_body_linear_control", True)

        # 定时器和状态参数 / Timer and state parameters.
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)
        self.declare_parameter("hand_lost_timeout_sec", 0.5)
        self.declare_parameter("stop_when_hand_lost", True)
        self.declare_parameter("auto_start_tracking", True)
        self.declare_parameter("print_cmd_vel", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

        self.refresh_parameters_from_ros()

    def refresh_parameters_from_ros(self) -> None:
        """从 ROS 参数服务器刷新参数 / Refresh parameters from ROS parameter server."""
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self.ptz_command_topic = str(self.get_parameter("ptz_command_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.processing_width = max(int(self.get_parameter("processing_width").value), MINIMUM_PROCESSING_SIZE_PIXEL)
        self.processing_height = max(int(self.get_parameter("processing_height").value), MINIMUM_PROCESSING_SIZE_PIXEL)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_show_background = bool(self.get_parameter("overlay_show_background").value)
        self.overlay_show_legend = bool(self.get_parameter("overlay_show_legend").value)
        self.enable_horizontal_flip = bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)

        self.fixed_horizontal_deg = float(self.get_parameter("fixed_horizontal_deg").value)
        self.fixed_pitch_deg = float(self.get_parameter("fixed_pitch_deg").value)
        self.publish_initial_ptz_command = bool(self.get_parameter("publish_initial_ptz_command").value)
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)
        self.publish_ptz_hold_command = bool(self.get_parameter("publish_ptz_hold_command").value)
        self.ptz_hold_rate_hz = max(float(self.get_parameter("ptz_hold_rate_hz").value), MINIMUM_TIMER_RATE_HZ)

        self.horizontal_deadband_pixel = float(self.get_parameter("horizontal_deadband_pixel").value)
        self.vertical_deadband_pixel = float(self.get_parameter("vertical_deadband_pixel").value)
        self.horizontal_pd_kp = float(self.get_parameter("horizontal_pd_kp").value)
        self.horizontal_pd_kd = float(self.get_parameter("horizontal_pd_kd").value)
        self.vertical_pd_kp = float(self.get_parameter("vertical_pd_kp").value)
        self.vertical_pd_kd = float(self.get_parameter("vertical_pd_kd").value)
        self.max_linear_x = abs(float(self.get_parameter("max_linear_x").value))
        self.max_angular_z = abs(float(self.get_parameter("max_angular_z").value))
        self.linear_velocity_deadband = abs(float(self.get_parameter("linear_velocity_deadband").value))
        self.angular_velocity_deadband = abs(float(self.get_parameter("angular_velocity_deadband").value))
        self.reverse_body_angular_direction = bool(self.get_parameter("reverse_body_angular_direction").value)
        self.reverse_linear_from_vertical = bool(self.get_parameter("reverse_linear_from_vertical").value)
        self.enable_body_angular_control = bool(self.get_parameter("enable_body_angular_control").value)
        self.enable_body_linear_control = bool(self.get_parameter("enable_body_linear_control").value)

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self.image_processing_rate_hz = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self.hand_lost_timeout_sec = max(float(self.get_parameter("hand_lost_timeout_sec").value), 0.05)
        self.stop_when_hand_lost = bool(self.get_parameter("stop_when_hand_lost").value)
        self.auto_start_tracking = bool(self.get_parameter("auto_start_tracking").value)
        self.print_cmd_vel = bool(self.get_parameter("print_cmd_vel").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
        self.enable_rqt_dynamic_parameters = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

    def image_callback(self, image_msg: Image) -> None:
        """
        缓存最新原始图像 / Cache latest raw image.

        Args:
            image_msg: 原始图像消息 / Raw image message.
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
            self.reset_pd_controllers()
            self.publish_zero_twist()

    def publish_initial_ptz_command_callback(self) -> None:
        """重复发布初始固定云台角 / Repeatedly publish initial fixed PTZ angle."""
        if self.initial_ptz_publish_counter >= self.initial_publish_count:
            if self.initial_ptz_timer is not None:
                self.destroy_timer(self.initial_ptz_timer)
                self.initial_ptz_timer = None
            return
        self.publish_fixed_ptz_angle()
        self.initial_ptz_publish_counter += 1

    def publish_fixed_ptz_angle(self) -> None:
        """发布固定云台角度 / Publish fixed PTZ angle."""
        # ptz_command_msg: 云台角度消息 / PTZ angle message.
        ptz_command_msg = Vector3()
        ptz_command_msg.x = float(self.fixed_horizontal_deg)
        ptz_command_msg.y = float(self.fixed_pitch_deg)
        ptz_command_msg.z = 0.0
        self.ptz_command_publisher.publish(ptz_command_msg)

    def process_latest_image(self) -> None:
        """处理最新图像 / Process latest image."""
        with self.image_lock:
            if self.latest_image_msg is None:
                return
            # local_image_msg: 本次处理的图像消息 / Local image message for processing.
            local_image_msg = self.latest_image_msg
            # local_sequence_id: 本次图像序号 / Local image sequence ID.
            local_sequence_id = self.latest_image_sequence_id

        if local_sequence_id == self.processed_image_sequence_id:
            self.update_debug_window_if_needed()
            return
        self.processed_image_sequence_id = local_sequence_id

        # decoded_frame: 解码图像 / Decoded image.
        decoded_frame = self.decode_image(local_image_msg)
        if decoded_frame is None:
            return

        # processed_frame: 固定计算尺寸图像 / Fixed-size processing frame.
        processed_frame = self.prepare_image_frame(decoded_frame)
        # detection_frame: 绘制检测结果的图像 / Frame with detection drawings.
        detection_frame = processed_frame.copy()
        # hand_state: 当前识别到的手势状态 / Current detected hand state.
        hand_state = self.detect_hand_gesture(detection_frame)
        self.update_hand_state(hand_state)
        self.draw_control_guides(detection_frame)
        self.draw_overlay_text(detection_frame, hand_state)
        self.latest_display_frame = detection_frame
        self.update_fps_statistics()
        self.update_debug_window_if_needed()

    def decode_image(self, image_msg: Image) -> Optional[np.ndarray]:
        """
        解码原始图像 / Decode raw image.

        Args:
            image_msg: 原始图像消息 / Raw image message.

        Returns:
            Optional[np.ndarray]: BGR 图像 / BGR image.
        """
        try:
            # decoded_frame: 解码后的图像 / Decoded frame.
            decoded_frame = self.image_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
            return decoded_frame
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().error("Failed to decode raw image: %s" % exception)
            return None

    def prepare_image_frame(self, image_frame: np.ndarray) -> np.ndarray:
        """
        生成固定计算尺寸图像 / Prepare fixed-size processing image.

        Args:
            image_frame: 原始图像 / Raw image.

        Returns:
            np.ndarray: 处理后的图像 / Processed image.
        """
        # resized_frame: 固定计算尺寸图像 / Fixed-size frame.
        resized_frame = cv.resize(image_frame, (int(self.processing_width), int(self.processing_height)))
        if self.enable_horizontal_flip:
            resized_frame = cv.flip(resized_frame, 1)
        if self.enable_vertical_flip:
            resized_frame = cv.flip(resized_frame, 0)
        if self.rotate_image_code == "90_clockwise":
            resized_frame = cv.rotate(resized_frame, cv.ROTATE_90_CLOCKWISE)
        elif self.rotate_image_code == "90_counterclockwise":
            resized_frame = cv.rotate(resized_frame, cv.ROTATE_90_COUNTERCLOCKWISE)
        elif self.rotate_image_code == "180":
            resized_frame = cv.rotate(resized_frame, cv.ROTATE_180)
        return resized_frame

    def detect_hand_gesture(self, image_frame: np.ndarray) -> HandGestureState:
        """
        检测手势中心 / Detect gesture center.

        Args:
            image_frame: BGR 图像 / BGR image.

        Returns:
            HandGestureState: 手势状态 / Hand gesture state.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        try:
            self.hand_detector.findHands(image_frame, draw=True)
        except TypeError:
            self.hand_detector.findHands(image_frame)

        if len(self.hand_detector.lmList) == 0:
            return HandGestureState(False, update_time_sec=current_time_sec)

        # hand_center_x, hand_center_y: 第 9 号关键点作为手势中心 / Landmark 9 as gesture center.
        try:
            hand_center_x, hand_center_y = self.hand_detector.findPoint(9)
        except Exception:  # pylint: disable=broad-except
            # landmark_9: 第 9 号关键点 / Landmark 9.
            landmark_9 = self.hand_detector.lmList[9]
            hand_center_x = float(landmark_9[1])
            hand_center_y = float(landmark_9[2])

        # landmark_x_values: 手部 x 坐标列表 / Hand x coordinate list.
        landmark_x_values = [float(landmark[1]) for landmark in self.hand_detector.lmList]
        # landmark_y_values: 手部 y 坐标列表 / Hand y coordinate list.
        landmark_y_values = [float(landmark[2]) for landmark in self.hand_detector.lmList]
        # hand_width: 手部宽度 / Hand width.
        hand_width = max(landmark_x_values) - min(landmark_x_values)
        # hand_height: 手部高度 / Hand height.
        hand_height = max(landmark_y_values) - min(landmark_y_values)
        # gesture_name: 手势名称 / Gesture name.
        gesture_name = self.get_current_gesture_name()
        # finger_code, finger_count, is_follow_gesture: 手指状态 / Finger state.
        finger_code, finger_count, is_follow_gesture = self.get_current_finger_state()

        cv.circle(image_frame, (int(hand_center_x), int(hand_center_y)), 6, (0, 255, 255), -1)
        cv.rectangle(
            image_frame,
            (int(min(landmark_x_values)), int(min(landmark_y_values))),
            (int(max(landmark_x_values)), int(max(landmark_y_values))),
            (0, 255, 0),
            1,
        )
        return HandGestureState(
            True,
            float(hand_center_x),
            float(hand_center_y),
            float(hand_width),
            float(hand_height),
            gesture_name,
            finger_code,
            finger_count,
            is_follow_gesture,
            current_time_sec,
        )

    def get_current_gesture_name(self) -> str:
        """
        获取当前手势名称 / Get current gesture name.

        Returns:
            str: 手势名称 / Gesture name.
        """
        try:
            return str(self.hand_detector.get_gesture())
        except Exception:  # pylint: disable=broad-except
            return "Unknown"

    def get_current_finger_state(self) -> Tuple[str, int, bool]:
        """
        获取当前手指状态 / Get current finger state.

        Returns:
            Tuple[str, int, bool]: 手指代码、伸出手指数、是否允许跟随 / Finger code, raised finger count, whether follow is enabled.
        """
        try:
            # fingers: 手指伸展状态列表，通常为 [拇指, 食指, 中指, 无名指, 小指] / Finger up state list.
            fingers = self.hand_detector.fingersUp()
            if fingers is None:
                return "Unknown", 0, False
            # normalized_fingers: 归一化手指状态 / Normalized finger state.
            normalized_fingers = [1 if int(value) != 0 else 0 for value in fingers]
            # finger_count: 伸出手指数 / Raised finger count.
            finger_count = int(sum(normalized_fingers))
            # finger_code: 手指状态代码 / Finger state code.
            finger_code = "[" + ",".join(str(value) for value in normalized_fingers) + "]"
            # is_follow_gesture: 只有 4 指或 5 指允许跟随 / Only 4 or 5 fingers enables following.
            is_follow_gesture = finger_count in (4, 5)
            return finger_code, finger_count, is_follow_gesture
        except Exception:  # pylint: disable=broad-except
            return "Unknown", 0, False

    def update_hand_state(self, new_hand_state: HandGestureState) -> None:
        """
        更新手势状态 / Update hand state.

        Args:
            new_hand_state: 新手势状态 / New hand state.
        """
        with self.gesture_state_lock:
            if new_hand_state.is_hand_valid:
                self.hand_state = new_hand_state
                self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            else:
                if time.perf_counter() - self.hand_state.update_time_sec > self.hand_lost_timeout_sec:
                    self.hand_state = new_hand_state
                    if self.control_is_enabled:
                        self.control_state = CONTROL_STATE_LOST

    def publish_control_command(self) -> None:
        """固定频率发布控制命令 / Publish control commands at a fixed rate."""
        if not self.is_node_running:
            return

        if self.joy_is_active:
            self.reset_pd_controllers()
            self.publish_zero_twist()
            return

        if not self.control_is_enabled:
            self.reset_pd_controllers()
            self.publish_zero_twist()
            return

        with self.gesture_state_lock:
            # local_hand_state: 本地手势状态 / Local hand state.
            local_hand_state = self.hand_state

        # hand_age_sec: 手势数据年龄 / Hand state age.
        hand_age_sec = time.perf_counter() - local_hand_state.update_time_sec
        if (not local_hand_state.is_hand_valid or hand_age_sec > self.hand_lost_timeout_sec) and self.stop_when_hand_lost:
            self.control_state = CONTROL_STATE_LOST
            self.reset_pd_controllers()
            self.publish_zero_twist()
            return

        if not local_hand_state.is_follow_gesture:
            self.control_state = CONTROL_STATE_GESTURE_STOP
            self.reset_pd_controllers()
            self.publish_zero_twist()
            return

        # image_center_x: 计算图像中心 x / Processing image center x.
        image_center_x = self.processing_width * 0.5
        # image_center_y: 计算图像中心 y / Processing image center y.
        image_center_y = self.processing_height * 0.5
        # horizontal_error_pixel: 水平误差 / Horizontal pixel error.
        horizontal_error_pixel = local_hand_state.hand_center_x - image_center_x
        # vertical_error_pixel: 竖直误差 / Vertical pixel error.
        vertical_error_pixel = local_hand_state.hand_center_y - image_center_y

        # twist_command: 速度命令 / Velocity command.
        twist_command = Twist()

        if self.enable_body_angular_control and abs(horizontal_error_pixel) > self.horizontal_deadband_pixel:
            # angular_command: 车体角速度命令 / Chassis angular velocity command.
            angular_command = -self.horizontal_pd_controller.update(horizontal_error_pixel)
            if self.reverse_body_angular_direction:
                angular_command = -angular_command
            if abs(angular_command) < self.angular_velocity_deadband:
                angular_command = 0.0
            twist_command.angular.z = float(np.clip(angular_command, -self.max_angular_z, self.max_angular_z))
        else:
            self.horizontal_pd_controller.reset()
            twist_command.angular.z = 0.0

        if self.enable_body_linear_control and abs(vertical_error_pixel) > self.vertical_deadband_pixel:
            # linear_command: 车体线速度命令 / Chassis linear velocity command.
            linear_command = self.vertical_pd_controller.update(vertical_error_pixel)
            if self.reverse_linear_from_vertical:
                linear_command = -linear_command
            if abs(linear_command) < self.linear_velocity_deadband:
                linear_command = 0.0
            twist_command.linear.x = float(np.clip(linear_command, -self.max_linear_x, self.max_linear_x))
        else:
            self.vertical_pd_controller.reset()
            twist_command.linear.x = 0.0

        self.cmd_vel_publisher.publish(twist_command)
        self.previous_twist_command = twist_command
        self.control_state = CONTROL_STATE_TRACKING
        self.print_cmd_vel_if_needed(twist_command, horizontal_error_pixel, vertical_error_pixel)

    def reset_pd_controllers(self) -> None:
        """重置 PD 控制器 / Reset PD controllers."""
        self.horizontal_pd_controller.reset()
        self.vertical_pd_controller.reset()

    def publish_zero_twist(self) -> None:
        """发布零速度 / Publish zero velocity."""
        # zero_twist_msg: 零速度消息 / Zero velocity message.
        zero_twist_msg = Twist()
        self.cmd_vel_publisher.publish(zero_twist_msg)
        self.previous_twist_command = zero_twist_msg

    def initialize_display_window_if_needed(self) -> None:
        """初始化显示窗口 / Initialize display window."""
        if self.display_window_initialized:
            return
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, int(self.display_width), int(self.display_height))
        self.display_window_initialized = True

    def update_debug_window_if_needed(self) -> None:
        """刷新调试窗口 / Update debug window."""
        if self.latest_display_frame is None:
            return
        self.initialize_display_window_if_needed()
        # display_frame: 放大显示图像 / Resized display frame.
        display_frame = cv.resize(self.latest_display_frame, (int(self.display_width), int(self.display_height)))
        cv.imshow(self.window_name, display_frame)
        # key_code: 键盘输入 / Keyboard input.
        key_code = cv.waitKey(1) & 0xFF
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
            self.reset_tracking_state()
        elif key_code in (ord("i"), ord("I")):
            self.control_is_enabled = False
            self.control_state = CONTROL_STATE_IDENTIFY
            self.reset_pd_controllers()
            self.publish_zero_twist()
        elif key_code == KEY_CODE_SPACE:
            if self.control_is_enabled:
                self.control_is_enabled = False
                self.control_state = CONTROL_STATE_PAUSED
                self.reset_pd_controllers()
                self.publish_zero_twist()
            else:
                self.control_is_enabled = True
                self.control_state = CONTROL_STATE_TRACKING
                self.reset_pd_controllers()

    def reset_tracking_state(self) -> None:
        """重置跟随状态 / Reset tracking state."""
        with self.gesture_state_lock:
            self.hand_state = HandGestureState()
        self.reset_pd_controllers()
        self.control_state = CONTROL_STATE_IDENTIFY
        self.control_is_enabled = False
        self.publish_zero_twist()
        self.publish_fixed_ptz_angle()
        self.get_logger().info("Gesture following state reset.")

    def cancel_node(self) -> None:
        """取消节点运行 / Cancel node running."""
        self.is_node_running = False
        self.reset_pd_controllers()
        self.publish_zero_twist()
        self.publish_fixed_ptz_angle()
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        self.get_logger().info("Gesture follow node requested to stop from keyboard.")
        os._exit(0)  # pylint: disable=protected-access

    def draw_control_guides(self, frame: np.ndarray) -> None:
        """
        绘制控制参考线 / Draw control guide lines.

        Args:
            frame: 显示图像 / Display frame.
        """
        # center_x: 图像中心 x / Image center x.
        center_x = int(self.processing_width * 0.5)
        # center_y: 图像中心 y / Image center y.
        center_y = int(self.processing_height * 0.5)
        cv.line(frame, (center_x, 0), (center_x, self.processing_height), (255, 255, 0), 1)
        cv.line(frame, (0, center_y), (self.processing_width, center_y), (255, 255, 0), 1)
        cv.rectangle(
            frame,
            (int(center_x - self.horizontal_deadband_pixel), int(center_y - self.vertical_deadband_pixel)),
            (int(center_x + self.horizontal_deadband_pixel), int(center_y + self.vertical_deadband_pixel)),
            (0, 180, 255),
            1,
        )

    def draw_overlay_text(self, frame: np.ndarray, current_hand_state: HandGestureState) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            frame: 显示图像 / Display frame.
            current_hand_state: 当前手势状态 / Current hand state.
        """
        # horizontal_error: 水平误差文本 / Horizontal error text.
        horizontal_error = current_hand_state.hand_center_x - self.processing_width * 0.5 if current_hand_state.is_hand_valid else 0.0
        # vertical_error: 竖直误差文本 / Vertical error text.
        vertical_error = current_hand_state.hand_center_y - self.processing_height * 0.5 if current_hand_state.is_hand_valid else 0.0
        # overlay_lines: 叠加文字行 / Overlay text lines.
        overlay_lines = [
            "State:%s FPS:%d" % (self.control_state, int(self.measured_fps)),
            "Gesture:%s Finger:%s Count:%d Follow:%s" % (
                current_hand_state.gesture_name,
                current_hand_state.finger_code,
                current_hand_state.finger_count,
                "YES" if current_hand_state.is_follow_gesture else "NO",
            ),
            "Center:(%.0f,%.0f) Error H:%.1f V:%.1f" % (
                current_hand_state.hand_center_x,
                current_hand_state.hand_center_y,
                horizontal_error,
                vertical_error,
            ),
            "PD H kp/kd:%.4f/%.4f V kp/kd:%.4f/%.4f" % (self.horizontal_pd_kp, self.horizontal_pd_kd, self.vertical_pd_kp, self.vertical_pd_kd),
            "Cmd x:%.2f z:%.2f PTZ H:%.1f P:%.1f" % (
                self.previous_twist_command.linear.x,
                self.previous_twist_command.angular.z,
                self.fixed_horizontal_deg,
                self.fixed_pitch_deg,
            ),
        ]
        if self.overlay_show_legend:
            overlay_lines.append("4/5 fingers:follow | others/no hand:stop | SPACE:start/pause")

        if self.overlay_show_background:
            # background_height: 背景高度 / Background height.
            background_height = self.overlay_line_height * len(overlay_lines) + 8
            cv.rectangle(frame, (0, 0), (self.processing_width, background_height), (0, 0, 0), -1)

        for line_index, line_text in enumerate(overlay_lines):
            # text_origin: 文字位置 / Text origin.
            text_origin = (6, 15 + line_index * self.overlay_line_height)
            cv.putText(frame, line_text, text_origin, cv.FONT_HERSHEY_SIMPLEX, self.overlay_font_scale, (0, 255, 255), 1, cv.LINE_AA)

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

    def print_cmd_vel_if_needed(self, twist_command: Twist, horizontal_error_pixel: float, vertical_error_pixel: float) -> None:
        """
        按需打印速度命令 / Print velocity command if needed.

        Args:
            twist_command: 速度命令 / Velocity command.
            horizontal_error_pixel: 水平误差 / Horizontal error.
            vertical_error_pixel: 竖直误差 / Vertical error.
        """
        if not self.print_cmd_vel:
            return
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_cmd_print_time_sec < self.print_interval_sec:
            return
        self.get_logger().info(
            "cmd_vel x=%.3f z=%.3f error_h=%.1f error_v=%.1f"
            % (twist_command.linear.x, twist_command.angular.z, horizontal_error_pixel, vertical_error_pixel)
        )
        self.last_cmd_print_time_sec = current_time_sec

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        动态参数更新回调 / Dynamic parameter update callback.

        Args:
            parameters: 参数列表 / Parameter list.

        Returns:
            SetParametersResult: 设置结果 / Set result.
        """
        if not self.enable_rqt_dynamic_parameters:
            return SetParametersResult(successful=False, reason="Dynamic parameters are disabled.")

        # should_recreate_control_timer: 是否重建控制定时器 / Whether to recreate control timer.
        should_recreate_control_timer = False
        # should_recreate_image_timer: 是否重建图像定时器 / Whether to recreate image timer.
        should_recreate_image_timer = False
        # should_recreate_ptz_hold_timer: 是否重建云台保持定时器 / Whether to recreate PTZ hold timer.
        should_recreate_ptz_hold_timer = False
        # should_publish_ptz_once: 是否立即发布一次云台角度 / Whether to publish PTZ once immediately.
        should_publish_ptz_once = False
        # should_update_pd_gains: 是否更新 PD 参数 / Whether to update PD gains.
        should_update_pd_gains = False

        try:
            for parameter in parameters:
                if parameter.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason="Parameter '%s' requires node restart." % parameter.name)
                if parameter.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                if parameter.name == "control_rate_hz":
                    should_recreate_control_timer = True
                elif parameter.name == "image_processing_rate_hz":
                    should_recreate_image_timer = True
                elif parameter.name == "ptz_hold_rate_hz":
                    should_recreate_ptz_hold_timer = True
                elif parameter.name in {"fixed_horizontal_deg", "fixed_pitch_deg"}:
                    should_publish_ptz_once = True
                elif parameter.name in {"horizontal_pd_kp", "horizontal_pd_kd", "vertical_pd_kp", "vertical_pd_kd"}:
                    should_update_pd_gains = True

            self.apply_dynamic_parameters_to_attributes(parameters)

            if should_update_pd_gains:
                self.horizontal_pd_controller.update_gains(self.horizontal_pd_kp, self.horizontal_pd_kd)
                self.vertical_pd_controller.update_gains(self.vertical_pd_kp, self.vertical_pd_kd)

            if should_recreate_control_timer:
                self.destroy_timer(self.control_timer)
                self.control_timer = self.create_timer(1.0 / max(self.control_rate_hz, MINIMUM_TIMER_RATE_HZ), self.publish_control_command)
            if should_recreate_image_timer:
                self.destroy_timer(self.image_processing_timer)
                self.image_processing_timer = self.create_timer(1.0 / max(self.image_processing_rate_hz, MINIMUM_TIMER_RATE_HZ), self.process_latest_image)
            if should_recreate_ptz_hold_timer:
                if self.ptz_hold_timer is not None:
                    self.destroy_timer(self.ptz_hold_timer)
                    self.ptz_hold_timer = None
                if self.publish_ptz_hold_command:
                    self.ptz_hold_timer = self.create_timer(1.0 / max(self.ptz_hold_rate_hz, MINIMUM_TIMER_RATE_HZ), self.publish_fixed_ptz_angle)
            if should_publish_ptz_once:
                self.publish_fixed_ptz_angle()
            return SetParametersResult(successful=True)
        except Exception as exception:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception))

    def apply_dynamic_parameters_to_attributes(self, parameters: List[Parameter]) -> None:
        """
        将动态参数应用到节点属性 / Apply dynamic parameters to node attributes.

        Args:
            parameters: 参数列表 / Parameter list.
        """
        for parameter in parameters:
            parameter_name = parameter.name
            parameter_value = parameter.value
            if parameter_name in {"processing_width", "processing_height", "display_width", "display_height", "overlay_line_height", "initial_publish_count"}:
                setattr(self, parameter_name, int(parameter_value))
                if parameter_name in {"processing_width", "processing_height"}:
                    setattr(self, parameter_name, max(int(parameter_value), MINIMUM_PROCESSING_SIZE_PIXEL))
            elif parameter_name in {
                "overlay_font_scale",
                "fixed_horizontal_deg",
                "fixed_pitch_deg",
                "initial_publish_interval_sec",
                "ptz_hold_rate_hz",
                "horizontal_deadband_pixel",
                "vertical_deadband_pixel",
                "horizontal_pd_kp",
                "horizontal_pd_kd",
                "vertical_pd_kp",
                "vertical_pd_kd",
                "max_linear_x",
                "max_angular_z",
                "linear_velocity_deadband",
                "angular_velocity_deadband",
                "control_rate_hz",
                "image_processing_rate_hz",
                "hand_lost_timeout_sec",
                "print_interval_sec",
            }:
                setattr(self, parameter_name, float(parameter_value))
                if parameter_name in {"control_rate_hz", "image_processing_rate_hz", "ptz_hold_rate_hz"}:
                    setattr(self, parameter_name, max(float(parameter_value), MINIMUM_TIMER_RATE_HZ))
                if parameter_name in {"max_linear_x", "max_angular_z", "linear_velocity_deadband", "angular_velocity_deadband"}:
                    setattr(self, parameter_name, abs(float(parameter_value)))
            elif parameter_name in {
                "overlay_show_background",
                "overlay_show_legend",
                "enable_horizontal_flip",
                "enable_vertical_flip",
                "publish_initial_ptz_command",
                "publish_ptz_hold_command",
                "reverse_body_angular_direction",
                "reverse_linear_from_vertical",
                "enable_body_angular_control",
                "enable_body_linear_control",
                "stop_when_hand_lost",
                            "auto_start_tracking",
                "print_cmd_vel",
                "enable_rqt_dynamic_parameters",
            }:
                setattr(self, parameter_name, bool(parameter_value))
                if parameter_name == "auto_start_tracking":
                    self.control_is_enabled = bool(parameter_value)
                    self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            elif parameter_name == "rotate_image_code":
                self.rotate_image_code = str(parameter_value)

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            bool: 销毁结果 / Destroy result.
        """
        self.is_node_running = False
        self.reset_pd_controllers()
        self.publish_zero_twist()
        self.publish_fixed_ptz_angle()
        try:
            self.hand_detector.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """
    节点入口 / Node entry point.

    Args:
        args: ROS 参数 / ROS args.
    """
    rclpy.init(args=args)
    # gesture_follow_node: 手势跟随节点 / Gesture follow node.
    gesture_follow_node = GestureFollowNode()
    try:
        rclpy.spin(gesture_follow_node)
    except KeyboardInterrupt:
        gesture_follow_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        if rclpy.ok():
            gesture_follow_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
