#!/usr/bin/env python3
# encoding: utf-8
"""
RobotCtrl.py

中文说明:
    ROS2 Humble 手掌位置遥控节点。
    本版本参考已测试通过的手势控制、巡线和二维码控制程序结构，将图像接收、手部识别、底盘控制解耦：
    图像回调只缓存最新原始图像，图像处理定时器负责检测手部中心，控制定时器固定频率发布 /cmd_vel。
    云台只通过 /cmd_ptz_angle 初始化角度，不再使用 servo_s1/servo_s2。

English Description:
    ROS2 Humble palm-position robot control node.
    This version follows the tested gesture/line/QR control architecture: image receiving, hand detection,
    and chassis control are decoupled. The image callback only caches raw images, the image timer detects
    the palm center, and the control timer publishes /cmd_vel at a fixed rate. PTZ initialization only uses
    /cmd_ptz_angle; servo_s1/servo_s2 are no longer used.
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
from typing import Any, List, Optional, Tuple

import cv2 as cv
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

try:
    # HandDetector: 优先使用当前功能包内的 media_library / Prefer local media_library in this package.
    from .media_library import HandDetector
except Exception:  # pylint: disable=broad-except
    try:
        # HandDetector: 兼容旧工程 yahboom_esp32ai_car.media_library / Compatible with legacy yahboom package.
        from yahboom_esp32ai_car.media_library import HandDetector
    except Exception as import_exception:  # pylint: disable=broad-except
        raise ImportError(
            "Cannot import HandDetector. Please make sure media_library.py exists in this package "
            "or yahboom_esp32ai_car.media_library is installed."
        ) from import_exception


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "robot_ctrl_node"

# CONTROL_STATE_IDENTIFY: 只识别手部但不控制底盘状态 / Identify hand without chassis control state.
CONTROL_STATE_IDENTIFY = "identify"

# CONTROL_STATE_TRACKING: 手掌位置控制底盘状态 / Active palm-position control state.
CONTROL_STATE_TRACKING = "tracking"

# CONTROL_STATE_LOST: 手部丢失状态 / Hand lost state.
CONTROL_STATE_LOST = "lost"

# CONTROL_STATE_PAUSED: 暂停控制状态 / Paused control state.
CONTROL_STATE_PAUSED = "paused"

# KEY_CODE_SPACE: 空格键编码 / Space key code.
KEY_CODE_SPACE = 32

# KEY_CODE_ESCAPE: ESC 键编码 / ESC key code.
KEY_CODE_ESCAPE = 27

# MINIMUM_TIMER_RATE_HZ: 定时器最小频率 / Minimum timer frequency.
MINIMUM_TIMER_RATE_HZ = 1.0

# MINIMUM_HAND_LOST_TIMEOUT_SEC: 手部丢失超时时间下限 / Lower bound of hand lost timeout.
MINIMUM_HAND_LOST_TIMEOUT_SEC = 0.05

# DYNAMIC_PARAMETER_NAMES: 支持 rqt 运行时动态修改的参数 / Runtime dynamic parameters for rqt.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "show_debug_window",
    "display_width",
    "display_height",
    "overlay_font_scale",
    "overlay_line_height",
    "overlay_show_background",
    "overlay_show_legend",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "linear_speed",
    "angular_speed",
    "forward_y_threshold",
    "back_y_threshold",
    "left_x_threshold",
    "right_x_threshold",
    "control_rate_hz",
    "image_processing_rate_hz",
    "hand_lost_timeout_sec",
    "stop_when_hand_lost",
    "auto_start_control",
    "enable_joy_override",
    "publish_initial_ptz_command",
    "initial_horizontal_deg",
    "initial_pitch_deg",
    "initial_publish_count",
    "initial_publish_interval_sec",
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
    "ptz_command_topic",
    "window_name",
})


@dataclass
class HandControlResult:
    """手掌位置控制结果 / Palm-position control result."""

    # is_hand_found: 是否识别到手 / Whether a hand is found.
    is_hand_found: bool

    # hand_center_x: 手掌中心 x 坐标 / Palm center x coordinate.
    hand_center_x: float = 0.0

    # hand_center_y: 手掌中心 y 坐标 / Palm center y coordinate.
    hand_center_y: float = 0.0

    # linear_velocity: 目标线速度 / Target linear velocity.
    linear_velocity: float = 0.0

    # angular_velocity: 目标角速度 / Target angular velocity.
    angular_velocity: float = 0.0

    # action_text: 动作含义 / Action meaning.
    action_text: str = "No hand - Stop"


class RobotCtrlNode(Node):
    """手掌位置遥控节点 / Palm-position robot control node."""

    def __init__(self) -> None:
        """初始化节点 / Initialize node."""
        super().__init__(DEFAULT_NODE_NAME)

        # image_lock: 图像缓存互斥锁 / Image cache mutex.
        self.image_lock = threading.Lock()
        # hand_state_lock: 手部状态互斥锁 / Hand state mutex.
        self.hand_state_lock = threading.Lock()

        # latest_image_msg: 最新原始图像消息 / Latest raw image message.
        self.latest_image_msg: Optional[Image] = None
        # latest_image_sequence_id: 最新图像序号 / Latest image sequence ID.
        self.latest_image_sequence_id = 0
        # processed_image_sequence_id: 已处理图像序号 / Processed image sequence ID.
        self.processed_image_sequence_id = 0

        # control_state: 当前控制状态 / Current control state.
        self.control_state = CONTROL_STATE_IDENTIFY
        # control_is_enabled: 是否允许手掌位置控制底盘 / Whether palm-position control is enabled.
        self.control_is_enabled = False
        # is_node_running: 节点运行标志 / Node running flag.
        self.is_node_running = True
        # joy_is_active: 手柄接管标志 / Joystick override flag.
        self.joy_is_active = False

        # hand_is_valid: 手部有效标志 / Hand valid flag.
        self.hand_is_valid = False
        # hand_center: 手掌中心 / Palm center.
        self.hand_center: Tuple[float, float] = (0.0, 0.0)
        # latest_hand_update_time_sec: 最近手部更新时间 / Latest hand update time.
        self.latest_hand_update_time_sec = 0.0
        # target_linear_velocity: 目标线速度 / Target linear velocity.
        self.target_linear_velocity = 0.0
        # target_angular_velocity: 目标角速度 / Target angular velocity.
        self.target_angular_velocity = 0.0
        # action_text: 当前动作文本 / Current action text.
        self.action_text = "No hand - Stop"

        # image_width: 图像宽度 / Image width.
        self.image_width = 0
        # image_height: 图像高度 / Image height.
        self.image_height = 0
        # latest_display_frame: 最新显示图像 / Latest display frame.
        self.latest_display_frame: Optional[np.ndarray] = None

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
        # debug_window_initialized: 调试窗口初始化标志 / Debug window initialized flag.
        self.debug_window_initialized = False

        self.declare_and_get_parameters()
        self.control_is_enabled = self.auto_start_control
        self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY

        # bridge: CV Bridge 工具 / CV Bridge helper.
        self.bridge = CvBridge()
        # hand_detector: 手部检测器 / Hand detector.
        self.hand_detector = HandDetector()

        # image_qos_profile: 图像订阅 QoS 配置 / QoS profile for image subscription.
        self.image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # image_subscription: 原始图像订阅器 / Raw image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            self.image_qos_profile,
        )
        # joy_state_subscription: 手柄状态订阅器 / Joystick state subscriber.
        self.joy_state_subscription = self.create_subscription(
            Bool,
            self.joy_state_topic,
            self.joy_state_callback,
            10,
        )
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

        self.get_logger().info(
            "RobotCtrl node started. image_topic=%s, cmd_vel_topic=%s, initial_pitch_deg=%.1f"
            % (self.image_topic, self.cmd_vel_topic, self.initial_pitch_deg)
        )

    def declare_and_get_parameters(self) -> None:
        """声明并读取参数 / Declare and read parameters."""
        # 图像和话题参数 / Image and topic parameters.
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_state_topic", "/JoyState")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "robot_ctrl")

        # 显示参数 / Display parameters.
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.32)
        self.declare_parameter("overlay_line_height", 13)
        self.declare_parameter("overlay_show_background", False)
        self.declare_parameter("overlay_show_legend", False)

        # 图像方向参数 / Image orientation parameters.
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")

        # 手掌位置控制参数 / Palm-position control parameters.
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.20)
        self.declare_parameter("forward_y_threshold", 150)
        self.declare_parameter("back_y_threshold", 270)
        self.declare_parameter("left_x_threshold", 300)
        self.declare_parameter("right_x_threshold", 350)
        self.declare_parameter("hand_lost_timeout_sec", 0.5)

        # 控制频率参数 / Control frequency parameters.
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("image_processing_rate_hz", 30.0)

        # 安全和状态参数 / Safety and state parameters.
        self.declare_parameter("stop_when_hand_lost", True)
        self.declare_parameter("auto_start_control", True)
        self.declare_parameter("enable_joy_override", True)

        # 云台初始化参数 / PTZ initialization parameters.
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("initial_horizontal_deg", 0.0)
        self.declare_parameter("initial_pitch_deg", -45.0)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)

        # 调试参数 / Debug parameters.
        self.declare_parameter("debug_log_interval_sec", 2.0)
        self.declare_parameter("print_cmd_vel", False)
        self.declare_parameter("print_interval_sec", 0.5)
        self.declare_parameter("enable_rqt_dynamic_parameters", True)

        self.refresh_parameters_from_ros()

    def refresh_parameters_from_ros(self) -> None:
        """从 ROS 参数服务器刷新参数 / Refresh parameters from ROS parameter server."""
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self.ptz_command_topic = str(self.get_parameter("ptz_command_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)
        self.overlay_show_background = bool(self.get_parameter("overlay_show_background").value)
        self.overlay_show_legend = bool(self.get_parameter("overlay_show_legend").value)

        self.enable_horizontal_flip = bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)

        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self.angular_speed = abs(float(self.get_parameter("angular_speed").value))
        self.forward_y_threshold = int(self.get_parameter("forward_y_threshold").value)
        self.back_y_threshold = int(self.get_parameter("back_y_threshold").value)
        self.left_x_threshold = int(self.get_parameter("left_x_threshold").value)
        self.right_x_threshold = int(self.get_parameter("right_x_threshold").value)
        self.hand_lost_timeout_sec = max(
            float(self.get_parameter("hand_lost_timeout_sec").value),
            MINIMUM_HAND_LOST_TIMEOUT_SEC,
        )

        self.control_rate_hz = max(float(self.get_parameter("control_rate_hz").value), MINIMUM_TIMER_RATE_HZ)
        self.image_processing_rate_hz = max(float(self.get_parameter("image_processing_rate_hz").value), MINIMUM_TIMER_RATE_HZ)

        self.stop_when_hand_lost = bool(self.get_parameter("stop_when_hand_lost").value)
        self.auto_start_control = bool(self.get_parameter("auto_start_control").value)
        self.enable_joy_override = bool(self.get_parameter("enable_joy_override").value)

        self.publish_initial_ptz_command = bool(self.get_parameter("publish_initial_ptz_command").value)
        self.initial_horizontal_deg = float(self.get_parameter("initial_horizontal_deg").value)
        self.initial_pitch_deg = float(self.get_parameter("initial_pitch_deg").value)
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)

        self.debug_log_interval_sec = float(self.get_parameter("debug_log_interval_sec").value)
        self.print_cmd_vel = bool(self.get_parameter("print_cmd_vel").value)
        self.print_interval_sec = float(self.get_parameter("print_interval_sec").value)
        self.enable_rqt_dynamic_parameters = bool(self.get_parameter("enable_rqt_dynamic_parameters").value)

    def image_callback(self, image_msg: Image) -> None:
        """
        缓存最新原始图像 / Cache the latest raw image.

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
        if self.enable_joy_override and self.joy_is_active:
            self.publish_zero_twist()

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
        """处理最新图像并更新手部状态 / Process latest image and update hand state."""
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

        # hand_result: 手掌位置控制结果 / Palm-position control result.
        hand_result = self.detect_hand_and_compute_command(processed_frame)
        self.update_hand_state(hand_result)

        # display_frame: 调试显示图像 / Debug display frame.
        display_frame = processed_frame.copy()
        self.draw_control_guides(display_frame)
        self.draw_hand_result(display_frame, hand_result)
        self.draw_overlay_text(display_frame)
        self.latest_display_frame = display_frame

        self.update_fps_statistics()
        self.update_debug_window_if_needed()

    def decode_image(self, image_msg: Image) -> Optional[np.ndarray]:
        """
        解码原始图像 / Decode raw image.

        Args:
            image_msg: 原始图像消息 / Raw image message.

        Returns:
            Optional[np.ndarray]: 解码后的 BGR 图像 / Decoded BGR frame.
        """
        try:
            return self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
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
        resized_frame = cv.resize(image_frame, (640, 480))
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

    def detect_hand_and_compute_command(self, image_frame: np.ndarray) -> HandControlResult:
        """
        检测手部并计算控制命令 / Detect hand and compute control command.

        Args:
            image_frame: BGR 图像 / BGR frame.

        Returns:
            HandControlResult: 手掌位置控制结果 / Palm-position control result.
        """
        try:
            # detected_frame: 检测器绘制后的图像 / Detector-rendered image.
            detected_frame, landmark_list, _ = self.hand_detector.findHands(image_frame)
            image_frame[:, :] = detected_frame
        except Exception as exception:  # pylint: disable=broad-except
            self.throttled_debug_log("Hand detection failed: %s" % exception)
            return HandControlResult(False)

        if len(landmark_list) == 0:
            return HandControlResult(False)
        if len(landmark_list) <= 9 or len(landmark_list[9]) < 3:
            return HandControlResult(False)

        # palm_center_x: 手掌中心 x 坐标，沿用旧程序 lmList[9] / Palm center x from legacy lmList[9].
        palm_center_x = float(landmark_list[9][1])
        # palm_center_y: 手掌中心 y 坐标，沿用旧程序 lmList[9] / Palm center y from legacy lmList[9].
        palm_center_y = float(landmark_list[9][2])
        return self.compute_command_from_hand_center(palm_center_x, palm_center_y)

    def compute_command_from_hand_center(self, hand_center_x: float, hand_center_y: float) -> HandControlResult:
        """
        根据手掌中心计算底盘速度 / Compute chassis velocity from palm center.

        Args:
            hand_center_x: 手掌中心 x 坐标 / Palm center x coordinate.
            hand_center_y: 手掌中心 y 坐标 / Palm center y coordinate.

        Returns:
            HandControlResult: 控制结果 / Control result.
        """
        # linear_velocity: 线速度命令 / Linear velocity command.
        linear_velocity = 0.0
        # angular_velocity: 角速度命令 / Angular velocity command.
        angular_velocity = 0.0
        # action_parts: 动作描述列表 / Action description parts.
        action_parts: List[str] = []

        if hand_center_y <= self.forward_y_threshold:
            linear_velocity = self.linear_speed
            action_parts.append("Forward")
        elif hand_center_y >= self.back_y_threshold:
            linear_velocity = -self.linear_speed
            action_parts.append("Back")

        if hand_center_x <= self.left_x_threshold:
            angular_velocity = self.angular_speed
            action_parts.append("Left")
        elif hand_center_x >= self.right_x_threshold:
            angular_velocity = -self.angular_speed
            action_parts.append("Right")

        if not action_parts:
            action_parts.append("Stop")

        return HandControlResult(
            True,
            hand_center_x,
            hand_center_y,
            linear_velocity,
            angular_velocity,
            " + ".join(action_parts),
        )

    def update_hand_state(self, hand_result: HandControlResult) -> None:
        """
        更新手部状态 / Update hand state.

        Args:
            hand_result: 手部控制结果 / Hand control result.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        with self.hand_state_lock:
            if hand_result.is_hand_found:
                self.hand_is_valid = True
                self.hand_center = (hand_result.hand_center_x, hand_result.hand_center_y)
                self.target_linear_velocity = hand_result.linear_velocity
                self.target_angular_velocity = hand_result.angular_velocity
                self.action_text = hand_result.action_text
                self.latest_hand_update_time_sec = current_time_sec
                self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            else:
                # hand_age_sec: 手部丢失时间 / Hand lost age.
                hand_age_sec = current_time_sec - self.latest_hand_update_time_sec
                if hand_age_sec > self.hand_lost_timeout_sec:
                    self.hand_is_valid = False
                    self.target_linear_velocity = 0.0
                    self.target_angular_velocity = 0.0
                    self.action_text = "No hand - Stop"
                    if self.control_is_enabled:
                        self.control_state = CONTROL_STATE_LOST

    def publish_chassis_control_command(self) -> None:
        """固定频率发布底盘控制命令 / Publish chassis control command at a fixed rate."""
        if not self.is_node_running:
            return

        if self.enable_joy_override and self.joy_is_active:
            self.publish_zero_twist()
            return

        if not self.control_is_enabled:
            self.publish_zero_twist()
            return

        with self.hand_state_lock:
            # local_hand_is_valid: 本地手部有效标志 / Local hand valid flag.
            local_hand_is_valid = self.hand_is_valid
            # local_linear_velocity: 本地线速度 / Local linear velocity.
            local_linear_velocity = self.target_linear_velocity
            # local_angular_velocity: 本地角速度 / Local angular velocity.
            local_angular_velocity = self.target_angular_velocity
            # local_latest_update_time_sec: 本地最近更新时间 / Local latest update time.
            local_latest_update_time_sec = self.latest_hand_update_time_sec

        # hand_age_sec: 距离上一次识别手部的时间 / Age since latest hand update.
        hand_age_sec = time.perf_counter() - local_latest_update_time_sec
        if (not local_hand_is_valid or hand_age_sec > self.hand_lost_timeout_sec) and self.stop_when_hand_lost:
            self.control_state = CONTROL_STATE_LOST
            self.publish_zero_twist()
            return

        # twist_command: 底盘速度命令 / Chassis velocity command.
        twist_command = Twist()
        twist_command.linear.x = float(local_linear_velocity)
        twist_command.angular.z = float(local_angular_velocity)
        self.cmd_vel_publisher.publish(twist_command)
        self.previous_twist_command = twist_command
        self.print_cmd_vel_if_needed(twist_command)

    def publish_zero_twist(self) -> None:
        """发布零速度 / Publish zero velocity."""
        # zero_twist_msg: 零速度消息 / Zero velocity message.
        zero_twist_msg = Twist()
        self.cmd_vel_publisher.publish(zero_twist_msg)
        self.previous_twist_command = zero_twist_msg

    def initialize_debug_window_if_needed(self) -> None:
        """按需初始化调试窗口 / Initialize debug window if needed."""
        if not self.show_debug_window:
            return
        if self.debug_window_initialized:
            return
        cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
        cv.resizeWindow(self.window_name, int(self.display_width), int(self.display_height))
        self.debug_window_initialized = True

    def update_debug_window_if_needed(self) -> None:
        """按需刷新调试窗口 / Update debug window if needed."""
        if not self.show_debug_window:
            if self.debug_window_initialized:
                cv.destroyWindow(self.window_name)
                self.debug_window_initialized = False
            return
        if self.latest_display_frame is None:
            return
        self.initialize_debug_window_if_needed()
        # display_image: 调整后的显示图像 / Resized display image.
        display_image = cv.resize(self.latest_display_frame, (int(self.display_width), int(self.display_height)))
        cv.imshow(self.window_name, display_image)
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
            self.reset_control_state()
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
                self.control_state = CONTROL_STATE_TRACKING if self.hand_is_valid else CONTROL_STATE_LOST

    def reset_control_state(self) -> None:
        """重置控制状态 / Reset control state."""
        with self.hand_state_lock:
            self.hand_is_valid = False
            self.hand_center = (0.0, 0.0)
            self.latest_hand_update_time_sec = 0.0
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.action_text = "Reset - Stop"
        self.control_is_enabled = False
        self.control_state = CONTROL_STATE_IDENTIFY
        self.publish_zero_twist()
        self.get_logger().info("RobotCtrl state reset. Press SPACE to enable palm-position control.")

    def cancel_node(self) -> None:
        """取消节点运行 / Cancel node running."""
        self.is_node_running = False
        self.publish_zero_twist()
        if self.show_debug_window:
            cv.destroyAllWindows()
        self.get_logger().info("RobotCtrl node requested to stop from keyboard.")
        os._exit(0)  # pylint: disable=protected-access

    def draw_control_guides(self, display_frame: np.ndarray) -> None:
        """
        绘制控制阈值线 / Draw control threshold guides.

        Args:
            display_frame: 显示图像 / Display frame.
        """
        # frame_width: 图像宽度 / Frame width.
        frame_width = display_frame.shape[1]
        # frame_height: 图像高度 / Frame height.
        frame_height = display_frame.shape[0]
        cv.line(display_frame, (0, self.forward_y_threshold), (frame_width, self.forward_y_threshold), (0, 180, 255), 1)
        cv.line(display_frame, (0, self.back_y_threshold), (frame_width, self.back_y_threshold), (0, 180, 255), 1)
        cv.line(display_frame, (self.left_x_threshold, 0), (self.left_x_threshold, frame_height), (255, 255, 0), 1)
        cv.line(display_frame, (self.right_x_threshold, 0), (self.right_x_threshold, frame_height), (255, 255, 0), 1)
        cv.rectangle(
            display_frame,
            (self.left_x_threshold, self.forward_y_threshold),
            (self.right_x_threshold, self.back_y_threshold),
            (80, 80, 80),
            1,
        )

    def draw_hand_result(self, display_frame: np.ndarray, hand_result: HandControlResult) -> None:
        """
        绘制手掌中心结果 / Draw palm center result.

        Args:
            display_frame: 显示图像 / Display frame.
            hand_result: 手部控制结果 / Hand control result.
        """
        if not hand_result.is_hand_found:
            return
        # center_point: 手掌中心点 / Palm center point.
        center_point = (int(hand_result.hand_center_x), int(hand_result.hand_center_y))
        cv.circle(display_frame, center_point, 7, (0, 0, 255), -1)
        cv.putText(
            display_frame,
            hand_result.action_text,
            (max(0, center_point[0] - 80), max(20, center_point[1] - 12)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv.LINE_AA,
        )

    def draw_overlay_text(self, display_frame: np.ndarray) -> None:
        """
        绘制叠加文字 / Draw overlay text.

        Args:
            display_frame: 显示图像 / Display frame.
        """
        with self.hand_state_lock:
            # center_text: 手掌中心文本 / Palm center text.
            center_text = "None" if not self.hand_is_valid else "%.1f, %.1f" % self.hand_center
            # action_text: 当前动作文本 / Current action text.
            action_text = self.action_text

        # overlay_lines: 叠加文字行 / Overlay text lines.
        overlay_lines = [
            "State: %s | FPS: %.1f" % (self.control_state, self.measured_fps),
            "Hand center: %s" % center_text,
            "Action: %s" % action_text,
            "cmd_vel: x=%.2f z=%.2f" % (self.previous_twist_command.linear.x, self.previous_twist_command.angular.z),
            "Keys: SPACE control/pause, i identify, r reset, q quit",
        ]
        if self.overlay_show_legend:
            overlay_lines.extend([
                "Up:Forward | Down:Back",
                "Left area:Left turn | Right area:Right turn",
            ])

        if self.overlay_show_background:
            # background_height: 背景高度 / Background height.
            background_height = self.overlay_line_height * len(overlay_lines) + 8
            cv.rectangle(
                display_frame,
                (0, 0),
                (min(display_frame.shape[1], 720), background_height),
                (0, 0, 0),
                -1,
            )

        for line_index, line_text in enumerate(overlay_lines):
            # text_origin: 文字起点 / Text origin.
            text_origin = (6, 15 + line_index * self.overlay_line_height)
            cv.putText(
                display_frame,
                line_text,
                text_origin,
                cv.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv.LINE_AA,
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

    def print_cmd_vel_if_needed(self, twist_command: Twist) -> None:
        """
        按需打印速度命令 / Print velocity command if needed.

        Args:
            twist_command: 速度命令 / Velocity command.
        """
        if not self.print_cmd_vel:
            return
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if current_time_sec - self.last_cmd_print_time_sec < self.print_interval_sec:
            return
        self.get_logger().info(
            "cmd_vel linear.x=%.3f angular.z=%.3f"
            % (twist_command.linear.x, twist_command.angular.z)
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
        # should_publish_ptz_once: 是否立即发布一次云台角度 / Whether to publish PTZ once immediately.
        should_publish_ptz_once = False

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
                elif parameter_name in {"initial_horizontal_deg", "initial_pitch_deg"}:
                    should_publish_ptz_once = True

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
            if should_publish_ptz_once:
                self.publish_ptz_angle(self.initial_horizontal_deg, self.initial_pitch_deg)
            if any(parameter.name == "show_debug_window" for parameter in parameters):
                if not self.show_debug_window and self.debug_window_initialized:
                    cv.destroyWindow(self.window_name)
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
            elif parameter_name == "display_width":
                self.display_width = int(parameter_value)
            elif parameter_name == "display_height":
                self.display_height = int(parameter_value)
            elif parameter_name == "overlay_font_scale":
                self.overlay_font_scale = float(parameter_value)
            elif parameter_name == "overlay_line_height":
                self.overlay_line_height = int(parameter_value)
            elif parameter_name == "overlay_show_background":
                self.overlay_show_background = bool(parameter_value)
            elif parameter_name == "overlay_show_legend":
                self.overlay_show_legend = bool(parameter_value)
            elif parameter_name == "enable_horizontal_flip":
                self.enable_horizontal_flip = bool(parameter_value)
            elif parameter_name == "enable_vertical_flip":
                self.enable_vertical_flip = bool(parameter_value)
            elif parameter_name == "rotate_image_code":
                self.rotate_image_code = str(parameter_value)
            elif parameter_name in {"linear_speed", "angular_speed"}:
                setattr(self, parameter_name, abs(float(parameter_value)))
            elif parameter_name in {"forward_y_threshold", "back_y_threshold", "left_x_threshold", "right_x_threshold", "initial_publish_count"}:
                setattr(self, parameter_name, int(parameter_value))
            elif parameter_name == "hand_lost_timeout_sec":
                self.hand_lost_timeout_sec = max(float(parameter_value), MINIMUM_HAND_LOST_TIMEOUT_SEC)
            elif parameter_name == "control_rate_hz":
                self.control_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name == "image_processing_rate_hz":
                self.image_processing_rate_hz = max(float(parameter_value), MINIMUM_TIMER_RATE_HZ)
            elif parameter_name in {"stop_when_hand_lost", "auto_start_control", "enable_joy_override", "publish_initial_ptz_command", "print_cmd_vel", "enable_rqt_dynamic_parameters"}:
                setattr(self, parameter_name, bool(parameter_value))
                if parameter_name == "auto_start_control":
                    self.control_is_enabled = bool(parameter_value)
                    self.control_state = CONTROL_STATE_TRACKING if self.control_is_enabled else CONTROL_STATE_IDENTIFY
            elif parameter_name in {"initial_horizontal_deg", "initial_pitch_deg", "initial_publish_interval_sec", "debug_log_interval_sec", "print_interval_sec"}:
                setattr(self, parameter_name, float(parameter_value))

    def destroy_node(self) -> bool:
        """
        销毁节点 / Destroy node.

        Returns:
            bool: 销毁结果 / Destroy result.
        """
        self.is_node_running = False
        self.publish_zero_twist()
        try:
            self.hand_detector.close()
        except Exception:  # pylint: disable=broad-except
            pass
        if self.show_debug_window:
            cv.destroyAllWindows()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    """
    节点入口函数 / Node entry point.

    Args:
        args: ROS 参数 / ROS arguments.
    """
    rclpy.init(args=args)
    # robot_ctrl_node: 手掌位置遥控节点 / Palm-position robot control node.
    robot_ctrl_node = RobotCtrlNode()
    try:
        rclpy.spin(robot_ctrl_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            robot_ctrl_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
