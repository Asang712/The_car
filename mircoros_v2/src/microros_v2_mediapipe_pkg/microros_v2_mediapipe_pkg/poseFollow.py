#!/usr/bin/env python3
# encoding: utf-8
"""
poseFollow_compressed.py

中文说明:
    ROS2 Humble 人体姿态跟随节点。
    本版本删除深度相机相关程序，只订阅原始图像话题 /camera/image_raw 进行人体姿态检测。
    因为不再使用深度图，线速度默认根据人体姿态框高度进行视觉尺度估计，角速度根据人体中心横向偏差进行控制。
    本版本增加云台保持控制：默认通过 /cmd_ptz_angle 持续发布 horizontal=0 deg、pitch=0 deg。
    当前 RGB 单目版本将 1m 跟随距离近似标定为 target_pose_height_pixel=300。

English Description:
    ROS2 Humble pose following node.
    This version removes depth-camera related logic and only subscribes to the raw image topic
    /camera/image_raw for human pose detection. Without depth input, linear velocity is estimated
    from the detected pose bounding-box height, and angular velocity is controlled by horizontal pose offset.
    This version also keeps PTZ at horizontal=0 deg and pitch=0 deg through /cmd_ptz_angle.
    In this RGB-only version, 1m following is approximated by target_pose_height_pixel=300.
"""

import os

# MEDIAPIPE_DISABLE_TF: 禁用 MediaPipe TensorFlow 初始化 / Disable MediaPipe TensorFlow initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_TF", "1")
# MEDIAPIPE_DISABLE_GPU: 禁用 MediaPipe GPU 初始化 / Disable MediaPipe GPU initialization.
os.environ.setdefault("MEDIAPIPE_DISABLE_GPU", "1")

import time
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

from .media_common import PoseDetector


# DEFAULT_NODE_NAME: 默认节点名称 / Default node name.
DEFAULT_NODE_NAME = "pose_follow_node"

# MINIMUM_PID_DELTA_TIME: PID 最小时间间隔 / Minimum PID delta time.
MINIMUM_PID_DELTA_TIME = 1e-3

# MINIMUM_PTZ_HOLD_RATE_HZ: 云台保持发布最小频率 / Minimum PTZ hold publish rate.
MINIMUM_PTZ_HOLD_RATE_HZ = 0.2

# DYNAMIC_PARAMETER_NAMES: 支持运行时动态调节的参数 / Runtime dynamic parameters.
DYNAMIC_PARAMETER_NAMES = frozenset({
    "linear_Kp",
    "linear_Ki",
    "linear_Kd",
    "angular_Kp",
    "angular_Ki",
    "angular_Kd",
    "target_pose_height_pixel",
    "pose_height_deadband_pixel",
    "center_deadband_pixel",
    "max_linear_x",
    "max_angular_z",
    "reverse_linear_direction",
    "reverse_angular_direction",
    "enable_linear_follow",
    "enable_horizontal_flip",
    "enable_vertical_flip",
    "rotate_image_code",
    "display_width",
    "display_height",
    "overlay_font_scale",
    "overlay_line_height",
    "show_debug_window",
    "publish_initial_ptz_command",
    "publish_ptz_hold_command",
    "ptz_horizontal_deg",
    "ptz_pitch_deg",
    "initial_publish_count",
    "initial_publish_interval_sec",
    "ptz_hold_rate_hz",
    "target_follow_distance_meter",
})

# RESTART_REQUIRED_PARAMETER_NAMES: 修改后需要重启的参数 / Parameters requiring node restart.
RESTART_REQUIRED_PARAMETER_NAMES = frozenset({
    "rgb_topic",
    "cmd_vel_topic",
    "joy_state_topic",
    "window_name",
    "ptz_command_topic",
})


class DiscretePidController:
    """离散 PID 控制器 / Discrete PID controller."""

    def __init__(self, proportional_gain: float, integral_gain: float, derivative_gain: float) -> None:
        """
        初始化 PID 控制器 / Initialize PID controller.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            integral_gain: 积分增益 / Integral gain.
            derivative_gain: 微分增益 / Derivative gain.
        """
        # proportional_gain: 比例增益 / Proportional gain.
        self.proportional_gain = float(proportional_gain)
        # integral_gain: 积分增益 / Integral gain.
        self.integral_gain = float(integral_gain)
        # derivative_gain: 微分增益 / Derivative gain.
        self.derivative_gain = float(derivative_gain)
        # previous_error: 上一次误差 / Previous error.
        self.previous_error = 0.0
        # integral_error: 积分误差 / Integral error.
        self.integral_error = 0.0
        # previous_time_sec: 上一次更新时间 / Previous update time.
        self.previous_time_sec: Optional[float] = None

    def update_gains(self, proportional_gain: float, integral_gain: float, derivative_gain: float) -> None:
        """
        更新 PID 参数 / Update PID gains.

        Args:
            proportional_gain: 比例增益 / Proportional gain.
            integral_gain: 积分增益 / Integral gain.
            derivative_gain: 微分增益 / Derivative gain.
        """
        self.proportional_gain = float(proportional_gain)
        self.integral_gain = float(integral_gain)
        self.derivative_gain = float(derivative_gain)

    def reset(self) -> None:
        """重置 PID 状态 / Reset PID state."""
        self.previous_error = 0.0
        self.integral_error = 0.0
        self.previous_time_sec = None

    def update_from_error(self, error_value: float) -> float:
        """
        根据误差计算 PID 输出 / Compute PID output from error.

        Args:
            error_value: 当前误差 / Current error.

        Returns:
            float: PID 输出 / PID output.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        if self.previous_time_sec is None:
            self.previous_time_sec = current_time_sec
            self.previous_error = error_value
            return 0.0

        # delta_time_sec: PID 时间间隔 / PID delta time.
        delta_time_sec = max(current_time_sec - self.previous_time_sec, MINIMUM_PID_DELTA_TIME)
        self.previous_time_sec = current_time_sec
        self.integral_error += error_value * delta_time_sec
        # derivative_error: 微分误差 / Derivative error.
        derivative_error = (error_value - self.previous_error) / delta_time_sec
        self.previous_error = error_value
        return (
            self.proportional_gain * error_value
            + self.integral_gain * self.integral_error
            + self.derivative_gain * derivative_error
        )


class PoseFollowNode(Node):
    """人体姿态跟随节点 / Human pose following node."""

    def __init__(self) -> None:
        """初始化人体姿态跟随节点 / Initialize pose following node."""
        super().__init__(DEFAULT_NODE_NAME)
        self.declare_and_get_parameters()

        # qos_profile: 图像 QoS 配置 / Image QoS profile.
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
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
        # image_subscription: 原始图像订阅器 / Raw image subscriber.
        self.image_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            self.qos_profile,
        )
        # joy_state_subscription: 手柄接管状态订阅器 / Joystick override subscriber.
        self.joy_state_subscription = self.create_subscription(
            Bool,
            self.joy_state_topic,
            self.joy_state_callback,
            10,
        )

        # initial_ptz_publish_counter: 初始云台角度发布计数 / Initial PTZ angle publish counter.
        self.initial_ptz_publish_counter = 0
        # initial_ptz_timer: 初始云台角度发布定时器 / Initial PTZ angle publish timer.
        self.initial_ptz_timer = None
        if self.publish_initial_ptz_command:
            self.initial_ptz_timer = self.create_timer(
                max(self.initial_publish_interval_sec, 0.02),
                self.publish_initial_ptz_command_callback,
            )

        # ptz_hold_timer: 云台角度保持定时器 / PTZ angle hold timer.
        self.ptz_hold_timer = None
        if self.publish_ptz_hold_command:
            self.ptz_hold_timer = self.create_timer(
                1.0 / max(self.ptz_hold_rate_hz, MINIMUM_PTZ_HOLD_RATE_HZ),
                self.publish_ptz_hold_command_callback,
            )

        # image_bridge: 图像转换工具 / Image conversion helper.
        self.image_bridge = CvBridge()
        # pose_detector: 人体姿态检测器 / Human pose detector.
        self.pose_detector = PoseDetector()
        # joy_is_active: 手柄接管标志 / Joystick override flag.
        self.joy_is_active = False
        # previous_time_sec: FPS 上一次时间 / Previous FPS time.
        self.previous_time_sec = time.perf_counter()
        # display_window_initialized: 显示窗口初始化标志 / Display window initialized flag.
        self.display_window_initialized = False
        # latest_twist_command: 最近发布的速度命令 / Latest velocity command.
        self.latest_twist_command = Twist()
        # latest_action_text: 最近动作含义 / Latest action text.
        self.latest_action_text = "Waiting for pose"

        # linear_pid_controller: 线速度 PID 控制器 / Linear velocity PID controller.
        self.linear_pid_controller = DiscretePidController(
            self.linear_Kp / 1000.0,
            self.linear_Ki / 1000.0,
            self.linear_Kd / 1000.0,
        )
        # angular_pid_controller: 角速度 PID 控制器 / Angular velocity PID controller.
        self.angular_pid_controller = DiscretePidController(
            self.angular_Kp / 100.0,
            self.angular_Ki / 100.0,
            self.angular_Kd / 100.0,
        )
        # dynamic_parameter_callback_handle: 动态参数回调句柄 / Dynamic parameter callback handle.
        self.dynamic_parameter_callback_handle = self.add_on_set_parameters_callback(self.on_dynamic_parameter_update)
        self.get_logger().info(
            "poseFollow RGB-only node started. rgb_topic=%s, cmd_vel_topic=%s, ptz=(%.1f, %.1f), target_distance=%.2fm" % (self.rgb_topic, self.cmd_vel_topic, self.ptz_horizontal_deg, self.ptz_pitch_deg, self.target_follow_distance_meter)
        )

    def declare_and_get_parameters(self) -> None:
        """声明并读取 ROS 参数 / Declare and read ROS parameters."""
        # 图像和话题参数 / Image and topic parameters.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joy_state_topic", "/JoyState")
        self.declare_parameter("ptz_command_topic", "/cmd_ptz_angle")
        self.declare_parameter("window_name", "pose_follow")

        # PID 参数 / PID parameters.
        self.declare_parameter("linear_Kp", 10.0)
        self.declare_parameter("linear_Ki", 0.0)
        self.declare_parameter("linear_Kd", 1.0)
        self.declare_parameter("angular_Kp", 0.2)
        self.declare_parameter("angular_Ki", 0.0)
        self.declare_parameter("angular_Kd", 2.0)

        # RGB 单目跟随参数 / RGB-only following parameters.
        self.declare_parameter("target_follow_distance_meter", 1.0)
        self.declare_parameter("target_pose_height_pixel", 300.0)
        self.declare_parameter("pose_height_deadband_pixel", 25.0)
        self.declare_parameter("center_deadband_pixel", 30.0)
        self.declare_parameter("max_linear_x", 0.3)
        self.declare_parameter("max_angular_z", 2.0)
        self.declare_parameter("reverse_linear_direction", False)
        self.declare_parameter("reverse_angular_direction", False)
        self.declare_parameter("enable_linear_follow", True)

        # 云台保持参数 / PTZ hold parameters.
        self.declare_parameter("publish_initial_ptz_command", True)
        self.declare_parameter("publish_ptz_hold_command", True)
        self.declare_parameter("ptz_horizontal_deg", 0.0)
        self.declare_parameter("ptz_pitch_deg", 0.0)
        self.declare_parameter("initial_publish_count", 20)
        self.declare_parameter("initial_publish_interval_sec", 0.2)
        self.declare_parameter("ptz_hold_rate_hz", 2.0)

        # 图像显示和方向参数 / Image display and orientation parameters.
        self.declare_parameter("enable_horizontal_flip", False)
        self.declare_parameter("enable_vertical_flip", False)
        self.declare_parameter("rotate_image_code", "none")
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("display_width", 960)
        self.declare_parameter("display_height", 720)
        self.declare_parameter("overlay_font_scale", 0.38)
        self.declare_parameter("overlay_line_height", 15)

        self.refresh_parameters_from_ros()

    def refresh_parameters_from_ros(self) -> None:
        """从 ROS 参数刷新到节点属性 / Refresh node attributes from ROS parameters."""
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.joy_state_topic = str(self.get_parameter("joy_state_topic").value)
        self.ptz_command_topic = str(self.get_parameter("ptz_command_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.linear_Kp = float(self.get_parameter("linear_Kp").value)
        self.linear_Ki = float(self.get_parameter("linear_Ki").value)
        self.linear_Kd = float(self.get_parameter("linear_Kd").value)
        self.angular_Kp = float(self.get_parameter("angular_Kp").value)
        self.angular_Ki = float(self.get_parameter("angular_Ki").value)
        self.angular_Kd = float(self.get_parameter("angular_Kd").value)

        self.target_follow_distance_meter = float(self.get_parameter("target_follow_distance_meter").value)
        self.target_pose_height_pixel = float(self.get_parameter("target_pose_height_pixel").value)
        self.pose_height_deadband_pixel = float(self.get_parameter("pose_height_deadband_pixel").value)
        self.center_deadband_pixel = float(self.get_parameter("center_deadband_pixel").value)
        self.max_linear_x = abs(float(self.get_parameter("max_linear_x").value))
        self.max_angular_z = abs(float(self.get_parameter("max_angular_z").value))
        self.reverse_linear_direction = bool(self.get_parameter("reverse_linear_direction").value)
        self.reverse_angular_direction = bool(self.get_parameter("reverse_angular_direction").value)
        self.enable_linear_follow = bool(self.get_parameter("enable_linear_follow").value)

        self.publish_initial_ptz_command = bool(self.get_parameter("publish_initial_ptz_command").value)
        self.publish_ptz_hold_command = bool(self.get_parameter("publish_ptz_hold_command").value)
        self.ptz_horizontal_deg = float(self.get_parameter("ptz_horizontal_deg").value)
        self.ptz_pitch_deg = float(self.get_parameter("ptz_pitch_deg").value)
        self.initial_publish_count = int(self.get_parameter("initial_publish_count").value)
        self.initial_publish_interval_sec = float(self.get_parameter("initial_publish_interval_sec").value)
        self.ptz_hold_rate_hz = max(float(self.get_parameter("ptz_hold_rate_hz").value), MINIMUM_PTZ_HOLD_RATE_HZ)

        self.enable_horizontal_flip = bool(self.get_parameter("enable_horizontal_flip").value)
        self.enable_vertical_flip = bool(self.get_parameter("enable_vertical_flip").value)
        self.rotate_image_code = str(self.get_parameter("rotate_image_code").value)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        self.overlay_font_scale = float(self.get_parameter("overlay_font_scale").value)
        self.overlay_line_height = int(self.get_parameter("overlay_line_height").value)

    def publish_initial_ptz_command_callback(self) -> None:
        """重复发布初始云台角度 / Repeatedly publish the initial PTZ angle."""
        if self.initial_ptz_publish_counter >= self.initial_publish_count:
            if self.initial_ptz_timer is not None:
                self.destroy_timer(self.initial_ptz_timer)
                self.initial_ptz_timer = None
            return
        self.publish_ptz_angle(self.ptz_horizontal_deg, self.ptz_pitch_deg)
        self.initial_ptz_publish_counter += 1

    def publish_ptz_hold_command_callback(self) -> None:
        """持续发布云台保持角度 / Continuously publish the PTZ hold angle."""
        self.publish_ptz_angle(self.ptz_horizontal_deg, self.ptz_pitch_deg)

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

    def image_callback(self, image_msg: Image) -> None:
        """
        原始图像回调，执行人体姿态检测 / Raw image callback for pose detection.

        Args:
            image_msg: 原始图像消息 / Raw image message.
        """
        try:
            # color_frame: 解码后的 BGR 图像 / Decoded BGR image.
            color_frame = self.image_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except Exception as exception:  # pylint: disable=broad-except
            self.get_logger().error("Failed to convert raw image: %s" % exception)
            return

        # result_image: 统一尺寸并处理方向后的图像 / Resized and orientation-corrected image.
        result_image = self.prepare_image_frame(color_frame)
        # fps: 当前图像帧率 / Current image FPS.
        fps = self.calculate_fps()

        # pose_points: 姿态关键点 / Pose key points.
        result_image, pose_points, _ = self.pose_detector.pubPosePoint(result_image)
        if pose_points:
            # center_x, center_y, pose_height: 人体中心和高度 / Pose center and height.
            center_x, center_y, pose_height = self.calculate_pose_geometry(pose_points)
            cv.circle(result_image, (int(center_x), int(center_y)), 8, (0, 255, 255), -1)
            self.draw_pose_box(result_image, pose_points)
            self.execute_rgb_only_follow(center_x, pose_height, result_image.shape[1])
            self.draw_overlay_text(result_image, fps, center_x, center_y, pose_height, True)
        else:
            self.latest_action_text = "Pose lost - Stop"
            self.publish_zero_twist()
            self.linear_pid_controller.reset()
            self.angular_pid_controller.reset()
            self.draw_overlay_text(result_image, fps, 0.0, 0.0, 0.0, False)

        self.update_debug_window(result_image)

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

    @staticmethod
    def calculate_pose_geometry(pose_points: List[List[float]]) -> Tuple[float, float, float]:
        """
        根据姿态关键点计算人体中心和人体框高度 / Calculate pose center and height from pose key points.

        Args:
            pose_points: 姿态关键点列表 / Pose key point list.

        Returns:
            Tuple[float, float, float]: 中心 x、中心 y、人体框高度 / Center x, center y, pose height.
        """
        # x_values: 姿态关键点 x 坐标列表 / Pose key point x coordinates.
        x_values = [float(point[1]) for point in pose_points]
        # y_values: 姿态关键点 y 坐标列表 / Pose key point y coordinates.
        y_values = [float(point[2]) for point in pose_points]
        # center_x: 人体中心 x / Pose center x.
        center_x = float(sum(x_values) / len(x_values))
        # center_y: 人体中心 y / Pose center y.
        center_y = float(sum(y_values) / len(y_values))
        # pose_height: 人体框高度 / Pose bounding height.
        pose_height = float(max(y_values) - min(y_values)) if y_values else 0.0
        return center_x, center_y, pose_height

    @staticmethod
    def draw_pose_box(image_frame: np.ndarray, pose_points: List[List[float]]) -> None:
        """
        绘制人体姿态外接框 / Draw pose bounding box.

        Args:
            image_frame: 显示图像 / Display image.
            pose_points: 姿态关键点列表 / Pose key point list.
        """
        # x_values: 姿态关键点 x 坐标列表 / Pose key point x coordinates.
        x_values = [int(point[1]) for point in pose_points]
        # y_values: 姿态关键点 y 坐标列表 / Pose key point y coordinates.
        y_values = [int(point[2]) for point in pose_points]
        if not x_values or not y_values:
            return
        cv.rectangle(image_frame, (min(x_values), min(y_values)), (max(x_values), max(y_values)), (0, 255, 0), 2)

    def execute_rgb_only_follow(self, center_x: float, pose_height: float, image_width: int) -> None:
        """
        基于 RGB 姿态中心和人体框高度执行跟随 / Follow using RGB pose center and pose height.

        Args:
            center_x: 人体中心 x / Pose center x.
            pose_height: 人体框高度 / Pose bounding height.
            image_width: 图像宽度 / Image width.
        """
        if self.joy_is_active:
            self.publish_zero_twist()
            return

        # image_center_x: 图像中心 x / Image center x.
        image_center_x = image_width * 0.5
        # center_error_x: 横向中心误差，右侧为正 / Horizontal center error, positive when target is right.
        center_error_x = center_x - image_center_x
        # height_error: 人体框高度误差，目标高度大于当前高度时表示目标偏远 / Pose height error.
        height_error = self.target_pose_height_pixel - pose_height

        # linear_velocity: 线速度命令 / Linear velocity command.
        linear_velocity = 0.0
        if self.enable_linear_follow and abs(height_error) > self.pose_height_deadband_pixel:
            linear_velocity = self.linear_pid_controller.update_from_error(height_error)
            if self.reverse_linear_direction:
                linear_velocity = -linear_velocity

        # angular_velocity: 角速度命令 / Angular velocity command.
        angular_velocity = 0.0
        if abs(center_error_x) > self.center_deadband_pixel:
            angular_velocity = -self.angular_pid_controller.update_from_error(center_error_x)
            if self.reverse_angular_direction:
                angular_velocity = -angular_velocity

        linear_velocity = float(np.clip(linear_velocity, -self.max_linear_x, self.max_linear_x))
        angular_velocity = float(np.clip(angular_velocity, -self.max_angular_z, self.max_angular_z))

        # twist_command: 底盘速度命令 / Chassis velocity command.
        twist_command = Twist()
        twist_command.linear.x = linear_velocity
        twist_command.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist_command)
        self.latest_twist_command = twist_command
        self.latest_action_text = "Tracking pose"

    def publish_zero_twist(self) -> None:
        """发布零速度 / Publish zero velocity."""
        # zero_twist: 零速度命令 / Zero velocity command.
        zero_twist = Twist()
        self.cmd_vel_publisher.publish(zero_twist)
        self.latest_twist_command = zero_twist

    def joy_state_callback(self, joy_msg: Bool) -> None:
        """
        手柄接管回调 / Joystick override callback.

        Args:
            joy_msg: 手柄状态消息 / Joystick state message.
        """
        self.joy_is_active = bool(joy_msg.data)
        if self.joy_is_active:
            self.publish_zero_twist()

    def calculate_fps(self) -> float:
        """
        计算 FPS / Calculate FPS.

        Returns:
            float: FPS 数值 / FPS value.
        """
        # current_time_sec: 当前时间 / Current time.
        current_time_sec = time.perf_counter()
        # fps: 帧率 / Frame rate.
        fps = 1.0 / max(current_time_sec - self.previous_time_sec, 1e-3)
        self.previous_time_sec = current_time_sec
        return fps

    def draw_overlay_text(self, image_frame: np.ndarray, fps: float, center_x: float, center_y: float, pose_height: float, pose_found: bool) -> None:
        """
        绘制画面叠加文字 / Draw overlay text.

        Args:
            image_frame: 显示图像 / Display image.
            fps: 图像帧率 / FPS.
            center_x: 人体中心 x / Pose center x.
            center_y: 人体中心 y / Pose center y.
            pose_height: 人体框高度 / Pose height.
            pose_found: 是否检测到人体 / Whether pose is detected.
        """
        # overlay_lines: 叠加文字列表 / Overlay text lines.
        overlay_lines = [
            "FPS: %d" % int(fps),
            "Pose: %s center=(%d,%d) height=%d" % ("YES" if pose_found else "NO", int(center_x), int(center_y), int(pose_height)),
            "Action: %s" % self.latest_action_text,
            "Cmd: x=%.2f z=%.2f" % (self.latest_twist_command.linear.x, self.latest_twist_command.angular.z),
            "Target: %.1fm ~= height %.0fpx | center deadband: %.0f" % (self.target_follow_distance_meter, self.target_pose_height_pixel, self.center_deadband_pixel),
            "PTZ hold: H=%.1f P=%.1f" % (self.ptz_horizontal_deg, self.ptz_pitch_deg),
        ]
        for line_index, line_text in enumerate(overlay_lines):
            # text_origin: 文字位置 / Text origin.
            text_origin = (6, 18 + line_index * self.overlay_line_height)
            cv.putText(
                image_frame,
                line_text,
                text_origin,
                cv.FONT_HERSHEY_SIMPLEX,
                self.overlay_font_scale,
                (0, 255, 255),
                1,
                cv.LINE_AA,
            )

    def update_debug_window(self, image_frame: np.ndarray) -> None:
        """
        更新调试窗口 / Update debug window.

        Args:
            image_frame: 显示图像 / Display image.
        """
        if not self.show_debug_window:
            return
        if not self.display_window_initialized:
            cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)
            cv.resizeWindow(self.window_name, int(self.display_width), int(self.display_height))
            self.display_window_initialized = True
        # display_frame: 调整后的显示图像 / Resized display frame.
        display_frame = cv.resize(image_frame, (int(self.display_width), int(self.display_height)))
        cv.imshow(self.window_name, display_frame)
        # key_code: 按键编码 / Key code.
        key_code = cv.waitKey(1) & 0xFF
        if key_code in (ord("q"), 27):
            self.get_logger().info("Pressed q/ESC, shutting down poseFollow node.")
            self.cancel()
            os._exit(0)  # pylint: disable=protected-access

    def cancel(self) -> None:
        """停止底盘运动 / Stop chassis movement."""
        self.publish_zero_twist()

    def on_dynamic_parameter_update(self, parameters: List[Parameter]) -> SetParametersResult:
        """
        动态参数回调 / Dynamic parameter callback.

        Args:
            parameters: 参数列表 / Parameter list.

        Returns:
            SetParametersResult: 参数设置结果 / Parameter setting result.
        """
        try:
            # should_publish_ptz_once: 是否立即发布一次云台角度 / Whether to immediately publish PTZ once.
            should_publish_ptz_once = False
            # should_recreate_ptz_hold_timer: 是否重建云台保持定时器 / Whether to recreate PTZ hold timer.
            should_recreate_ptz_hold_timer = False
            for parameter in parameters:
                if parameter.name in RESTART_REQUIRED_PARAMETER_NAMES:
                    return SetParametersResult(successful=False, reason="Parameter '%s' requires restart." % parameter.name)
                if parameter.name not in DYNAMIC_PARAMETER_NAMES:
                    continue
                setattr(self, parameter.name, parameter.value)
                if parameter.name in {"ptz_horizontal_deg", "ptz_pitch_deg"}:
                    should_publish_ptz_once = True
                if parameter.name in {"publish_ptz_hold_command", "ptz_hold_rate_hz"}:
                    should_recreate_ptz_hold_timer = True

            self.max_linear_x = abs(float(self.max_linear_x))
            self.max_angular_z = abs(float(self.max_angular_z))
            self.ptz_hold_rate_hz = max(float(self.ptz_hold_rate_hz), MINIMUM_PTZ_HOLD_RATE_HZ)

            if any(parameter.name in {"linear_Kp", "linear_Ki", "linear_Kd"} for parameter in parameters):
                self.linear_pid_controller.update_gains(self.linear_Kp / 1000.0, self.linear_Ki / 1000.0, self.linear_Kd / 1000.0)
            if any(parameter.name in {"angular_Kp", "angular_Ki", "angular_Kd"} for parameter in parameters):
                self.angular_pid_controller.update_gains(self.angular_Kp / 100.0, self.angular_Ki / 100.0, self.angular_Kd / 100.0)
            if should_publish_ptz_once:
                self.publish_ptz_angle(self.ptz_horizontal_deg, self.ptz_pitch_deg)
            if should_recreate_ptz_hold_timer:
                if self.ptz_hold_timer is not None:
                    self.destroy_timer(self.ptz_hold_timer)
                    self.ptz_hold_timer = None
                if self.publish_ptz_hold_command:
                    self.ptz_hold_timer = self.create_timer(
                        1.0 / max(self.ptz_hold_rate_hz, MINIMUM_PTZ_HOLD_RATE_HZ),
                        self.publish_ptz_hold_command_callback,
                    )
            return SetParametersResult(successful=True)
        except Exception as exception:  # pylint: disable=broad-except
            return SetParametersResult(successful=False, reason=str(exception))

    def destroy_node(self) -> bool:
        """
        销毁节点并释放资源 / Destroy node and release resources.

        Returns:
            bool: 销毁结果 / Destroy result.
        """
        self.cancel()
        try:
            self.pose_detector.close()
        except Exception:
            pass
        try:
            cv.destroyAllWindows()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """
    节点入口函数 / Node entry point.

    Args:
        args: ROS 参数 / ROS arguments.
    """
    rclpy.init(args=args)
    # pose_follow_node: 人体姿态跟随节点 / Pose following node.
    pose_follow_node = PoseFollowNode()
    try:
        rclpy.spin(pose_follow_node)
    except KeyboardInterrupt:
        pose_follow_node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        pose_follow_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
