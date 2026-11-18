#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qr_control.launch.py

中文说明:
    ROS2 Humble 二维码控制节点启动文件（No-PTZ 版）。
    本启动文件将原始图像输入、二维码识别参数、底盘动作速度和调试显示参数全部参数化。
    摄像头固定朝向正前方，无云台控制。

English Description:
    Launch file for ROS2 Humble QR code control node (No-PTZ version).
    This launch file parameterizes raw image input, QR detection parameters, chassis action speeds,
    and debug display options. Camera is fixed forward, no PTZ control.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_no_ptz_visual_control_pkg"
EXECUTABLE_NAME = "qr_control_node"
NODE_NAME = "qr_control_node"


def generate_launch_description() -> LaunchDescription:
    launch_arguments = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw",
                              description="原始图像输入话题 / Raw image input topic."),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel",
                              description="底盘速度输出话题 / Chassis velocity output topic."),
        DeclareLaunchArgument("joy_state_topic", default_value="/JoyState",
                              description="手柄接管状态输入话题 / Joystick override state input topic."),
        DeclareLaunchArgument("beep_topic", default_value="/beep",
                              description="蜂鸣器输出话题 / Buzzer output topic."),
        DeclareLaunchArgument("window_name", default_value="qr_control",
                              description="OpenCV 调试窗口名称 / OpenCV debug window name."),
        DeclareLaunchArgument("show_debug_window", default_value="true",
                              description="是否显示 OpenCV 调试窗口 / Whether to show OpenCV debug window."),
        DeclareLaunchArgument("window_width", default_value="640",
                              description="OpenCV 调试窗口宽度 / OpenCV debug window width."),
        DeclareLaunchArgument("window_height", default_value="480",
                              description="OpenCV 调试窗口高度 / OpenCV debug window height."),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.38",
                              description="画面叠加文字大小 / Overlay text font scale."),
        DeclareLaunchArgument("overlay_line_height", default_value="15",
                              description="画面叠加文字行高 / Overlay text line height."),
        DeclareLaunchArgument("overlay_margin_x", default_value="6",
                              description="画面叠加文字左边距 / Overlay text left margin."),
        DeclareLaunchArgument("overlay_margin_y", default_value="16",
                              description="画面叠加文字上边距 / Overlay text top margin."),
        DeclareLaunchArgument("overlay_show_background", default_value="false",
                              description="是否显示文字背景框 / Whether to show overlay text background."),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false",
                              description="是否水平翻转图像 / Whether to horizontally flip image."),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false",
                              description="是否垂直翻转图像 / Whether to vertically flip image."),
        DeclareLaunchArgument("rotate_image_code", default_value="none",
                              description="图像旋转: none/90_clockwise/90_counterclockwise/180 / Image rotation code."),
        DeclareLaunchArgument("qr_detector_backend", default_value="opencv",
                              description="二维码检测后端: pyzbar/opencv/auto / QR detector backend."),
        DeclareLaunchArgument("qr_select_strategy", default_value="largest",
                              description="多二维码选择策略: largest/nearest_center/first / QR selection strategy."),
        DeclareLaunchArgument("qr_command_hold_timeout_sec", default_value="1.5",
                              description="二维码命令保持时间 / QR command hold timeout."),
        DeclareLaunchArgument("forward_linear_speed", default_value="0.30",
                              description="forward 命令线速度 / Linear speed for forward command."),
        DeclareLaunchArgument("back_linear_speed", default_value="-0.30",
                              description="back 命令线速度 / Linear speed for back command."),
        DeclareLaunchArgument("rotate_angular_speed", default_value="1.00",
                              description="left/right 原地旋转角速度 / Angular speed for left/right commands."),
        DeclareLaunchArgument("turn_linear_speed", default_value="0.30",
                              description="turnleft/turnright 命令线速度 / Linear speed for turn commands."),
        DeclareLaunchArgument("turn_angular_speed", default_value="1.50",
                              description="turnleft/turnright 命令角速度 / Angular speed for turn commands."),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0",
                              description="底盘控制频率 / Chassis control rate."),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="30.0",
                              description="图像处理频率 / Image processing rate."),
        DeclareLaunchArgument("auto_start_control", default_value="false",
                              description="启动后是否自动开始二维码控制 / Whether to auto-start QR control."),
        DeclareLaunchArgument("stop_when_qr_lost", default_value="true",
                              description="二维码丢失时是否停车 / Whether to stop when QR is lost."),
        DeclareLaunchArgument("unknown_command_stop", default_value="true",
                              description="未知二维码命令时是否停车 / Whether to stop on unknown QR command."),
        DeclareLaunchArgument("enable_joy_override", default_value="true",
                              description="是否启用手柄接管停车 / Whether to enable joystick override stop."),
        DeclareLaunchArgument("enable_beep", default_value="false",
                              description="未知二维码命令时是否蜂鸣 / Whether to beep on unknown QR command."),
        DeclareLaunchArgument("debug_log_interval_sec", default_value="2.0",
                              description="调试日志打印间隔 / Debug log interval."),
        DeclareLaunchArgument("print_cmd_vel", default_value="false",
                              description="是否打印底盘速度命令 / Whether to print cmd_vel."),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5",
                              description="速度命令打印间隔 / Cmd_vel print interval."),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true",
                              description="是否允许 rqt/ros2 param 动态调参 / Whether to enable dynamic parameters."),
    ]

    venv_pythonpath = os.path.join(
        os.path.expanduser("~"),
        "microros_ws",
        "python_env",
        "mircoros_v2_env",
        ".venv",
        "lib",
        "python3.10",
        "site-packages",
    )
    original_pythonpath = os.environ.get("PYTHONPATH", "")

    qr_control_node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE_NAME,
        name=NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
            "beep_topic": LaunchConfiguration("beep_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_margin_x": ParameterValue(LaunchConfiguration("overlay_margin_x"), value_type=int),
            "overlay_margin_y": ParameterValue(LaunchConfiguration("overlay_margin_y"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "qr_detector_backend": LaunchConfiguration("qr_detector_backend"),
            "qr_select_strategy": LaunchConfiguration("qr_select_strategy"),
            "qr_command_hold_timeout_sec": ParameterValue(LaunchConfiguration("qr_command_hold_timeout_sec"), value_type=float),
            "forward_linear_speed": ParameterValue(LaunchConfiguration("forward_linear_speed"), value_type=float),
            "back_linear_speed": ParameterValue(LaunchConfiguration("back_linear_speed"), value_type=float),
            "rotate_angular_speed": ParameterValue(LaunchConfiguration("rotate_angular_speed"), value_type=float),
            "turn_linear_speed": ParameterValue(LaunchConfiguration("turn_linear_speed"), value_type=float),
            "turn_angular_speed": ParameterValue(LaunchConfiguration("turn_angular_speed"), value_type=float),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "auto_start_control": ParameterValue(LaunchConfiguration("auto_start_control"), value_type=bool),
            "stop_when_qr_lost": ParameterValue(LaunchConfiguration("stop_when_qr_lost"), value_type=bool),
            "unknown_command_stop": ParameterValue(LaunchConfiguration("unknown_command_stop"), value_type=bool),
            "enable_joy_override": ParameterValue(LaunchConfiguration("enable_joy_override"), value_type=bool),
            "enable_beep": ParameterValue(LaunchConfiguration("enable_beep"), value_type=bool),
            "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
            "print_cmd_vel": ParameterValue(LaunchConfiguration("print_cmd_vel"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [qr_control_node])
