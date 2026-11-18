#!/usr/bin/env python3
# encoding: utf-8
"""
robot_ctrl.launch.py

中文说明:
    ROS2 Humble 手掌位置遥控节点启动文件。
    默认订阅原始图像话题 /camera/image_raw，云台只通过 /cmd_ptz_angle 初始化角度。
    当前设备 pitch=-45 deg 为水平朝向，因此默认 initial_pitch_deg=-45.0。

English Description:
    Launch file for ROS2 Humble palm-position robot control node.
    The default input is the raw image topic /camera/image_raw. PTZ initialization only uses
    /cmd_ptz_angle. On the current device, pitch=-45 deg means horizontal view, so initial_pitch_deg=-45.0 by default.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# MEDIAPIPE_PACKAGE_NAME: MediaPipe 功能包名称 / MediaPipe package name.
MEDIAPIPE_PACKAGE_NAME = "microros_v2_mediapipe_pkg"

# ROBOT_CONTROL_EXECUTABLE_NAME: 手掌位置遥控节点可执行文件名称 / Robot control node executable name.
ROBOT_CONTROL_EXECUTABLE_NAME = "robot_ctrl_node"

# ROBOT_CONTROL_NODE_NAME: 手掌位置遥控节点运行名称 / Robot control node runtime name.
ROBOT_CONTROL_NODE_NAME = "robot_ctrl_node"


def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="底盘速度输出话题 / Chassis velocity output topic.",
        ),
        DeclareLaunchArgument(
            "joy_state_topic",
            default_value="/JoyState",
            description="手柄接管状态输入话题 / Joystick override state input topic.",
        ),
        DeclareLaunchArgument(
            "ptz_command_topic",
            default_value="/cmd_ptz_angle",
            description="云台角度输出话题 / PTZ angle output topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="robot_ctrl",
            description="OpenCV 调试窗口名称 / OpenCV debug window name.",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true",
            description="是否显示 OpenCV 调试窗口 / Whether to show OpenCV debug window.",
        ),
        DeclareLaunchArgument(
            "display_width",
            default_value="960",
            description="OpenCV 视频显示宽度 / OpenCV video display width.",
        ),
        DeclareLaunchArgument(
            "display_height",
            default_value="720",
            description="OpenCV 视频显示高度 / OpenCV video display height.",
        ),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="0.32",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="13",
            description="画面叠加文字行高 / Overlay text line height.",
        ),
        DeclareLaunchArgument(
            "overlay_show_background",
            default_value="false",
            description="是否显示文字背景框 / Whether to show overlay text background.",
        ),
        DeclareLaunchArgument(
            "overlay_show_legend",
            default_value="false",
            description="是否显示手掌位置控制说明 / Whether to show palm-position control legend.",
        ),
        DeclareLaunchArgument(
            "enable_horizontal_flip",
            default_value="false",
            description="是否水平翻转图像 / Whether to horizontally flip image.",
        ),
        DeclareLaunchArgument(
            "enable_vertical_flip",
            default_value="false",
            description="是否垂直翻转图像 / Whether to vertically flip image.",
        ),
        DeclareLaunchArgument(
            "rotate_image_code",
            default_value="none",
            description="图像旋转: none/90_clockwise/90_counterclockwise/180 / Image rotation code.",
        ),
        DeclareLaunchArgument(
            "linear_speed",
            default_value="0.20",
            description="手掌位置控制线速度 / Linear speed for palm-position control.",
        ),
        DeclareLaunchArgument(
            "angular_speed",
            default_value="1.50",
            description="手掌位置控制角速度 / Angular speed for palm-position control.",
        ),
        DeclareLaunchArgument(
            "forward_y_threshold",
            default_value="150",
            description="手掌偏上触发前进的 y 阈值 / Y threshold for forward command.",
        ),
        DeclareLaunchArgument(
            "back_y_threshold",
            default_value="270",
            description="手掌偏下触发后退的 y 阈值 / Y threshold for back command.",
        ),
        DeclareLaunchArgument(
            "left_x_threshold",
            default_value="300",
            description="手掌偏左触发左转的 x 阈值 / X threshold for left turn command.",
        ),
        DeclareLaunchArgument(
            "right_x_threshold",
            default_value="350",
            description="手掌偏右触发右转的 x 阈值 / X threshold for right turn command.",
        ),
        DeclareLaunchArgument(
            "hand_lost_timeout_sec",
            default_value="0.5",
            description="手部丢失超时时间 / Hand lost timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="底盘控制频率 / Chassis control rate.",
        ),
        DeclareLaunchArgument(
            "image_processing_rate_hz",
            default_value="30.0",
            description="图像处理频率 / Image processing rate.",
        ),
        DeclareLaunchArgument(
            "stop_when_hand_lost",
            default_value="true",
            description="手部丢失时是否停车 / Whether to stop when hand is lost.",
        ),
        DeclareLaunchArgument(
            "auto_start_control",
            default_value="true",
            description="启动后是否自动开始手掌位置控制 / Whether to auto-start palm-position control.",
        ),
        DeclareLaunchArgument(
            "enable_joy_override",
            default_value="true",
            description="是否启用手柄接管停车 / Whether to enable joystick override stop.",
        ),
        DeclareLaunchArgument(
            "publish_initial_ptz_command",
            default_value="true",
            description="启动时是否发布初始云台角度 / Whether to publish initial PTZ command.",
        ),
        DeclareLaunchArgument(
            "initial_horizontal_deg",
            default_value="0.0",
            description="初始水平角度 / Initial horizontal angle.",
        ),
        DeclareLaunchArgument(
            "initial_pitch_deg",
            default_value="0.0",
            description="初始俯仰角度，当前设备 -45 deg 为水平朝向 / Initial pitch angle; -45 deg is horizontal on current device.",
        ),
        DeclareLaunchArgument(
            "initial_publish_count",
            default_value="20",
            description="初始云台角度重复发布次数 / Initial PTZ repeated publish count.",
        ),
        DeclareLaunchArgument(
            "initial_publish_interval_sec",
            default_value="0.2",
            description="初始云台角度重复发布间隔 / Initial PTZ repeated publish interval.",
        ),
        DeclareLaunchArgument(
            "debug_log_interval_sec",
            default_value="2.0",
            description="调试日志打印间隔 / Debug log interval.",
        ),
        DeclareLaunchArgument(
            "print_cmd_vel",
            default_value="false",
            description="是否打印底盘速度命令 / Whether to print cmd_vel.",
        ),
        DeclareLaunchArgument(
            "print_interval_sec",
            default_value="0.5",
            description="速度命令打印间隔 / Cmd_vel print interval.",
        ),
        DeclareLaunchArgument(
            "enable_rqt_dynamic_parameters",
            default_value="true",
            description="是否允许 rqt/ros2 param 动态调参 / Whether to enable rqt/ros2 param dynamic parameters.",
        ),
    ]

    # robot_control_node: 手掌位置遥控节点 / Palm-position robot control node.
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

    robot_control_node = Node(
        package=MEDIAPIPE_PACKAGE_NAME,
        executable=ROBOT_CONTROL_EXECUTABLE_NAME,
        name=ROBOT_CONTROL_NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
            "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "overlay_show_legend": ParameterValue(LaunchConfiguration("overlay_show_legend"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "linear_speed": ParameterValue(LaunchConfiguration("linear_speed"), value_type=float),
            "angular_speed": ParameterValue(LaunchConfiguration("angular_speed"), value_type=float),
            "forward_y_threshold": ParameterValue(LaunchConfiguration("forward_y_threshold"), value_type=int),
            "back_y_threshold": ParameterValue(LaunchConfiguration("back_y_threshold"), value_type=int),
            "left_x_threshold": ParameterValue(LaunchConfiguration("left_x_threshold"), value_type=int),
            "right_x_threshold": ParameterValue(LaunchConfiguration("right_x_threshold"), value_type=int),
            "hand_lost_timeout_sec": ParameterValue(LaunchConfiguration("hand_lost_timeout_sec"), value_type=float),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "stop_when_hand_lost": ParameterValue(LaunchConfiguration("stop_when_hand_lost"), value_type=bool),
            "auto_start_control": ParameterValue(LaunchConfiguration("auto_start_control"), value_type=bool),
            "enable_joy_override": ParameterValue(LaunchConfiguration("enable_joy_override"), value_type=bool),
            "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
            "initial_horizontal_deg": ParameterValue(LaunchConfiguration("initial_horizontal_deg"), value_type=float),
            "initial_pitch_deg": ParameterValue(LaunchConfiguration("initial_pitch_deg"), value_type=float),
            "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
            "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
            "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
            "print_cmd_vel": ParameterValue(LaunchConfiguration("print_cmd_vel"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [robot_control_node])
