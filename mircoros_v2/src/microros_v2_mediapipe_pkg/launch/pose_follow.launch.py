#!/usr/bin/env python3
# encoding: utf-8
"""
pose_follow_compressed.launch.py

中文说明:
    ROS2 Humble 人体姿态跟随节点启动文件。
    本版本删除深度相机相关参数，默认只订阅原始图像话题 /camera/image_raw 进行姿态检测。
    同时通过 /cmd_ptz_angle 保持云台 horizontal=0 deg、pitch=0 deg；RGB 单目 1m 跟随默认标定为人体框高度 300px。

English Description:
    Launch file for ROS2 Humble pose following node.
    This version removes depth-camera parameters and only subscribes to the raw image topic
    /camera/image_raw for pose detection. It also keeps PTZ at horizontal=0 deg and pitch=0 deg
    through /cmd_ptz_angle. RGB-only 1m following is approximated by a 300px pose height.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# POSE_FOLLOW_PACKAGE_NAME: 姿态跟随功能包名称 / Pose follow package name.
POSE_FOLLOW_PACKAGE_NAME = "microros_v2_mediapipe_pkg"

# POSE_FOLLOW_EXECUTABLE_NAME: 姿态跟随节点可执行文件名称 / Pose follow executable name.
POSE_FOLLOW_EXECUTABLE_NAME = "pose_follow_node"

# POSE_FOLLOW_NODE_NAME: 姿态跟随节点名称 / Pose follow node name.
POSE_FOLLOW_NODE_NAME = "pose_follow_node"


def generate_launch_description():
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "rgb_topic",
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
            description="手柄接管状态话题 / Joystick override state topic.",
        ),
        DeclareLaunchArgument(
            "ptz_command_topic",
            default_value="/cmd_ptz_angle",
            description="云台角度输出话题 / PTZ angle output topic.",
        ),
        DeclareLaunchArgument(
            "linear_Kp",
            default_value="10.0",
            description="视觉尺度线速度 PID Kp / Visual-size linear PID Kp.",
        ),
        DeclareLaunchArgument(
            "linear_Ki",
            default_value="0.0",
            description="视觉尺度线速度 PID Ki / Visual-size linear PID Ki.",
        ),
        DeclareLaunchArgument(
            "linear_Kd",
            default_value="1.0",
            description="视觉尺度线速度 PID Kd / Visual-size linear PID Kd.",
        ),
        DeclareLaunchArgument(
            "angular_Kp",
            default_value="0.2",
            description="人体中心角速度 PID Kp / Pose center angular PID Kp.",
        ),
        DeclareLaunchArgument(
            "angular_Ki",
            default_value="0.0",
            description="人体中心角速度 PID Ki / Pose center angular PID Ki.",
        ),
        DeclareLaunchArgument(
            "angular_Kd",
            default_value="2.0",
            description="人体中心角速度 PID Kd / Pose center angular PID Kd.",
        ),
        DeclareLaunchArgument(
            "target_follow_distance_meter",
            default_value="2.0",
            description="目标跟随距离，仅用于 RGB 单目视觉标定说明 / Target following distance, used as RGB-only calibration label.",
        ),
        DeclareLaunchArgument(
            "target_pose_height_pixel",
            default_value="1400.0",
            description="1m 跟随对应的人体框高度像素标定值 / Calibrated pose height in pixels for about 1m following.",
        ),
        DeclareLaunchArgument(
            "pose_height_deadband_pixel",
            default_value="25.0",
            description="人体框高度死区像素 / Pose height deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "center_deadband_pixel",
            default_value="30.0",
            description="人体中心横向死区像素 / Pose center horizontal deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "max_linear_x",
            default_value="0.3",
            description="最大线速度 / Maximum linear speed.",
        ),
        DeclareLaunchArgument(
            "max_angular_z",
            default_value="2.0",
            description="最大角速度 / Maximum angular speed.",
        ),
        DeclareLaunchArgument(
            "reverse_linear_direction",
            default_value="false",
            description="是否反转线速度方向 / Whether to reverse linear direction.",
        ),
        DeclareLaunchArgument(
            "reverse_angular_direction",
            default_value="false",
            description="是否反转角速度方向 / Whether to reverse angular direction.",
        ),
        DeclareLaunchArgument(
            "enable_linear_follow",
            default_value="true",
            description="是否启用基于人体框高度的前后跟随 / Whether to enable visual-size linear following.",
        ),
        DeclareLaunchArgument(
            "publish_initial_ptz_command",
            default_value="true",
            description="启动时是否重复发布云台初始角度 / Whether to repeatedly publish initial PTZ angle at startup.",
        ),
        DeclareLaunchArgument(
            "publish_ptz_hold_command",
            default_value="true",
            description="运行中是否持续发布云台保持角度 / Whether to continuously publish PTZ hold angle while running.",
        ),
        DeclareLaunchArgument(
            "ptz_horizontal_deg",
            default_value="0.0",
            description="云台保持水平角度 / PTZ hold horizontal angle.",
        ),
        DeclareLaunchArgument(
            "ptz_pitch_deg",
            default_value="-22.0",
            description="云台保持俯仰角度，当前设备 0 deg 约等于相对水平上抬 45 deg / PTZ hold pitch angle.",
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
            "ptz_hold_rate_hz",
            default_value="2.0",
            description="运行中云台角度保持发布频率 / PTZ hold publish rate while running.",
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
            "show_debug_window",
            default_value="true",
            description="是否显示调试窗口 / Whether to show debug window.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="pose_follow",
            description="OpenCV 显示窗口名称 / OpenCV display window name.",
        ),
        DeclareLaunchArgument(
            "display_width",
            default_value="960",
            description="显示窗口宽度 / Display window width.",
        ),
        DeclareLaunchArgument(
            "display_height",
            default_value="720",
            description="显示窗口高度 / Display window height.",
        ),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="0.38",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="15",
            description="画面叠加文字行高 / Overlay text line height.",
        ),
    ]

    # pose_follow_node: 姿态跟随节点 / Pose following node.
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

    pose_follow_node = Node(
        package=POSE_FOLLOW_PACKAGE_NAME,
        executable=POSE_FOLLOW_EXECUTABLE_NAME,
        name=POSE_FOLLOW_NODE_NAME,
        output="screen",
        parameters=[{
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "linear_Kp": ParameterValue(LaunchConfiguration("linear_Kp"), value_type=float),
            "linear_Ki": ParameterValue(LaunchConfiguration("linear_Ki"), value_type=float),
            "linear_Kd": ParameterValue(LaunchConfiguration("linear_Kd"), value_type=float),
            "angular_Kp": ParameterValue(LaunchConfiguration("angular_Kp"), value_type=float),
            "angular_Ki": ParameterValue(LaunchConfiguration("angular_Ki"), value_type=float),
            "angular_Kd": ParameterValue(LaunchConfiguration("angular_Kd"), value_type=float),
            "target_follow_distance_meter": ParameterValue(LaunchConfiguration("target_follow_distance_meter"), value_type=float),
            "target_pose_height_pixel": ParameterValue(LaunchConfiguration("target_pose_height_pixel"), value_type=float),
            "pose_height_deadband_pixel": ParameterValue(LaunchConfiguration("pose_height_deadband_pixel"), value_type=float),
            "center_deadband_pixel": ParameterValue(LaunchConfiguration("center_deadband_pixel"), value_type=float),
            "max_linear_x": ParameterValue(LaunchConfiguration("max_linear_x"), value_type=float),
            "max_angular_z": ParameterValue(LaunchConfiguration("max_angular_z"), value_type=float),
            "reverse_linear_direction": ParameterValue(LaunchConfiguration("reverse_linear_direction"), value_type=bool),
            "reverse_angular_direction": ParameterValue(LaunchConfiguration("reverse_angular_direction"), value_type=bool),
            "enable_linear_follow": ParameterValue(LaunchConfiguration("enable_linear_follow"), value_type=bool),
            "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
            "publish_ptz_hold_command": ParameterValue(LaunchConfiguration("publish_ptz_hold_command"), value_type=bool),
            "ptz_horizontal_deg": ParameterValue(LaunchConfiguration("ptz_horizontal_deg"), value_type=float),
            "ptz_pitch_deg": ParameterValue(LaunchConfiguration("ptz_pitch_deg"), value_type=float),
            "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
            "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
            "ptz_hold_rate_hz": ParameterValue(LaunchConfiguration("ptz_hold_rate_hz"), value_type=float),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "window_name": LaunchConfiguration("window_name"),
            "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
            "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [pose_follow_node])
