#!/usr/bin/env python3
# encoding: utf-8
"""
gesture_follow.launch.py

中文说明:
    ROS2 Humble 手势中心车体 PD 跟随节点启动文件。
    本启动文件只使用原始图像 /camera/image_raw，不使用深度相机。
    图像先缩放到固定计算尺寸 processing_width × processing_height，再进行手势检测和控制计算；
    display_width × display_height 只用于显示，不参与控制计算。
    云台角度固定发布；只有 4 指或 5 指手势允许车体 PD 跟随，其它手势或无手势均停车。

English Description:
    Launch file for ROS2 Humble gesture-center chassis PD following node.
    This launch file uses only /camera/image_raw and does not use a depth camera.
    The image is resized to processing_width × processing_height for gesture detection and control computation;
    display_width × display_height is only for visualization. The PTZ angle is fixed, and only 4-finger
    or 5-finger gestures enable chassis PD following; other gestures or no hand stop the chassis.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# MEDIAPIPE_PACKAGE_NAME: MediaPipe 功能包名称 / MediaPipe package name.
MEDIAPIPE_PACKAGE_NAME = "microros_v2_mediapipe_pkg"

# GESTURE_FOLLOW_EXECUTABLE_NAME: 手势跟随节点可执行文件名称 / Gesture follow node executable name.
GESTURE_FOLLOW_EXECUTABLE_NAME = "gesture_follow_node"

# GESTURE_FOLLOW_NODE_NAME: 手势跟随节点名称 / Gesture follow node name.
GESTURE_FOLLOW_NODE_NAME = "gesture_follow_node"


def generate_launch_description():
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述 / ROS2 launch description.
    """
    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument("rgb_topic", default_value="/camera/image_raw", description="原始图像输入话题 / Raw image input topic."),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel", description="底盘速度输出话题 / Chassis velocity output topic."),
        DeclareLaunchArgument("joy_state_topic", default_value="/JoyState", description="手柄接管状态话题 / Joystick override state topic."),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle", description="固定云台角度输出话题 / Fixed PTZ command topic."),
        DeclareLaunchArgument("window_name", default_value="gesture_follow", description="OpenCV 显示窗口名称 / OpenCV display window name."),

        DeclareLaunchArgument("processing_width", default_value="640", description="控制计算图像宽度 / Processing image width."),
        DeclareLaunchArgument("processing_height", default_value="480", description="控制计算图像高度 / Processing image height."),
        DeclareLaunchArgument("display_width", default_value="960", description="显示窗口宽度 / Display width."),
        DeclareLaunchArgument("display_height", default_value="720", description="显示窗口高度 / Display height."),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.32", description="叠加文字大小 / Overlay font scale."),
        DeclareLaunchArgument("overlay_line_height", default_value="13", description="叠加文字行高 / Overlay line height."),
        DeclareLaunchArgument("overlay_show_background", default_value="false", description="是否显示文字背景 / Whether to show text background."),
        DeclareLaunchArgument("overlay_show_legend", default_value="false", description="是否显示按键说明 / Whether to show key legend."),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false", description="是否水平翻转图像 / Whether to horizontally flip image."),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false", description="是否垂直翻转图像 / Whether to vertically flip image."),
        DeclareLaunchArgument("rotate_image_code", default_value="none", description="图像旋转 none/90_clockwise/90_counterclockwise/180 / Image rotation code."),

        DeclareLaunchArgument("fixed_horizontal_deg", default_value="0.0", description="固定云台水平角 / Fixed PTZ horizontal angle."),
        DeclareLaunchArgument("fixed_pitch_deg", default_value="0.0", description="固定云台俯仰角 / Fixed PTZ pitch angle."),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true", description="启动时是否重复发布固定云台角 / Whether to publish fixed PTZ angle at startup."),
        DeclareLaunchArgument("initial_publish_count", default_value="20", description="启动固定云台角重复发布次数 / Initial PTZ publish count."),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2", description="启动固定云台角发布间隔 / Initial PTZ publish interval."),
        DeclareLaunchArgument("publish_ptz_hold_command", default_value="true", description="运行中是否持续保持固定云台角 / Whether to hold fixed PTZ angle during runtime."),
        DeclareLaunchArgument("ptz_hold_rate_hz", default_value="2.0", description="固定云台角保持发布频率 / Fixed PTZ hold publish rate."),

        DeclareLaunchArgument("horizontal_deadband_pixel", default_value="20.0", description="水平误差死区 / Horizontal error deadband."),
        DeclareLaunchArgument("vertical_deadband_pixel", default_value="20.0", description="竖直误差死区 / Vertical error deadband."),
        DeclareLaunchArgument("horizontal_pd_kp", default_value="0.006", description="水平误差车体转向 PD Kp / Horizontal-to-angular PD Kp."),
        DeclareLaunchArgument("horizontal_pd_kd", default_value="0.0015", description="水平误差车体转向 PD Kd / Horizontal-to-angular PD Kd."),
        DeclareLaunchArgument("vertical_pd_kp", default_value="0.002", description="竖直误差前后移动 PD Kp / Vertical-to-linear PD Kp."),
        DeclareLaunchArgument("vertical_pd_kd", default_value="0.0005", description="竖直误差前后移动 PD Kd / Vertical-to-linear PD Kd."),
        DeclareLaunchArgument("max_linear_x", default_value="0.25", description="最大线速度 / Maximum linear velocity."),
        DeclareLaunchArgument("max_angular_z", default_value="1.2", description="最大角速度 / Maximum angular velocity."),
        DeclareLaunchArgument("linear_velocity_deadband", default_value="0.02", description="线速度输出死区 / Linear velocity output deadband."),
        DeclareLaunchArgument("angular_velocity_deadband", default_value="0.03", description="角速度输出死区 / Angular velocity output deadband."),
        DeclareLaunchArgument("reverse_body_angular_direction", default_value="false", description="是否反转车体角速度方向 / Whether to reverse chassis angular direction."),
        DeclareLaunchArgument("reverse_linear_from_vertical", default_value="true", description="是否反转竖直误差对应线速度方向 / Whether to reverse vertical-to-linear direction."),
        DeclareLaunchArgument("enable_body_angular_control", default_value="true", description="是否启用水平误差车体转向 PD / Whether to enable horizontal-to-angular PD control."),
        DeclareLaunchArgument("enable_body_linear_control", default_value="true", description="是否启用竖直误差前后移动 PD / Whether to enable vertical-to-linear PD control."),

        DeclareLaunchArgument("control_rate_hz", default_value="20.0", description="车体控制频率 / Chassis control rate."),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="30.0", description="图像处理频率 / Image processing rate."),
        DeclareLaunchArgument("hand_lost_timeout_sec", default_value="0.5", description="手势丢失超时时间 / Hand lost timeout."),
        DeclareLaunchArgument("stop_when_hand_lost", default_value="true", description="手势丢失是否停车 / Whether to stop when hand is lost."),
        DeclareLaunchArgument("auto_start_tracking", default_value="true", description="启动后是否自动开始跟随 / Whether to auto-start tracking."),
        DeclareLaunchArgument("print_cmd_vel", default_value="false", description="是否打印速度命令 / Whether to print velocity command."),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5", description="速度命令打印间隔 / Velocity command print interval."),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true", description="是否允许 rqt/ros2 param 动态调参 / Whether to enable dynamic parameters."),
    ]

    # gesture_follow_node: 手势中心车体 PD 跟随节点 / Gesture-center chassis PD following node.
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

    gesture_follow_node = Node(
        package=MEDIAPIPE_PACKAGE_NAME,
        executable=GESTURE_FOLLOW_EXECUTABLE_NAME,
        name=GESTURE_FOLLOW_NODE_NAME,
        output="screen",
        parameters=[{
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "processing_width": ParameterValue(LaunchConfiguration("processing_width"), value_type=int),
            "processing_height": ParameterValue(LaunchConfiguration("processing_height"), value_type=int),
            "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
            "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "overlay_show_legend": ParameterValue(LaunchConfiguration("overlay_show_legend"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "fixed_horizontal_deg": ParameterValue(LaunchConfiguration("fixed_horizontal_deg"), value_type=float),
            "fixed_pitch_deg": ParameterValue(LaunchConfiguration("fixed_pitch_deg"), value_type=float),
            "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
            "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
            "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
            "publish_ptz_hold_command": ParameterValue(LaunchConfiguration("publish_ptz_hold_command"), value_type=bool),
            "ptz_hold_rate_hz": ParameterValue(LaunchConfiguration("ptz_hold_rate_hz"), value_type=float),
            "horizontal_deadband_pixel": ParameterValue(LaunchConfiguration("horizontal_deadband_pixel"), value_type=float),
            "vertical_deadband_pixel": ParameterValue(LaunchConfiguration("vertical_deadband_pixel"), value_type=float),
            "horizontal_pd_kp": ParameterValue(LaunchConfiguration("horizontal_pd_kp"), value_type=float),
            "horizontal_pd_kd": ParameterValue(LaunchConfiguration("horizontal_pd_kd"), value_type=float),
            "vertical_pd_kp": ParameterValue(LaunchConfiguration("vertical_pd_kp"), value_type=float),
            "vertical_pd_kd": ParameterValue(LaunchConfiguration("vertical_pd_kd"), value_type=float),
            "max_linear_x": ParameterValue(LaunchConfiguration("max_linear_x"), value_type=float),
            "max_angular_z": ParameterValue(LaunchConfiguration("max_angular_z"), value_type=float),
            "linear_velocity_deadband": ParameterValue(LaunchConfiguration("linear_velocity_deadband"), value_type=float),
            "angular_velocity_deadband": ParameterValue(LaunchConfiguration("angular_velocity_deadband"), value_type=float),
            "reverse_body_angular_direction": ParameterValue(LaunchConfiguration("reverse_body_angular_direction"), value_type=bool),
            "reverse_linear_from_vertical": ParameterValue(LaunchConfiguration("reverse_linear_from_vertical"), value_type=bool),
            "enable_body_angular_control": ParameterValue(LaunchConfiguration("enable_body_angular_control"), value_type=bool),
            "enable_body_linear_control": ParameterValue(LaunchConfiguration("enable_body_linear_control"), value_type=bool),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "hand_lost_timeout_sec": ParameterValue(LaunchConfiguration("hand_lost_timeout_sec"), value_type=float),
            "stop_when_hand_lost": ParameterValue(LaunchConfiguration("stop_when_hand_lost"), value_type=bool),
            "auto_start_tracking": ParameterValue(LaunchConfiguration("auto_start_tracking"), value_type=bool),
            "print_cmd_vel": ParameterValue(LaunchConfiguration("print_cmd_vel"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [gesture_follow_node])
