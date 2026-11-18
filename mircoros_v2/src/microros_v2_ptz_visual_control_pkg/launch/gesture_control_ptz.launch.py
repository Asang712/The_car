#!/usr/bin/env python3
# encoding: utf-8
"""
gesture_control_ptz.launch.py

中文说明:
    ROS2 Humble 手势控制节点启动文件。
    默认订阅原始图像话题 /camera/image_raw，支持调大视频显示窗口尺寸，默认使用小字体显示。

English Description:
    Launch file for ROS2 Humble hand gesture control node.
    The default input is the raw image topic /camera/image_raw, and the video display size
    can be enlarged through launch parameters. Overlay text uses a small font by default.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# HAND_CONTROL_PACKAGE_NAME: 手势控制功能包名称 / Hand control package name.
HAND_CONTROL_PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"

# HAND_CONTROL_EXECUTABLE_NAME: 手势控制节点可执行文件名称 / Hand control node executable name.
HAND_CONTROL_EXECUTABLE_NAME = "gesture_control_ptz_node"

# HAND_CONTROL_NODE_NAME: 手势控制节点名称 / Hand control node name.
HAND_CONTROL_NODE_NAME = "gesture_control_ptz_node"


def generate_launch_description():
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # rgb_topic_arg: 原始图像输入话题参数 / Raw image input topic argument.
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/image_raw",
        description="Input raw camera image topic.",
    )
    # cmd_vel_topic_arg: 底盘速度输出话题参数 / Chassis velocity output topic argument.
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="Output velocity command topic.",
    )
    # linear_speed_arg: 手势控制线速度参数 / Linear speed argument for gesture commands.
    linear_speed_arg = DeclareLaunchArgument(
        "linear_speed",
        default_value="0.3",
        description="Linear speed for gesture commands.",
    )
    # angular_speed_arg: 手势控制角速度参数 / Angular speed argument for gesture commands.
    angular_speed_arg = DeclareLaunchArgument(
        "angular_speed",
        default_value="1.5",
        description="Angular speed for gesture commands.",
    )
    # command_duration_arg: 单次手势命令持续时间参数 / Gesture command duration argument.
    command_duration_arg = DeclareLaunchArgument(
        "command_duration",
        default_value="0.5",
        description="Duration for each gesture command.",
    )
    # window_name_arg: OpenCV 窗口名称参数 / OpenCV window name argument.
    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="gesture_control_ptz",
        description="OpenCV display window name.",
    )
    # display_width_arg: 视频显示宽度参数 / Video display width argument.
    display_width_arg = DeclareLaunchArgument(
        "display_width",
        default_value="960",
        description="OpenCV video display width.",
    )
    # display_height_arg: 视频显示高度参数 / Video display height argument.
    display_height_arg = DeclareLaunchArgument(
        "display_height",
        default_value="720",
        description="OpenCV video display height.",
    )
    # overlay_font_scale_arg: 画面叠加文字大小参数 / Overlay text font scale argument.
    overlay_font_scale_arg = DeclareLaunchArgument(
        "overlay_font_scale",
        default_value="0.32",
        description="Overlay text font scale. Smaller default avoids blocking the video frame.",
    )
    # overlay_line_height_arg: 画面叠加文字行高参数 / Overlay text line height argument.
    overlay_line_height_arg = DeclareLaunchArgument(
        "overlay_line_height",
        default_value="13",
        description="Overlay text line height. Smaller default avoids blocking the video frame.",
    )
    # overlay_show_legend_arg: 是否显示手势说明表参数 / Gesture legend display argument.
    overlay_show_legend_arg = DeclareLaunchArgument(
        "overlay_show_legend",
        default_value="false",
        description="Whether to show gesture meaning legend on video frame. Disabled by default to reduce occlusion.",
    )

    # hand_control_node: 手势控制节点 / Hand control node.
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

    hand_control_node = Node(
        package=HAND_CONTROL_PACKAGE_NAME,
        executable=HAND_CONTROL_EXECUTABLE_NAME,
        name=HAND_CONTROL_NODE_NAME,
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "linear_speed": ParameterValue(LaunchConfiguration("linear_speed"), value_type=float),
                "angular_speed": ParameterValue(LaunchConfiguration("angular_speed"), value_type=float),
                "command_duration": ParameterValue(LaunchConfiguration("command_duration"), value_type=float),
                "window_name": LaunchConfiguration("window_name"),
                "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
                "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
                "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
                "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
                "overlay_show_legend": ParameterValue(LaunchConfiguration("overlay_show_legend"), value_type=bool),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg,
            cmd_vel_topic_arg,
            linear_speed_arg,
            angular_speed_arg,
            command_duration_arg,
            window_name_arg,
            display_width_arg,
            display_height_arg,
            overlay_font_scale_arg,
            overlay_line_height_arg,
            overlay_show_legend_arg,
            hand_control_node,
        ]
    )
