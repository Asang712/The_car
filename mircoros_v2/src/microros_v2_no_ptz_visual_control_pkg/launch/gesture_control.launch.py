#!/usr/bin/env python3
# encoding: utf-8
"""
gesture_control.launch.py

中文说明:
    ROS2 Humble 手势控制节点启动文件（No-PTZ 版）。
    默认订阅原始图像话题 /camera/image_raw，摄像头固定朝向正前方，无云台控制。

English Description:
    Launch file for ROS2 Humble hand gesture control node (No-PTZ version).
    The default input is the raw image topic /camera/image_raw.
    Camera is fixed forward, no PTZ control.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_no_ptz_visual_control_pkg"
EXECUTABLE_NAME = "gesture_control_node"
NODE_NAME = "gesture_control_node"


def generate_launch_description():
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic", default_value="/camera/image_raw",
        description="Input raw camera image topic.",
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic", default_value="/cmd_vel",
        description="Output velocity command topic.",
    )
    linear_speed_arg = DeclareLaunchArgument(
        "linear_speed", default_value="0.3",
        description="Linear speed for gesture commands.",
    )
    angular_speed_arg = DeclareLaunchArgument(
        "angular_speed", default_value="1.5",
        description="Angular speed for gesture commands.",
    )
    command_duration_arg = DeclareLaunchArgument(
        "command_duration", default_value="0.5",
        description="Duration for each gesture command.",
    )
    window_name_arg = DeclareLaunchArgument(
        "window_name", default_value="gesture_control",
        description="OpenCV display window name.",
    )
    display_width_arg = DeclareLaunchArgument(
        "display_width", default_value="960",
        description="OpenCV video display width.",
    )
    display_height_arg = DeclareLaunchArgument(
        "display_height", default_value="720",
        description="OpenCV video display height.",
    )
    overlay_font_scale_arg = DeclareLaunchArgument(
        "overlay_font_scale", default_value="0.32",
        description="Overlay text font scale.",
    )
    overlay_line_height_arg = DeclareLaunchArgument(
        "overlay_line_height", default_value="13",
        description="Overlay text line height.",
    )
    overlay_show_legend_arg = DeclareLaunchArgument(
        "overlay_show_legend", default_value="false",
        description="Whether to show gesture meaning legend on video frame.",
    )

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

    node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE_NAME,
        name=NODE_NAME,
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

    return LaunchDescription([
        rgb_topic_arg, cmd_vel_topic_arg,
        linear_speed_arg, angular_speed_arg, command_duration_arg,
        window_name_arg, display_width_arg, display_height_arg,
        overlay_font_scale_arg, overlay_line_height_arg, overlay_show_legend_arg,
        node,
    ])
