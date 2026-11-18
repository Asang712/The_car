#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MicroROS-V2 color tracker PTZ launch file.
MicroROS-V2 颜色追踪云台控制启动文件。

This launch file starts color_tracker_ptz_node and defaults to the raw image topic.
本启动文件用于启动 color_tracker_ptz_node，并默认订阅原始图像话题。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Package name / 功能包名称
VISUAL_CONTROL_PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"

# Node executable name / 节点可执行文件名称
COLOR_TRACKER_PTZ_EXECUTABLE_NAME = "color_tracker_ptz_node"

# Node runtime name / 节点运行名称
COLOR_TRACKER_PTZ_NODE_NAME = "color_tracker_ptz_node"


def generate_launch_description():
    """Create ROS2 launch description for color tracking PTZ node.

    创建颜色追踪云台控制节点的 ROS2 启动描述。
    """
    # Package share directory path / 功能包 share 目录路径
    visual_control_package_share_directory = get_package_share_directory(VISUAL_CONTROL_PACKAGE_NAME)

    # Default YAML config file path / 默认 YAML 配置文件路径
    default_tracker_config_file_path = os.path.join(
        visual_control_package_share_directory,
        "config",
        "color_tracker_ptz.yaml",
    )

    # Default HSV threshold file path / 默认 HSV 阈值文件路径
    default_hsv_threshold_file_path = os.path.join(
        visual_control_package_share_directory,
        "config",
        "color_hsv.txt",
    )

    # Launch argument: YAML config file / 启动参数：YAML 配置文件
    config_file_launch_argument = DeclareLaunchArgument(
        "config_file",
        default_value=default_tracker_config_file_path,
        description="Path to color tracker PTZ YAML config file / 颜色追踪云台 YAML 配置文件路径",
    )

    # Launch argument: input image topic / 启动参数：输入图像话题
    image_topic_launch_argument = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/image_raw",
        description="Input raw camera image topic / 输入原始摄像头图像话题",
    )

    # Launch argument: PTZ command topic / 启动参数：云台命令话题
    ptz_command_topic_launch_argument = DeclareLaunchArgument(
        "ptz_topic",
        default_value="/cmd_ptz_angle",
        description="PTZ Vector3 command topic / 云台 Vector3 控制话题",
    )

    # Launch argument: HSV threshold file / 启动参数：HSV 阈值文件
    hsv_file_launch_argument = DeclareLaunchArgument(
        "hsv_file",
        default_value=default_hsv_threshold_file_path,
        description="HSV threshold file path / HSV 阈值文件路径",
    )

    # Launch argument: OpenCV debug window switch / 启动参数：OpenCV 调试窗口开关
    display_debug_window_launch_argument = DeclareLaunchArgument(
        "display_debug_window",
        default_value="true",
        description="Show OpenCV debug window / 显示 OpenCV 调试窗口",
    )

    # Launch argument: auto start tracking switch / 启动参数：自动开始追踪开关
    auto_start_tracking_launch_argument = DeclareLaunchArgument(
        "auto_start_tracking",
        default_value="false",
        description="Start tracking immediately after launch if HSV is available / HSV 可用时启动后立即追踪",
    )

    # Launch argument: rqt dynamic parameter switch / 启动参数：rqt 动态调参开关
    enable_rqt_dynamic_parameters_launch_argument = DeclareLaunchArgument(
        "enable_rqt_dynamic_parameters",
        default_value="true",
        description="Enable rqt/ros2 param runtime tuning / 启用 rqt 或 ros2 param 运行时动态调参",
    )

    # ROS2 node action / ROS2 节点启动动作
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

    color_tracker_ptz_node_action = Node(
        package=VISUAL_CONTROL_PACKAGE_NAME,
        executable=COLOR_TRACKER_PTZ_EXECUTABLE_NAME,
        name=COLOR_TRACKER_PTZ_NODE_NAME,
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "ptz_topic": LaunchConfiguration("ptz_topic"),
                "hsv_file": LaunchConfiguration("hsv_file"),
                "display_debug_window": LaunchConfiguration("display_debug_window"),
                "auto_start_tracking": LaunchConfiguration("auto_start_tracking"),
                "enable_rqt_dynamic_parameters": LaunchConfiguration("enable_rqt_dynamic_parameters"),
            },
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription([
        config_file_launch_argument,
        image_topic_launch_argument,
        ptz_command_topic_launch_argument,
        hsv_file_launch_argument,
        display_debug_window_launch_argument,
        auto_start_tracking_launch_argument,
        enable_rqt_dynamic_parameters_launch_argument,
        color_tracker_ptz_node_action,
    ])
