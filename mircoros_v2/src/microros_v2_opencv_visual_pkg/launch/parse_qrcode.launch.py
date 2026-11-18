#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
parse_qrcode.launch.py


English Description:
    Launch file for ROS2 Humble QR code parser node.
    It starts parse_qrcode_node and subscribes to /camera/image_raw by default.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# QR parser executable name.
PARSE_QRCODE_EXECUTABLE_NAME = "parse_qrcode_node"

# QR parser runtime node name.
PARSE_QRCODE_NODE_NAME = "parse_qrcode_node"

# Default Chinese font path (package share directory).
DEFAULT_FONT_PATH = os.path.join(
    get_package_share_directory(VISUAL_PACKAGE_NAME),
    "config",
    "Block_Simplified.TTF",
)


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description.

    Returns:
        ROS2 launch description object.
    """
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="输入原始图像话题 / Input compressed image topic.",
        ),
        DeclareLaunchArgument(
            "use_compressed_image",
            default_value="true",
            description="是否使用 sensor_msgs/msg/CompressedImage / Whether to use CompressedImage input.",
        ),
        DeclareLaunchArgument(
            "font_path",
            default_value=DEFAULT_FONT_PATH,
            description="PIL 绘制二维码内容使用的字体路径 / Font path used by PIL for QR text drawing.",
        ),
        DeclareLaunchArgument(
            "camera_index",
            default_value="0",
            description="保留参数：摄像头索引 / Reserved parameter: camera index.",
        ),
        DeclareLaunchArgument(
            "frame_width",
            default_value="640",
            description="处理图像宽度 / Processing frame width.",
        ),
        DeclareLaunchArgument(
            "frame_height",
            default_value="480",
            description="处理图像高度 / Processing frame height.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="parse_qrcode",
            description="OpenCV 显示窗口名称 / OpenCV display window name.",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true",
            description="是否显示 OpenCV 窗口 / Whether to show OpenCV window.",
        ),
        DeclareLaunchArgument(
            "window_width",
            default_value="960",
            description="OpenCV 窗口宽度 / OpenCV window width.",
        ),
        DeclareLaunchArgument(
            "window_height",
            default_value="720",
            description="OpenCV 窗口高度 / OpenCV window height.",
        ),
        DeclareLaunchArgument(
            "font_size",
            default_value="18",
            description="二维码内容文字大小 / QR payload text font size.",
        ),
        DeclareLaunchArgument(
            "fps_font_scale",
            default_value="0.6",
            description="FPS 文字大小 / FPS text font scale.",
        ),
        DeclareLaunchArgument(
            "enable_horizontal_flip",
            default_value="false",
            description="是否水平镜像图像 / Whether to horizontally flip image.",
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
            "print_decode_log",
            default_value="true",
            description="是否打印二维码识别日志 / Whether to print QR decode log.",
        ),
        DeclareLaunchArgument(
            "decode_log_interval_sec",
            default_value="0.5",
            description="二维码识别日志打印间隔 / QR decode log interval.",
        ),
        DeclareLaunchArgument(
            "enable_rqt_dynamic_parameters",
            default_value="true",
            description="是否允许 rqt/ros2 param 动态调参 / Whether to enable runtime parameter tuning.",
        ),
    ]

    parse_qrcode_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=PARSE_QRCODE_EXECUTABLE_NAME,
        name=PARSE_QRCODE_NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "use_compressed_image": ParameterValue(LaunchConfiguration("use_compressed_image"), value_type=bool),
            "font_path": LaunchConfiguration("font_path"),
            "camera_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
            "frame_width": ParameterValue(LaunchConfiguration("frame_width"), value_type=int),
            "frame_height": ParameterValue(LaunchConfiguration("frame_height"), value_type=int),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            "font_size": ParameterValue(LaunchConfiguration("font_size"), value_type=int),
            "fps_font_scale": ParameterValue(LaunchConfiguration("fps_font_scale"), value_type=float),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "print_decode_log": ParameterValue(LaunchConfiguration("print_decode_log"), value_type=bool),
            "decode_log_interval_sec": ParameterValue(LaunchConfiguration("decode_log_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
    )

    return LaunchDescription(launch_arguments + [parse_qrcode_node])
