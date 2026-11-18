#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
create_qrcode.launch.py


English Description:
    Launch file for ROS2 Humble QR code creation node.
    It starts create_qrcode_node and saves QR images to the source package config directory by default.
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

# QR code creation executable name.
CREATE_QRCODE_EXECUTABLE_NAME = "create_qrcode_node"

# QR code creation runtime node name.
CREATE_QRCODE_NODE_NAME = "create_qrcode_node"

# Default QR code save directory (package share directory).
DEFAULT_QRCODE_SAVE_DIRECTORY = os.path.join(
    get_package_share_directory(VISUAL_PACKAGE_NAME),
    "config",
)


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description.

    Returns:
        ROS2 launch description object.
    """
    launch_arguments = [
        DeclareLaunchArgument(
            "file_path",
            default_value=DEFAULT_QRCODE_SAVE_DIRECTORY,
            description="二维码保存目录 / QR code output directory.",
        ),
        DeclareLaunchArgument(
            "logo_filename",
            default_value="yahboom_logo.png",
            description="Logo 图片文件名，位于 file_path 目录下 / Logo filename under file_path.",
        ),
        DeclareLaunchArgument(
            "output_filename",
            default_value="yahboom_logo_qr.jpg",
            description="输出二维码图片文件名 / Output QR code image filename.",
        ),
        DeclareLaunchArgument(
            "qr_text",
            default_value="666",
            description="二维码内容 / QR code payload text.",
        ),
        DeclareLaunchArgument(
            "fill_color",
            default_value="green",
            description="二维码前景颜色 / QR code foreground color.",
        ),
        DeclareLaunchArgument(
            "back_color",
            default_value="white",
            description="二维码背景颜色 / QR code background color.",
        ),
        DeclareLaunchArgument(
            "show_image",
            default_value="true",
            description="生成后是否显示图片 / Whether to show image after generation.",
        ),
        DeclareLaunchArgument(
            "qr_version",
            default_value="1",
            description="二维码版本 1-40 / QR code version 1-40.",
        ),
        DeclareLaunchArgument(
            "box_size",
            default_value="10",
            description="二维码每个小格像素大小 / QR code box size.",
        ),
        DeclareLaunchArgument(
            "border",
            default_value="4",
            description="二维码边框格数 / QR code border size.",
        ),
        DeclareLaunchArgument(
            "logo_scale_factor",
            default_value="6",
            description="Logo 缩放比例，数值越大 Logo 越小 / Logo scale factor; larger means smaller logo.",
        ),
    ]

    create_qrcode_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=CREATE_QRCODE_EXECUTABLE_NAME,
        name=CREATE_QRCODE_NODE_NAME,
        output="screen",
        parameters=[{
            "file_path": LaunchConfiguration("file_path"),
            "logo_filename": LaunchConfiguration("logo_filename"),
            "output_filename": LaunchConfiguration("output_filename"),
            "qr_text": LaunchConfiguration("qr_text"),
            "fill_color": LaunchConfiguration("fill_color"),
            "back_color": LaunchConfiguration("back_color"),
            "show_image": ParameterValue(LaunchConfiguration("show_image"), value_type=bool),
            "qr_version": ParameterValue(LaunchConfiguration("qr_version"), value_type=int),
            "box_size": ParameterValue(LaunchConfiguration("box_size"), value_type=int),
            "border": ParameterValue(LaunchConfiguration("border"), value_type=int),
            "logo_scale_factor": ParameterValue(LaunchConfiguration("logo_scale_factor"), value_type=int),
        }],
    )

    return LaunchDescription(launch_arguments + [create_qrcode_node])
