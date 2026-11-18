#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_face.launch.py


English Description:
    Launch file for a ROS2 Humble face detection node.
    This version removes PTZ control, PID, and PTZ command publishing, and keeps only face detection
    and visualization. The launch file has been renamed to detect_face.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Current visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

FACE_DETECTION_EXECUTABLE_NAME = "detect_face_node"

# Runtime node name.
FACE_DETECTION_NODE_NAME = "detect_face_node"

# Default face classifier filename.
DEFAULT_FACE_CASCADE_FILE_NAME = "haarcascade_frontalface_default.xml"


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description.

    Returns:
        ROS2 launch description object.
    """
    # Package share directory.
    package_share_directory = get_package_share_directory(VISUAL_PACKAGE_NAME)

    # Default face cascade classifier path.
    default_face_cascade_file = os.path.join(
        package_share_directory,
        "config",
        DEFAULT_FACE_CASCADE_FILE_NAME,
    )

    # Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="detect_face_node",
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
            "overlay_font_scale",
            default_value="0.45",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="18",
            description="画面叠加文字行高 / Overlay text line height.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_x",
            default_value="8",
            description="画面叠加文字左边距 / Overlay text left margin.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_y",
            default_value="18",
            description="画面叠加文字上边距 / Overlay text top margin.",
        ),
        DeclareLaunchArgument(
            "overlay_show_background",
            default_value="false",
            description="是否显示文字背景 / Whether to show overlay background.",
        ),
        DeclareLaunchArgument(
            "img_flip",
            default_value="false",
            description="旧版图像水平翻转开关 / Legacy horizontal image flip switch.",
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
            "face_cascade_file",
            default_value=default_face_cascade_file,
            description="Haar 人脸级联分类器路径 / Haar face cascade classifier file path.",
        ),
        DeclareLaunchArgument(
            "face_detection_scale_factor",
            default_value="1.1",
            description="Haar detectMultiScale 缩放因子 / Haar detectMultiScale scale factor.",
        ),
        DeclareLaunchArgument(
            "face_min_neighbors",
            default_value="5",
            description="Haar detectMultiScale 最小邻居数 / Haar detectMultiScale minimum neighbors.",
        ),
        DeclareLaunchArgument(
            "face_min_width",
            default_value="80",
            description="最小人脸宽度像素 / Minimum face width in pixels.",
        ),
        DeclareLaunchArgument(
            "face_min_height",
            default_value="80",
            description="最小人脸高度像素 / Minimum face height in pixels.",
        ),
        DeclareLaunchArgument(
            "face_detection_scale",
            default_value="0.75",
            description="人脸检测图像缩放比例，越小越快 / Face detection image scale.",
        ),
        DeclareLaunchArgument(
            "enable_gray_equalize",
            default_value="true",
            description="是否对灰度图做直方图均衡 / Whether to equalize grayscale image.",
        ),
        DeclareLaunchArgument(
            "print_detection_log",
            default_value="true",
            description="是否周期打印识别到的人脸数量 / Whether to print detected face count periodically.",
        ),
        DeclareLaunchArgument(
            "detection_log_interval_sec",
            default_value="1.0",
            description="识别日志打印间隔 / Detection log interval.",
        ),
        DeclareLaunchArgument(
            "use_header_stamp_for_fps",
            default_value="true",
            description="是否使用图像 header stamp 计算帧率 / Whether to use image header stamp for FPS.",
        ),
        DeclareLaunchArgument(
            "show_image_stamp_delay",
            default_value="true",
            description="是否显示图像时间戳延迟 / Whether to show image stamp delay.",
        ),
    ]

    # Face detection node.
    detect_face_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=FACE_DETECTION_EXECUTABLE_NAME,
        name=FACE_DETECTION_NODE_NAME,
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "window_name": LaunchConfiguration("window_name"),
                "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
                "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
                "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
                "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
                "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
                "overlay_margin_x": ParameterValue(LaunchConfiguration("overlay_margin_x"), value_type=int),
                "overlay_margin_y": ParameterValue(LaunchConfiguration("overlay_margin_y"), value_type=int),
                "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
                "img_flip": ParameterValue(LaunchConfiguration("img_flip"), value_type=bool),
                "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
                "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
                "rotate_image_code": LaunchConfiguration("rotate_image_code"),
                "face_cascade_file": LaunchConfiguration("face_cascade_file"),
                "face_detection_scale_factor": ParameterValue(LaunchConfiguration("face_detection_scale_factor"), value_type=float),
                "face_min_neighbors": ParameterValue(LaunchConfiguration("face_min_neighbors"), value_type=int),
                "face_min_width": ParameterValue(LaunchConfiguration("face_min_width"), value_type=int),
                "face_min_height": ParameterValue(LaunchConfiguration("face_min_height"), value_type=int),
                "face_detection_scale": ParameterValue(LaunchConfiguration("face_detection_scale"), value_type=float),
                "enable_gray_equalize": ParameterValue(LaunchConfiguration("enable_gray_equalize"), value_type=bool),
                "print_detection_log": ParameterValue(LaunchConfiguration("print_detection_log"), value_type=bool),
                "detection_log_interval_sec": ParameterValue(LaunchConfiguration("detection_log_interval_sec"), value_type=float),
                "use_header_stamp_for_fps": ParameterValue(LaunchConfiguration("use_header_stamp_for_fps"), value_type=bool),
                "show_image_stamp_delay": ParameterValue(LaunchConfiguration("show_image_stamp_delay"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(launch_arguments + [detect_face_node])
