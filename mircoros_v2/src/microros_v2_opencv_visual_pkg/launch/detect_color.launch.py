#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_color.launch.py


English Description:
    Launch file for a ROS2 Humble color detection node.
    This version removes PTZ control, PID, PTZ command publishing, and base cmd_vel control.
    It keeps only HSV color detection, ROI color sampling, visualization, and detection logs.
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

# 'detect_color_node = microros_v2_opencv_visual_pkg.detect_color:main'
COLOR_DETECTION_EXECUTABLE_NAME = "detect_color_node"

# Runtime node name.
COLOR_DETECTION_NODE_NAME = "detect_color_node"

# Default HSV threshold filename.
DEFAULT_HSV_FILE_NAME = "color_hsv.txt"


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description.

    Returns:
        ROS2 launch description object.
    """
    # Package share directory.
    package_share_directory = get_package_share_directory(VISUAL_PACKAGE_NAME)

    # Default HSV threshold file path.
    default_hsv_file = os.path.join(
        package_share_directory,
        "config",
        DEFAULT_HSV_FILE_NAME,
    )

    # Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="输入图像话题 / Input image topic.",
        ),
        DeclareLaunchArgument(
            "use_compressed_image",
            default_value="false",
            description="是否使用 sensor_msgs/msg/CompressedImage 输入 / Whether to use CompressedImage input.",
        ),
        DeclareLaunchArgument(
            "hsv_file",
            default_value=default_hsv_file,
            description="HSV 阈值文件路径 / HSV threshold file path.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="detect_color_node",
            description="OpenCV 显示窗口名称 / OpenCV display window name.",
        ),
        DeclareLaunchArgument(
            "display_debug_window",
            default_value="true",
            description="是否显示 OpenCV 窗口 / Whether to show OpenCV window.",
        ),
        DeclareLaunchArgument(
            "publish_debug_image",
            default_value="false",
            description="是否发布调试图像 / Whether to publish debug image.",
        ),
        DeclareLaunchArgument(
            "debug_image_topic",
            default_value="/detect_color/debug_image",
            description="调试图像输出话题 / Debug image output topic.",
        ),
        DeclareLaunchArgument(
            "show_binary_window",
            default_value="false",
            description="是否并排显示二值掩码 / Whether to show binary mask side-by-side.",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="320",
            description="处理图像宽度，0 表示不缩放 / Processing image width, 0 means no resize.",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="240",
            description="处理图像高度，0 表示不缩放 / Processing image height, 0 means no resize.",
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
            "Hmin",
            default_value="0",
            description="HSV H 下限 / HSV H lower threshold.",
        ),
        DeclareLaunchArgument(
            "Smin",
            default_value="85",
            description="HSV S 下限 / HSV S lower threshold.",
        ),
        DeclareLaunchArgument(
            "Vmin",
            default_value="126",
            description="HSV V 下限 / HSV V lower threshold.",
        ),
        DeclareLaunchArgument(
            "Hmax",
            default_value="9",
            description="HSV H 上限 / HSV H upper threshold.",
        ),
        DeclareLaunchArgument(
            "Smax",
            default_value="253",
            description="HSV S 上限 / HSV S upper threshold.",
        ),
        DeclareLaunchArgument(
            "Vmax",
            default_value="253",
            description="HSV V 上限 / HSV V upper threshold.",
        ),
        DeclareLaunchArgument(
            "h_expand",
            default_value="5",
            description="ROI 取色时 H 扩展量 / H expansion when sampling ROI.",
        ),
        DeclareLaunchArgument(
            "s_expand",
            default_value="20",
            description="ROI 取色时 S 扩展量 / S expansion when sampling ROI.",
        ),
        DeclareLaunchArgument(
            "v_expand",
            default_value="20",
            description="ROI 取色时 V 扩展量 / V expansion when sampling ROI.",
        ),
        DeclareLaunchArgument(
            "min_target_radius",
            default_value="5.0",
            description="最小目标半径像素 / Minimum target radius in pixels.",
        ),
        DeclareLaunchArgument(
            "min_target_area",
            default_value="60.0",
            description="最小目标轮廓面积 / Minimum target contour area.",
        ),
        DeclareLaunchArgument(
            "morph_kernel_size",
            default_value="5",
            description="形态学滤波核尺寸 / Morphology kernel size.",
        ),
        DeclareLaunchArgument(
            "print_detection_log",
            default_value="true",
            description="是否周期打印识别到的颜色目标数量 / Whether to print detected color target count periodically.",
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
        DeclareLaunchArgument(
            "enable_rqt_dynamic_parameters",
            default_value="true",
            description="是否允许 rqt/ros2 param 运行时动态调参 / Enable rqt runtime dynamic tuning.",
        ),
    ]

    # Color detection node.
    detect_color_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=COLOR_DETECTION_EXECUTABLE_NAME,
        name=COLOR_DETECTION_NODE_NAME,
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "use_compressed_image": ParameterValue(LaunchConfiguration("use_compressed_image"), value_type=bool),
                "hsv_file": LaunchConfiguration("hsv_file"),
                "window_name": LaunchConfiguration("window_name"),
                "display_debug_window": ParameterValue(LaunchConfiguration("display_debug_window"), value_type=bool),
                "publish_debug_image": ParameterValue(LaunchConfiguration("publish_debug_image"), value_type=bool),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "show_binary_window": ParameterValue(LaunchConfiguration("show_binary_window"), value_type=bool),
                "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
                "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
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
                "Hmin": ParameterValue(LaunchConfiguration("Hmin"), value_type=int),
                "Smin": ParameterValue(LaunchConfiguration("Smin"), value_type=int),
                "Vmin": ParameterValue(LaunchConfiguration("Vmin"), value_type=int),
                "Hmax": ParameterValue(LaunchConfiguration("Hmax"), value_type=int),
                "Smax": ParameterValue(LaunchConfiguration("Smax"), value_type=int),
                "Vmax": ParameterValue(LaunchConfiguration("Vmax"), value_type=int),
                "h_expand": ParameterValue(LaunchConfiguration("h_expand"), value_type=int),
                "s_expand": ParameterValue(LaunchConfiguration("s_expand"), value_type=int),
                "v_expand": ParameterValue(LaunchConfiguration("v_expand"), value_type=int),
                "min_target_radius": ParameterValue(LaunchConfiguration("min_target_radius"), value_type=float),
                "min_target_area": ParameterValue(LaunchConfiguration("min_target_area"), value_type=float),
                "morph_kernel_size": ParameterValue(LaunchConfiguration("morph_kernel_size"), value_type=int),
                "print_detection_log": ParameterValue(LaunchConfiguration("print_detection_log"), value_type=bool),
                "detection_log_interval_sec": ParameterValue(LaunchConfiguration("detection_log_interval_sec"), value_type=float),
                "use_header_stamp_for_fps": ParameterValue(LaunchConfiguration("use_header_stamp_for_fps"), value_type=bool),
                "show_image_stamp_delay": ParameterValue(LaunchConfiguration("show_image_stamp_delay"), value_type=bool),
                "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(launch_arguments + [detect_color_node])

