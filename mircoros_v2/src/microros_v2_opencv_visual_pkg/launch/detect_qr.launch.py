#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_qr.launch.py

中文说明:
    ROS2 Humble 二维码识别/检测节点启动文件。
    该版本已经去掉底盘控制、云台 PTZ、蜂鸣器、手柄接管等功能，只保留二维码检测与画面显示。
    默认使用 pyzbar 后端，避免 OpenCV 未链接 QUIRC 时无法解码。
    文件名重命名为 detect_qr.launch.py。

English Description:
    Launch file for a ROS2 Humble QR code detection node.
    This version removes chassis control, PTZ control, buzzer, joystick override, and keeps only
    QR code detection and visualization. The launch file has been renamed to detect_qr.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# VISUAL_PACKAGE_NAME: 当前 OpenCV 视觉功能包名称 / Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# QR_DETECTION_EXECUTABLE_NAME: 节点可执行文件名称。
# setup.py 中的 console_scripts 建议配置为 detect_qr_node。
QR_DETECTION_EXECUTABLE_NAME = "detect_qr_node"

# QR_DETECTION_NODE_NAME: 节点运行名称 / Runtime node name.
QR_DETECTION_NODE_NAME = "detect_qr_node"


def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="detect_qr_node",
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
            "image_width",
            default_value="640",
            description="处理图像宽度，0 表示不缩放 / Processing image width, 0 means no resize.",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="480",
            description="处理图像高度，0 表示不缩放 / Processing image height, 0 means no resize.",
        ),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="0.32",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="14",
            description="画面叠加文字行高 / Overlay text line height.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_x",
            default_value="6",
            description="画面叠加文字左边距 / Overlay text left margin.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_y",
            default_value="16",
            description="画面叠加文字上边距 / Overlay text top margin.",
        ),
        DeclareLaunchArgument(
            "overlay_show_background",
            default_value="false",
            description="是否显示文字背景 / Whether to show overlay background.",
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
            "qr_detector_backend",
            default_value="pyzbar",
            description="二维码检测后端: pyzbar/opencv/auto，默认 pyzbar 以兼容未链接 QUIRC 的 OpenCV / QR detector backend: pyzbar/opencv/auto.",
        ),
        DeclareLaunchArgument(
            "qr_select_strategy",
            default_value="largest",
            description="多二维码选择策略: largest/nearest_center/first / QR selection strategy.",
        ),
        DeclareLaunchArgument(
            "print_detection_log",
            default_value="true",
            description="是否周期打印二维码识别结果 / Whether to print QR detection result periodically.",
        ),
        DeclareLaunchArgument(
            "detection_log_interval_sec",
            default_value="1.0",
            description="识别日志打印间隔 / Detection log interval.",
        ),
        DeclareLaunchArgument(
            "debug_log_interval_sec",
            default_value="2.0",
            description="调试日志打印间隔 / Debug log interval.",
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
            description="是否允许 rqt/ros2 param 动态调参 / Whether to enable rqt/ros2 param dynamic parameters.",
        ),
    ]

    detect_qr_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=QR_DETECTION_EXECUTABLE_NAME,
        name=QR_DETECTION_NODE_NAME,
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "window_name": LaunchConfiguration("window_name"),
                "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
                "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
                "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
                "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
                "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
                "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
                "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
                "overlay_margin_x": ParameterValue(LaunchConfiguration("overlay_margin_x"), value_type=int),
                "overlay_margin_y": ParameterValue(LaunchConfiguration("overlay_margin_y"), value_type=int),
                "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
                "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
                "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
                "rotate_image_code": LaunchConfiguration("rotate_image_code"),
                "qr_detector_backend": LaunchConfiguration("qr_detector_backend"),
                "qr_select_strategy": LaunchConfiguration("qr_select_strategy"),
                "print_detection_log": ParameterValue(LaunchConfiguration("print_detection_log"), value_type=bool),
                "detection_log_interval_sec": ParameterValue(LaunchConfiguration("detection_log_interval_sec"), value_type=float),
                "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
                "use_header_stamp_for_fps": ParameterValue(LaunchConfiguration("use_header_stamp_for_fps"), value_type=bool),
                "show_image_stamp_delay": ParameterValue(LaunchConfiguration("show_image_stamp_delay"), value_type=bool),
                "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(launch_arguments + [detect_qr_node])
