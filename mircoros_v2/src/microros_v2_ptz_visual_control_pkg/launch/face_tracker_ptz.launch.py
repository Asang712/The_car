#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
face_tracker_ptz.launch.py

中文说明:
    ROS2 Humble 人脸识别云台控制节点启动文件。
    本启动文件将 Haar 人脸分类器默认指向当前功能包 config/haarcascade_frontalface_default.xml，
    并按物体跟踪节点的低帧率稳定控制配置提供目标中心滤波、云台角度滤波和固定频率控制参数。

English Description:
    Launch file for ROS2 Humble face tracking PTZ control node.
    This launch file points the Haar face classifier to config/haarcascade_frontalface_default.xml
    in the current package by default, and exposes target smoothing, PTZ smoothing, and fixed-rate
    control parameters aligned with the object tracking node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# VISUAL_CONTROL_PACKAGE_NAME: 视觉控制功能包名称 / Visual control package name.
VISUAL_CONTROL_PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"

# FACE_TRACKER_EXECUTABLE_NAME: 人脸跟踪节点可执行文件名称 / Face tracker executable name.
FACE_TRACKER_EXECUTABLE_NAME = "face_tracker_ptz_node"

# FACE_TRACKER_NODE_NAME: 人脸跟踪节点运行名称 / Face tracker runtime node name.
FACE_TRACKER_NODE_NAME = "face_tracker_ptz_node"

# DEFAULT_FACE_CASCADE_FILE_NAME: 默认人脸分类器文件名 / Default face classifier filename.
DEFAULT_FACE_CASCADE_FILE_NAME = "haarcascade_frontalface_default.xml"


def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # visual_control_package_share_directory: 视觉控制功能包 share 目录 / Visual control package share directory.
    visual_control_package_share_directory = get_package_share_directory(VISUAL_CONTROL_PACKAGE_NAME)

    # default_face_cascade_file: 默认人脸级联分类器路径 / Default face cascade classifier path.
    default_face_cascade_file = os.path.join(
        visual_control_package_share_directory,
        "config",
        DEFAULT_FACE_CASCADE_FILE_NAME,
    )

    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "ptz_command_topic",
            default_value="/cmd_ptz_angle",
            description="云台角度输出话题 / PTZ angle output topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="face_tracker_ptz",
            description="OpenCV 调试窗口名称 / OpenCV debug window name.",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true",
            description="是否显示 OpenCV 调试窗口 / Whether to show OpenCV debug window.",
        ),
        DeclareLaunchArgument(
            "window_width",
            default_value="960",
            description="OpenCV 调试窗口宽度 / OpenCV debug window width.",
        ),
        DeclareLaunchArgument(
            "window_height",
            default_value="720",
            description="OpenCV 调试窗口高度 / OpenCV debug window height.",
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
            description="人脸检测图像缩放比例 / Face detection image scale.",
        ),
        DeclareLaunchArgument(
            "face_select_strategy",
            default_value="largest",
            description="多人脸选择策略: largest/nearest_center / Multi-face selection strategy.",
        ),
        DeclareLaunchArgument(
            "enable_gray_equalize",
            default_value="true",
            description="是否对灰度图做直方图均衡 / Whether to equalize grayscale image.",
        ),
        DeclareLaunchArgument(
            "horizontal_min_deg",
            default_value="-90.0",
            description="s1 水平舵机最小角度 / Minimum s1 horizontal servo angle.",
        ),
        DeclareLaunchArgument(
            "horizontal_max_deg",
            default_value="90.0",
            description="s1 水平舵机最大角度 / Maximum s1 horizontal servo angle.",
        ),
        DeclareLaunchArgument(
            "pitch_min_deg",
            default_value="-90.0",
            description="s2 俯仰舵机最小角度 / Minimum s2 pitch servo angle.",
        ),
        DeclareLaunchArgument(
            "pitch_max_deg",
            default_value="20.0",
            description="s2 俯仰舵机最大角度 / Maximum s2 pitch servo angle.",
        ),
        DeclareLaunchArgument(
            "initial_horizontal_deg",
            default_value="0.0",
            description="s1 初始水平角度 / Initial s1 horizontal angle.",
        ),
        DeclareLaunchArgument(
            "initial_pitch_deg",
            default_value="-45.0",
            description="s2 初始俯仰角度 / Initial s2 pitch angle.",
        ),
        DeclareLaunchArgument(
            "reverse_horizontal_direction",
            default_value="false",
            description="是否反转 s1 水平方向 / Whether to reverse s1 horizontal direction.",
        ),
        DeclareLaunchArgument(
            "reverse_pitch_direction",
            default_value="true",
            description="是否反转 s2 俯仰方向 / Whether to reverse s2 pitch direction.",
        ),
        DeclareLaunchArgument(
            "deadband_x",
            default_value="20.0",
            description="水平死区像素 / Horizontal deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "deadband_y",
            default_value="20.0",
            description="俯仰死区像素 / Pitch deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "pd_kp",
            default_value="0.7",
            description="PD 比例增益 / PD proportional gain.",
        ),
        DeclareLaunchArgument(
            "pd_kd",
            default_value="0.25",
            description="PD 微分增益 / PD derivative gain.",
        ),
        DeclareLaunchArgument(
            "ptz_step_smooth_alpha",
            default_value="0.35",
            description="云台步进输出平滑系数 / PTZ step output smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "max_ptz_step",
            default_value="1.2",
            description="单次最大云台角度步进 / Maximum PTZ angle step per control tick.",
        ),
        DeclareLaunchArgument(
            "target_smooth_alpha",
            default_value="0.55",
            description="人脸中心低通滤波系数 / Face center low-pass smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "ptz_smooth_alpha",
            default_value="0.75",
            description="云台角度低通滤波系数 / PTZ angle low-pass smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="固定频率云台控制发布频率 / Fixed PTZ command publish rate.",
        ),
        DeclareLaunchArgument(
            "image_processing_rate_hz",
            default_value="30.0",
            description="图像处理定时器频率 / Image processing timer rate.",
        ),
        DeclareLaunchArgument(
            "target_lost_timeout_sec",
            default_value="1.0",
            description="人脸目标丢失超时时间 / Face target lost timeout.",
        ),
        DeclareLaunchArgument(
            "lost_target_stop",
            default_value="true",
            description="目标超时丢失后是否清空目标并停止控制 / Whether to stop when target is lost.",
        ),
        DeclareLaunchArgument(
            "auto_start_tracking",
            default_value="true",
            description="检测到人脸后是否自动开始跟踪 / Whether to start tracking automatically.",
        ),
        DeclareLaunchArgument(
            "auto_reacquire_face",
            default_value="true",
            description="人脸丢失后重新出现时是否自动恢复跟踪 / Whether to auto reacquire lost face.",
        ),
        DeclareLaunchArgument(
            "publish_initial_ptz_command",
            default_value="true",
            description="启动时是否发布初始云台角度 / Whether to publish initial PTZ angle at startup.",
        ),
        DeclareLaunchArgument(
            "initial_publish_count",
            default_value="20",
            description="启动时初始云台角度重复发布次数 / Initial PTZ publish repeat count.",
        ),
        DeclareLaunchArgument(
            "initial_publish_interval_sec",
            default_value="0.2",
            description="启动时初始云台角度重复发布间隔 / Initial PTZ publish interval.",
        ),
        DeclareLaunchArgument(
            "debug_log_interval_sec",
            default_value="1.0",
            description="调试日志间隔，0表示关闭 / Debug log interval, 0 means disabled.",
        ),
        DeclareLaunchArgument(
            "print_ptz_angle",
            default_value="false",
            description="是否周期打印云台角度 / Whether to print PTZ angle periodically.",
        ),
        DeclareLaunchArgument(
            "print_interval_sec",
            default_value="0.5",
            description="云台角度打印间隔 / PTZ angle print interval.",
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

    # face_tracker_ptz_node: 人脸识别云台控制节点 / Face tracking PTZ control node.
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

    face_tracker_ptz_node = Node(
        package=VISUAL_CONTROL_PACKAGE_NAME,
        executable=FACE_TRACKER_EXECUTABLE_NAME,
        name=FACE_TRACKER_NODE_NAME,
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
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
                "face_select_strategy": LaunchConfiguration("face_select_strategy"),
                "enable_gray_equalize": ParameterValue(LaunchConfiguration("enable_gray_equalize"), value_type=bool),
                "horizontal_min_deg": ParameterValue(LaunchConfiguration("horizontal_min_deg"), value_type=float),
                "horizontal_max_deg": ParameterValue(LaunchConfiguration("horizontal_max_deg"), value_type=float),
                "pitch_min_deg": ParameterValue(LaunchConfiguration("pitch_min_deg"), value_type=float),
                "pitch_max_deg": ParameterValue(LaunchConfiguration("pitch_max_deg"), value_type=float),
                "initial_horizontal_deg": ParameterValue(LaunchConfiguration("initial_horizontal_deg"), value_type=float),
                "initial_pitch_deg": ParameterValue(LaunchConfiguration("initial_pitch_deg"), value_type=float),
                "reverse_horizontal_direction": ParameterValue(LaunchConfiguration("reverse_horizontal_direction"), value_type=bool),
                "reverse_pitch_direction": ParameterValue(LaunchConfiguration("reverse_pitch_direction"), value_type=bool),
                "deadband_x": ParameterValue(LaunchConfiguration("deadband_x"), value_type=float),
                "deadband_y": ParameterValue(LaunchConfiguration("deadband_y"), value_type=float),
                "pd_kp": ParameterValue(LaunchConfiguration("pd_kp"), value_type=float),
                "pd_kd": ParameterValue(LaunchConfiguration("pd_kd"), value_type=float),
                "ptz_step_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_step_smooth_alpha"), value_type=float),
                "max_ptz_step": ParameterValue(LaunchConfiguration("max_ptz_step"), value_type=float),
                "target_smooth_alpha": ParameterValue(LaunchConfiguration("target_smooth_alpha"), value_type=float),
                "ptz_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_smooth_alpha"), value_type=float),
                "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
                "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
                "target_lost_timeout_sec": ParameterValue(LaunchConfiguration("target_lost_timeout_sec"), value_type=float),
                "lost_target_stop": ParameterValue(LaunchConfiguration("lost_target_stop"), value_type=bool),
                "auto_start_tracking": ParameterValue(LaunchConfiguration("auto_start_tracking"), value_type=bool),
                "auto_reacquire_face": ParameterValue(LaunchConfiguration("auto_reacquire_face"), value_type=bool),
                "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
                "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
                "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
                "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
                "print_ptz_angle": ParameterValue(LaunchConfiguration("print_ptz_angle"), value_type=bool),
                "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
                "use_header_stamp_for_fps": ParameterValue(LaunchConfiguration("use_header_stamp_for_fps"), value_type=bool),
                "show_image_stamp_delay": ParameterValue(LaunchConfiguration("show_image_stamp_delay"), value_type=bool),
                "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [face_tracker_ptz_node])
