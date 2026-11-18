#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
object_tracker_ptz.launch.py

中文说明:
    ROS2 Humble 物体追踪云台控制节点启动文件。
    按 ptz-tracking-rules SKILL 优化，使用 PD 控制替代 PID。

English Description:
    Launch file for ROS2 Humble object tracking PTZ control node.
    Optimized per ptz-tracking-rules SKILL, using PD control instead of PID.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# VISUAL_CONTROL_PACKAGE_NAME: 视觉控制功能包名称 / Visual control package name.
VISUAL_CONTROL_PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"

# OBJECT_TRACKER_EXECUTABLE_NAME: 物体追踪节点可执行文件名称 / Object tracker executable name.
OBJECT_TRACKER_EXECUTABLE_NAME = "object_tracker_ptz_node"

# OBJECT_TRACKER_NODE_NAME: 物体追踪节点运行名称 / Object tracker node runtime name.
OBJECT_TRACKER_NODE_NAME = "object_tracker_ptz_node"


def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="960",
            description="处理图像宽度 / Processing image width.",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="720",
            description="处理图像高度 / Processing image height.",
        ),
        DeclareLaunchArgument(
            "ptz_command_topic",
            default_value="/cmd_ptz_angle",
            description="云台角度输出话题 / PTZ angle output topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="object_tracker_ptz",
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
            default_value="0.45",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="22",
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
            default_value="true",
            description="是否显示文字半透明背景 / Whether to show overlay transparent background.",
        ),
        DeclareLaunchArgument(
            "show_binary_window",
            default_value="false",
            description="是否显示原图和二值图并排窗口 / Whether to show image and binary mask side by side.",
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
        # PD 控制参数 / PD control parameters.
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
            default_value="10.0",
            description="水平方向死区像素 / Horizontal deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "deadband_y",
            default_value="10.0",
            description="俯仰方向死区像素 / Pitch deadband in pixels.",
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
            description="云台步进平滑系数 / PTZ step smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "max_ptz_step",
            default_value="1.2",
            description="单次最大云台角度步进 / Maximum PTZ step per control update.",
        ),
        # 滤波与控制频率 / Filtering and control rate.
        DeclareLaunchArgument(
            "target_smooth_alpha",
            default_value="0.55",
            description="目标中心低通滤波系数 / Target center low-pass smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "ptz_smooth_alpha",
            default_value="0.85",
            description="云台角度低通滤波系数 / PTZ angle smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="固定频率云台控制发布频率 / Fixed-rate PTZ command publish rate.",
        ),
        DeclareLaunchArgument(
            "image_processing_rate_hz",
            default_value="30.0",
            description="图像处理定时频率 / Image processing timer rate.",
        ),
        # 追踪参数 / Tracking parameters.
        DeclareLaunchArgument(
            "tracking_min_area",
            default_value="100.0",
            description="有效目标最小面积 / Minimum valid target area.",
        ),
        DeclareLaunchArgument(
            "min_target_radius",
            default_value="5.0",
            description="有效目标最小半径（像素） / Minimum valid target radius in pixels.",
        ),
        DeclareLaunchArgument(
            "hsv_file",
            default_value="",
            description="HSV 阈值持久化文件路径 / HSV threshold file path for persistence.",
        ),
        DeclareLaunchArgument(
            "tracking_backend",
            default_value="opencv",
            description="追踪后端: camshift/opencv/color_hsv / Tracking backend: camshift/opencv/color_hsv.",
        ),
        DeclareLaunchArgument(
            "opencv_tracker_type",
            default_value="CSRT",
            description="OpenCV追踪器类型: CSRT/KCF/MIL/BOOSTING/MOSSE / OpenCV tracker type.",
        ),
        DeclareLaunchArgument(
            "hsv_saturation_min",
            default_value="30",
            description="CamShift HSV 最小饱和度 / CamShift minimum HSV saturation.",
        ),
        DeclareLaunchArgument(
            "hsv_value_min",
            default_value="30",
            description="CamShift HSV 最小亮度 / CamShift minimum HSV value.",
        ),
        DeclareLaunchArgument(
            "hsv_value_max",
            default_value="255",
            description="CamShift HSV 最大亮度 / CamShift maximum HSV value.",
        ),
        DeclareLaunchArgument(
            "Hmin",
            default_value="0",
            description="color_hsv HSV 色相下限 / color_hsv HSV hue minimum.",
        ),
        DeclareLaunchArgument(
            "Smin",
            default_value="85",
            description="color_hsv HSV 饱和度下限 / color_hsv HSV saturation minimum.",
        ),
        DeclareLaunchArgument(
            "Vmin",
            default_value="126",
            description="color_hsv HSV 明度下限 / color_hsv HSV value minimum.",
        ),
        DeclareLaunchArgument(
            "Hmax",
            default_value="9",
            description="color_hsv HSV 色相上限 / color_hsv HSV hue maximum.",
        ),
        DeclareLaunchArgument(
            "Smax",
            default_value="253",
            description="color_hsv HSV 饱和度上限 / color_hsv HSV saturation maximum.",
        ),
        DeclareLaunchArgument(
            "Vmax",
            default_value="253",
            description="color_hsv HSV 明度上限 / color_hsv HSV value maximum.",
        ),
        DeclareLaunchArgument(
            "h_expand",
            default_value="5",
            description="ROI 采样 HSV H 扩展量 / HSV hue expansion after ROI sampling.",
        ),
        DeclareLaunchArgument(
            "s_expand",
            default_value="20",
            description="ROI 采样 HSV S 扩展量 / HSV saturation expansion after ROI sampling.",
        ),
        DeclareLaunchArgument(
            "v_expand",
            default_value="20",
            description="ROI 采样 HSV V 扩展量 / HSV value expansion after ROI sampling.",
        ),
        DeclareLaunchArgument(
            "morph_kernel_size",
            default_value="5",
            description="color_hsv 形态学核尺寸 / color_hsv morphology kernel size.",
        ),
        DeclareLaunchArgument(
            "target_lost_timeout_sec",
            default_value="1.0",
            description="目标丢失超时时间 / Target lost timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "lost_target_stop",
            default_value="true",
            description="目标丢失后是否停止控制输出更新 / Stop control update after target is lost.",
        ),
        DeclareLaunchArgument(
            "publish_initial_ptz_command",
            default_value="true",
            description="启动时是否发布初始云台角度 / Whether to publish initial PTZ angle at startup.",
        ),
        DeclareLaunchArgument(
            "initial_publish_count",
            default_value="20",
            description="启动时初始云台角度重复发布次数 / Initial PTZ publish repeat count at startup.",
        ),
        DeclareLaunchArgument(
            "initial_publish_interval_sec",
            default_value="0.2",
            description="启动时初始云台角度重复发布间隔 / Initial PTZ publish interval at startup.",
        ),
        DeclareLaunchArgument(
            "debug_log_interval_sec",
            default_value="1.0",
            description="调试日志间隔，0表示关闭 / Debug log interval, 0 means disabled.",
        ),
        DeclareLaunchArgument(
            "print_ptz_angle",
            default_value="false",
            description="是否周期打印云台角度调试信息 / Whether to print PTZ angle debug info periodically.",
        ),
        DeclareLaunchArgument(
            "print_interval_sec",
            default_value="0.5",
            description="云台角度调试信息打印间隔 / PTZ angle debug print interval.",
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

    # object_tracker_ptz_node: 物体追踪云台控制节点 / Object tracking PTZ control node.
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

    object_tracker_ptz_node = Node(
        package=VISUAL_CONTROL_PACKAGE_NAME,
        executable=OBJECT_TRACKER_EXECUTABLE_NAME,
        name=OBJECT_TRACKER_NODE_NAME,
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
                "show_binary_window": ParameterValue(LaunchConfiguration("show_binary_window"), value_type=bool),
                "img_flip": ParameterValue(LaunchConfiguration("img_flip"), value_type=bool),
                "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
                "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
                "rotate_image_code": LaunchConfiguration("rotate_image_code"),
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
                "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
                "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
                "tracking_min_area": ParameterValue(LaunchConfiguration("tracking_min_area"), value_type=float),
                "min_target_radius": ParameterValue(LaunchConfiguration("min_target_radius"), value_type=float),
                "hsv_file": LaunchConfiguration("hsv_file"),
                "tracking_backend": LaunchConfiguration("tracking_backend"),
                "opencv_tracker_type": LaunchConfiguration("opencv_tracker_type"),
                "hsv_saturation_min": ParameterValue(LaunchConfiguration("hsv_saturation_min"), value_type=int),
                "hsv_value_min": ParameterValue(LaunchConfiguration("hsv_value_min"), value_type=int),
                "hsv_value_max": ParameterValue(LaunchConfiguration("hsv_value_max"), value_type=int),
                "Hmin": ParameterValue(LaunchConfiguration("Hmin"), value_type=int),
                "Smin": ParameterValue(LaunchConfiguration("Smin"), value_type=int),
                "Vmin": ParameterValue(LaunchConfiguration("Vmin"), value_type=int),
                "Hmax": ParameterValue(LaunchConfiguration("Hmax"), value_type=int),
                "Smax": ParameterValue(LaunchConfiguration("Smax"), value_type=int),
                "Vmax": ParameterValue(LaunchConfiguration("Vmax"), value_type=int),
                "h_expand": ParameterValue(LaunchConfiguration("h_expand"), value_type=int),
                "s_expand": ParameterValue(LaunchConfiguration("s_expand"), value_type=int),
                "v_expand": ParameterValue(LaunchConfiguration("v_expand"), value_type=int),
                "morph_kernel_size": ParameterValue(LaunchConfiguration("morph_kernel_size"), value_type=int),
                "target_lost_timeout_sec": ParameterValue(LaunchConfiguration("target_lost_timeout_sec"), value_type=float),
                "lost_target_stop": ParameterValue(LaunchConfiguration("lost_target_stop"), value_type=bool),
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

    return LaunchDescription(launch_arguments + [object_tracker_ptz_node])
