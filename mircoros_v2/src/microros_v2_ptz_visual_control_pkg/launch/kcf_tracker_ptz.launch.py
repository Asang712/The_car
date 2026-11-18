#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch file for kcf_tracker_ptz_node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG, EXE, NAME = "microros_v2_ptz_visual_control_pkg", "kcf_tracker_ptz_node", "kcf_tracker_ptz_node"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("window_name", default_value="kcf_tracker_ptz"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("image_width", default_value="960"),
        DeclareLaunchArgument("image_height", default_value="720"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.9"),
        DeclareLaunchArgument("overlay_line_height", default_value="20"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("horizontal_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("horizontal_max_deg", default_value="90.0"),
        DeclareLaunchArgument("pitch_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("pitch_max_deg", default_value="20.0"),
        DeclareLaunchArgument("initial_horizontal_deg", default_value="0.0"),
        DeclareLaunchArgument("initial_pitch_deg", default_value="-45.0"),
        DeclareLaunchArgument("reverse_horizontal_direction", default_value="false"),
        DeclareLaunchArgument("reverse_pitch_direction", default_value="true"),
        # PD 控制参数（SKILL）
        DeclareLaunchArgument("pd_kp", default_value="0.7"),
        DeclareLaunchArgument("pd_kd", default_value="0.25"),
        DeclareLaunchArgument("deadband_x", default_value="20.0"),
        DeclareLaunchArgument("deadband_y", default_value="20.0"),
        DeclareLaunchArgument("ptz_step_smooth_alpha", default_value="0.35"),
        DeclareLaunchArgument("max_ptz_step", default_value="1.2"),
        DeclareLaunchArgument("ptz_smooth_alpha", default_value="0.85"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true"),
        DeclareLaunchArgument("initial_publish_count", default_value="20"),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2"),
        DeclareLaunchArgument("tracker_type", default_value="KCF"),
        DeclareLaunchArgument("img_flip", default_value="false"),
        DeclareLaunchArgument("print_ptz_angle", default_value="false"),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
        # 自动 ROI 初始化参数（用于 Dify 视觉检测后自动启动追踪）
        DeclareLaunchArgument("auto_init_roi_x1", default_value="-1"),
        DeclareLaunchArgument("auto_init_roi_y1", default_value="-1"),
        DeclareLaunchArgument("auto_init_roi_x2", default_value="-1"),
        DeclareLaunchArgument("auto_init_roi_y2", default_value="-1"),
    ]

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

    node = Node(package=PKG, executable=EXE, name=NAME, output="screen", parameters=[{
        "image_topic": LaunchConfiguration("image_topic"),
        "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
        "window_name": LaunchConfiguration("window_name"),
        "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
        "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
        "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
        "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
        "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
        "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
        "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
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
        "pd_kp": ParameterValue(LaunchConfiguration("pd_kp"), value_type=float),
        "pd_kd": ParameterValue(LaunchConfiguration("pd_kd"), value_type=float),
        "deadband_x": ParameterValue(LaunchConfiguration("deadband_x"), value_type=float),
        "deadband_y": ParameterValue(LaunchConfiguration("deadband_y"), value_type=float),
        "ptz_step_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_step_smooth_alpha"), value_type=float),
        "max_ptz_step": ParameterValue(LaunchConfiguration("max_ptz_step"), value_type=float),
        "ptz_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_smooth_alpha"), value_type=float),
        "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
        "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
        "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
        "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
        "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
        "tracker_type": LaunchConfiguration("tracker_type"),
        "img_flip": ParameterValue(LaunchConfiguration("img_flip"), value_type=bool),
        "print_ptz_angle": ParameterValue(LaunchConfiguration("print_ptz_angle"), value_type=bool),
        "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
        "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        "auto_init_roi_x1": ParameterValue(LaunchConfiguration("auto_init_roi_x1"), value_type=int),
        "auto_init_roi_y1": ParameterValue(LaunchConfiguration("auto_init_roi_y1"), value_type=int),
        "auto_init_roi_x2": ParameterValue(LaunchConfiguration("auto_init_roi_x2"), value_type=int),
        "auto_init_roi_y2": ParameterValue(LaunchConfiguration("auto_init_roi_y2"), value_type=int),
    }], additional_env={"PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath})
    return LaunchDescription(args + [node])
