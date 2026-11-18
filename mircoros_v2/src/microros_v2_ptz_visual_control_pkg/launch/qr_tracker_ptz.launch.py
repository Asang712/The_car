#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch QR code PTZ tracker."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"
EXECUTABLE_NAME = "qr_tracker_ptz_node"
NODE_NAME = "qr_tracker_ptz_node"


def bool_param(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def int_param(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def float_param(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description() -> LaunchDescription:
    launch_arguments = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("window_name", default_value="qr_control_ptz"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.45"),
        DeclareLaunchArgument("overlay_line_height", default_value="18"),
        DeclareLaunchArgument("overlay_margin_x", default_value="8"),
        DeclareLaunchArgument("overlay_margin_y", default_value="20"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("qr_detector_backend", default_value="pyzbar"),
        DeclareLaunchArgument("qr_select_strategy", default_value="largest"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("auto_start_control", default_value="true"),
        DeclareLaunchArgument("lost_timeout", default_value="0.5"),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true"),
        DeclareLaunchArgument("initial_horizontal_deg", default_value="0.0"),
        DeclareLaunchArgument("initial_pitch_deg", default_value="-45.0"),
        DeclareLaunchArgument("initial_publish_count", default_value="20"),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2"),
        DeclareLaunchArgument("ptz_x_min", default_value="-90.0"),
        DeclareLaunchArgument("ptz_x_max", default_value="90.0"),
        DeclareLaunchArgument("ptz_y_min", default_value="-90.0"),
        DeclareLaunchArgument("ptz_y_max", default_value="20.0"),
        DeclareLaunchArgument("ptz_x_reverse", default_value="true"),
        DeclareLaunchArgument("ptz_y_reverse", default_value="true"),
        DeclareLaunchArgument("deadband_x", default_value="18.0"),
        DeclareLaunchArgument("deadband_y", default_value="18.0"),
        DeclareLaunchArgument("pd_kp", default_value="0.7"),
        DeclareLaunchArgument("pd_kd", default_value="0.25"),
        DeclareLaunchArgument("ptz_step_smooth_alpha", default_value="0.35"),
        DeclareLaunchArgument("max_ptz_step", default_value="1.2"),
        DeclareLaunchArgument("debug_log_interval_sec", default_value="2.0"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
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

    node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE_NAME,
        name=NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": bool_param("show_debug_window"),
            "window_width": int_param("window_width"),
            "window_height": int_param("window_height"),
            "overlay_font_scale": float_param("overlay_font_scale"),
            "overlay_line_height": int_param("overlay_line_height"),
            "overlay_margin_x": int_param("overlay_margin_x"),
            "overlay_margin_y": int_param("overlay_margin_y"),
            "overlay_show_background": bool_param("overlay_show_background"),
            "enable_horizontal_flip": bool_param("enable_horizontal_flip"),
            "enable_vertical_flip": bool_param("enable_vertical_flip"),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "qr_detector_backend": LaunchConfiguration("qr_detector_backend"),
            "qr_select_strategy": LaunchConfiguration("qr_select_strategy"),
            "control_rate_hz": float_param("control_rate_hz"),
            "image_processing_rate_hz": float_param("image_processing_rate_hz"),
            "auto_start_control": bool_param("auto_start_control"),
            "lost_timeout": float_param("lost_timeout"),
            "publish_initial_ptz_command": bool_param("publish_initial_ptz_command"),
            "initial_horizontal_deg": float_param("initial_horizontal_deg"),
            "initial_pitch_deg": float_param("initial_pitch_deg"),
            "initial_publish_count": int_param("initial_publish_count"),
            "initial_publish_interval_sec": float_param("initial_publish_interval_sec"),
            "ptz_x_min": float_param("ptz_x_min"),
            "ptz_x_max": float_param("ptz_x_max"),
            "ptz_y_min": float_param("ptz_y_min"),
            "ptz_y_max": float_param("ptz_y_max"),
            "ptz_x_reverse": bool_param("ptz_x_reverse"),
            "ptz_y_reverse": bool_param("ptz_y_reverse"),
            "deadband_x": float_param("deadband_x"),
            "deadband_y": float_param("deadband_y"),
            "pd_kp": float_param("pd_kp"),
            "pd_kd": float_param("pd_kd"),
            "ptz_step_smooth_alpha": float_param("ptz_step_smooth_alpha"),
            "max_ptz_step": float_param("max_ptz_step"),
            "debug_log_interval_sec": float_param("debug_log_interval_sec"),
            "enable_rqt_dynamic_parameters": bool_param("enable_rqt_dynamic_parameters"),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [node])
