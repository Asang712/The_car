#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_line.launch.py

中文说明:
    ROS2 Humble 巡线底盘控制节点启动文件（No-PTZ 版）。
    摄像头 pitch 手动调整朝下固定，无云台初始化。所有巡线参数均可通过 launch 配置。

English Description:
    Launch file for ROS2 Humble line following control node (No-PTZ version).
    Camera pitch is manually adjusted downward, no PTZ initialization.
    All line following parameters are configurable via launch.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_no_ptz_visual_control_pkg"
EXECUTABLE_NAME = "follow_line_node"
NODE_NAME = "follow_line_node"


def generate_launch_description() -> LaunchDescription:
    launch_arguments = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("beep_topic", default_value="/beep"),
        DeclareLaunchArgument("joy_state_topic", default_value="/JoyState"),
        DeclareLaunchArgument("window_name", default_value="follow_line"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.38"),
        DeclareLaunchArgument("overlay_line_height", default_value="15"),
        DeclareLaunchArgument("overlay_margin_x", default_value="6"),
        DeclareLaunchArgument("overlay_margin_y", default_value="16"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("show_binary_window", default_value="true"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="true"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("Hmin", default_value="0"),
        DeclareLaunchArgument("Smin", default_value="85"),
        DeclareLaunchArgument("Vmin", default_value="126"),
        DeclareLaunchArgument("Hmax", default_value="9"),
        DeclareLaunchArgument("Smax", default_value="253"),
        DeclareLaunchArgument("Vmax", default_value="253"),
        DeclareLaunchArgument("h_expand", default_value="8"),
        DeclareLaunchArgument("s_expand", default_value="40"),
        DeclareLaunchArgument("v_expand", default_value="40"),
        DeclareLaunchArgument("morph_kernel_size", default_value="5"),
        DeclareLaunchArgument("hsv_config_file", default_value=""),
        DeclareLaunchArgument("save_hsv_to_file", default_value="false"),
        DeclareLaunchArgument("load_hsv_from_file", default_value="false"),
        DeclareLaunchArgument("line_roi_top_ratio", default_value="0.55"),
        DeclareLaunchArgument("line_roi_bottom_ratio", default_value="2.00"),
        DeclareLaunchArgument("line_min_area", default_value="150.0"),
        DeclareLaunchArgument("line_lost_timeout_sec", default_value="1.5"),
        DeclareLaunchArgument("target_smooth_alpha", default_value="0.55"),
        DeclareLaunchArgument("line_pid_kp", default_value="22.0"),
        DeclareLaunchArgument("line_pid_ki", default_value="0.0"),
        DeclareLaunchArgument("line_pid_kd", default_value="5.0"),
        DeclareLaunchArgument("pid_scale", default_value="1000.0"),
        DeclareLaunchArgument("pid_integral_limit", default_value="100.0"),
        DeclareLaunchArgument("line_error_divisor", default_value="16.0"),
        DeclareLaunchArgument("line_linear_speed", default_value="0.15"),
        DeclareLaunchArgument("line_center_deadband_pixel", default_value="40.0"),
        DeclareLaunchArgument("max_angular_speed", default_value="1.2"),
        DeclareLaunchArgument("reverse_angular_direction", default_value="true"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("enable_obstacle_stop", default_value="true"),
        DeclareLaunchArgument("obstacle_min_valid_distance", default_value="0.20"),
        DeclareLaunchArgument("obstacle_response_distance", default_value="0.35"),
        DeclareLaunchArgument("obstacle_check_angle_deg", default_value="60.0"),
        DeclareLaunchArgument("obstacle_warning_count_threshold", default_value="10"),
        DeclareLaunchArgument("enable_beep", default_value="false"),
        DeclareLaunchArgument("auto_start_tracking", default_value="false"),
        DeclareLaunchArgument("lost_line_stop", default_value="true"),
        DeclareLaunchArgument("debug_log_interval_sec", default_value="2.0"),
        DeclareLaunchArgument("print_cmd_vel", default_value="false"),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5"),
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
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "beep_topic": LaunchConfiguration("beep_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
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
            "morph_kernel_size": ParameterValue(LaunchConfiguration("morph_kernel_size"), value_type=int),
            "hsv_config_file": LaunchConfiguration("hsv_config_file"),
            "save_hsv_to_file": ParameterValue(LaunchConfiguration("save_hsv_to_file"), value_type=bool),
            "load_hsv_from_file": ParameterValue(LaunchConfiguration("load_hsv_from_file"), value_type=bool),
            "line_roi_top_ratio": ParameterValue(LaunchConfiguration("line_roi_top_ratio"), value_type=float),
            "line_roi_bottom_ratio": ParameterValue(LaunchConfiguration("line_roi_bottom_ratio"), value_type=float),
            "line_min_area": ParameterValue(LaunchConfiguration("line_min_area"), value_type=float),
            "line_lost_timeout_sec": ParameterValue(LaunchConfiguration("line_lost_timeout_sec"), value_type=float),
            "target_smooth_alpha": ParameterValue(LaunchConfiguration("target_smooth_alpha"), value_type=float),
            "line_pid_kp": ParameterValue(LaunchConfiguration("line_pid_kp"), value_type=float),
            "line_pid_ki": ParameterValue(LaunchConfiguration("line_pid_ki"), value_type=float),
            "line_pid_kd": ParameterValue(LaunchConfiguration("line_pid_kd"), value_type=float),
            "pid_scale": ParameterValue(LaunchConfiguration("pid_scale"), value_type=float),
            "pid_integral_limit": ParameterValue(LaunchConfiguration("pid_integral_limit"), value_type=float),
            "line_error_divisor": ParameterValue(LaunchConfiguration("line_error_divisor"), value_type=float),
            "line_linear_speed": ParameterValue(LaunchConfiguration("line_linear_speed"), value_type=float),
            "line_center_deadband_pixel": ParameterValue(LaunchConfiguration("line_center_deadband_pixel"), value_type=float),
            "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
            "reverse_angular_direction": ParameterValue(LaunchConfiguration("reverse_angular_direction"), value_type=bool),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "enable_obstacle_stop": ParameterValue(LaunchConfiguration("enable_obstacle_stop"), value_type=bool),
            "obstacle_min_valid_distance": ParameterValue(LaunchConfiguration("obstacle_min_valid_distance"), value_type=float),
            "obstacle_response_distance": ParameterValue(LaunchConfiguration("obstacle_response_distance"), value_type=float),
            "obstacle_check_angle_deg": ParameterValue(LaunchConfiguration("obstacle_check_angle_deg"), value_type=float),
            "obstacle_warning_count_threshold": ParameterValue(LaunchConfiguration("obstacle_warning_count_threshold"), value_type=int),
            "enable_beep": ParameterValue(LaunchConfiguration("enable_beep"), value_type=bool),
            "auto_start_tracking": ParameterValue(LaunchConfiguration("auto_start_tracking"), value_type=bool),
            "lost_line_stop": ParameterValue(LaunchConfiguration("lost_line_stop"), value_type=bool),
            "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
            "print_cmd_vel": ParameterValue(LaunchConfiguration("print_cmd_vel"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(launch_arguments + [node])

