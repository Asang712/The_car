#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch file for gesture_follow_ptz_node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PKG = "microros_v2_ptz_visual_control_pkg"
EXE = "gesture_follow_ptz_node"
NAME = "gesture_follow_ptz_node"


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("window_name", default_value="gesture_follow_ptz"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("processing_width", default_value="320"),
        DeclareLaunchArgument("processing_height", default_value="240"),
        DeclareLaunchArgument("display_width", default_value="960"),
        DeclareLaunchArgument("display_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.32"),
        DeclareLaunchArgument("overlay_line_height", default_value="13"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("overlay_show_legend", default_value="true"),
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
        DeclareLaunchArgument("deadband_x", default_value="20.0"),
        DeclareLaunchArgument("deadband_y", default_value="20.0"),
        DeclareLaunchArgument("pd_kp", default_value="1.5"),
        DeclareLaunchArgument("pd_kd", default_value="0.35"),
        DeclareLaunchArgument("ptz_step_smooth_alpha", default_value="0.35"),
        DeclareLaunchArgument("max_ptz_step", default_value="1.2"),
        DeclareLaunchArgument("ptz_smooth_alpha", default_value="0.85"),
        DeclareLaunchArgument("ptz_control_rate", default_value="20.0"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("body_control_rate", default_value="10.0"),
        DeclareLaunchArgument("angular_kp", default_value="0.05"),
        DeclareLaunchArgument("angular_kd", default_value="0.05"),
        DeclareLaunchArgument("body_share", default_value="0.5"),
        DeclareLaunchArgument("body_ff_gain", default_value="1.3"),
        DeclareLaunchArgument("lost_frame_threshold", default_value="5"),
        DeclareLaunchArgument("angular_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("max_angular_z", default_value="0.12"),
        DeclareLaunchArgument("reverse_body_direction", default_value="true"),
        DeclareLaunchArgument("enable_distance_follow", default_value="true"),
        DeclareLaunchArgument("target_hand_size_px", default_value="150.0"),
        DeclareLaunchArgument("distance_tolerance_hand_size_px", default_value="12.0"),
        DeclareLaunchArgument("distance_kp", default_value="0.008"),
        DeclareLaunchArgument("distance_kd", default_value="0.003"),
        DeclareLaunchArgument("max_linear_x", default_value="0.12"),
        DeclareLaunchArgument("reverse_linear_direction", default_value="false"),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true"),
        DeclareLaunchArgument("initial_publish_count", default_value="20"),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2"),
        DeclareLaunchArgument("hand_detection_confidence", default_value="0.8"),
        DeclareLaunchArgument("print_ptz_angle", default_value="false"),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
    ]

    def bv(name):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)

    def iv(name):
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    def fv(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

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
        package=PKG,
        executable=EXE,
        name=NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": bv("show_debug_window"),
            "processing_width": iv("processing_width"),
            "processing_height": iv("processing_height"),
            "display_width": iv("display_width"),
            "display_height": iv("display_height"),
            "overlay_font_scale": fv("overlay_font_scale"),
            "overlay_line_height": iv("overlay_line_height"),
            "overlay_show_background": bv("overlay_show_background"),
            "overlay_show_legend": bv("overlay_show_legend"),
            "enable_horizontal_flip": bv("enable_horizontal_flip"),
            "enable_vertical_flip": bv("enable_vertical_flip"),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "horizontal_min_deg": fv("horizontal_min_deg"),
            "horizontal_max_deg": fv("horizontal_max_deg"),
            "pitch_min_deg": fv("pitch_min_deg"),
            "pitch_max_deg": fv("pitch_max_deg"),
            "initial_horizontal_deg": fv("initial_horizontal_deg"),
            "initial_pitch_deg": fv("initial_pitch_deg"),
            "reverse_horizontal_direction": bv("reverse_horizontal_direction"),
            "reverse_pitch_direction": bv("reverse_pitch_direction"),
            "deadband_x": fv("deadband_x"),
            "deadband_y": fv("deadband_y"),
            "pd_kp": fv("pd_kp"),
            "pd_kd": fv("pd_kd"),
            "ptz_step_smooth_alpha": fv("ptz_step_smooth_alpha"),
            "max_ptz_step": fv("max_ptz_step"),
            "ptz_smooth_alpha": fv("ptz_smooth_alpha"),
            "ptz_control_rate": fv("ptz_control_rate"),
            "image_processing_rate_hz": fv("image_processing_rate_hz"),
            "body_control_rate": fv("body_control_rate"),
            "angular_kp": fv("angular_kp"),
            "angular_kd": fv("angular_kd"),
            "body_share": fv("body_share"),
            "body_ff_gain": fv("body_ff_gain"),
            "lost_frame_threshold": iv("lost_frame_threshold"),
            "angular_tolerance_deg": fv("angular_tolerance_deg"),
            "max_angular_z": fv("max_angular_z"),
            "reverse_body_direction": bv("reverse_body_direction"),
            "enable_distance_follow": bv("enable_distance_follow"),
            "target_hand_size_px": fv("target_hand_size_px"),
            "distance_tolerance_hand_size_px": fv("distance_tolerance_hand_size_px"),
            "distance_kp": fv("distance_kp"),
            "distance_kd": fv("distance_kd"),
            "max_linear_x": fv("max_linear_x"),
            "reverse_linear_direction": bv("reverse_linear_direction"),
            "publish_initial_ptz_command": bv("publish_initial_ptz_command"),
            "initial_publish_count": iv("initial_publish_count"),
            "initial_publish_interval_sec": fv("initial_publish_interval_sec"),
            "hand_detection_confidence": fv("hand_detection_confidence"),
            "print_ptz_angle": bv("print_ptz_angle"),
            "print_interval_sec": fv("print_interval_sec"),
            "enable_rqt_dynamic_parameters": bv("enable_rqt_dynamic_parameters"),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )
    return LaunchDescription(args + [node])

