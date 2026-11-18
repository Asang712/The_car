#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
hand_follow_ptz.launch.py

ROS2 Humble 手掌检测 + PTZ 云台 + 车身跟随节点启动文件。
PTZ 控制框架与 gesture_follow_ptz_node 一致。
车身控制：PTZ 水平角度 → 车身 share PD → /cmd_vel.angular.z。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"
EXECUTABLE_NAME = "hand_follow_ptz_node"
NODE_NAME = "hand_follow_ptz_node"


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("window_name", default_value="hand_follow_ptz"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("processing_width", default_value="320"),
        DeclareLaunchArgument("processing_height", default_value="240"),
        DeclareLaunchArgument("display_width", default_value="640"),
        DeclareLaunchArgument("display_height", default_value="480"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.32"),
        DeclareLaunchArgument("overlay_line_height", default_value="13"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("horizontal_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("horizontal_max_deg", default_value="90.0"),
        DeclareLaunchArgument("pitch_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("pitch_max_deg", default_value="20.0"),
        DeclareLaunchArgument("reverse_horizontal_direction", default_value="false"),
        DeclareLaunchArgument("reverse_pitch_direction", default_value="true"),
        DeclareLaunchArgument("deadband_x", default_value="20.0"),
        DeclareLaunchArgument("deadband_y", default_value="20.0"),
        DeclareLaunchArgument("pd_kp", default_value="0.7"),
        DeclareLaunchArgument("pd_kd", default_value="0.25"),
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

    node = Node(package=PACKAGE_NAME, executable=EXECUTABLE_NAME, name=NODE_NAME, output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "processing_width": ParameterValue(LaunchConfiguration("processing_width"), value_type=int),
            "processing_height": ParameterValue(LaunchConfiguration("processing_height"), value_type=int),
            "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
            "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "horizontal_min_deg": ParameterValue(LaunchConfiguration("horizontal_min_deg"), value_type=float),
            "horizontal_max_deg": ParameterValue(LaunchConfiguration("horizontal_max_deg"), value_type=float),
            "pitch_min_deg": ParameterValue(LaunchConfiguration("pitch_min_deg"), value_type=float),
            "pitch_max_deg": ParameterValue(LaunchConfiguration("pitch_max_deg"), value_type=float),
            "reverse_horizontal_direction": ParameterValue(LaunchConfiguration("reverse_horizontal_direction"), value_type=bool),
            "reverse_pitch_direction": ParameterValue(LaunchConfiguration("reverse_pitch_direction"), value_type=bool),
            "deadband_x": ParameterValue(LaunchConfiguration("deadband_x"), value_type=float),
            "deadband_y": ParameterValue(LaunchConfiguration("deadband_y"), value_type=float),
            "pd_kp": ParameterValue(LaunchConfiguration("pd_kp"), value_type=float),
            "pd_kd": ParameterValue(LaunchConfiguration("pd_kd"), value_type=float),
            "ptz_step_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_step_smooth_alpha"), value_type=float),
            "max_ptz_step": ParameterValue(LaunchConfiguration("max_ptz_step"), value_type=float),
            "ptz_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_smooth_alpha"), value_type=float),
            "ptz_control_rate": ParameterValue(LaunchConfiguration("ptz_control_rate"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "body_control_rate": ParameterValue(LaunchConfiguration("body_control_rate"), value_type=float),
            "angular_kp": ParameterValue(LaunchConfiguration("angular_kp"), value_type=float),
            "angular_kd": ParameterValue(LaunchConfiguration("angular_kd"), value_type=float),
            "body_share": ParameterValue(LaunchConfiguration("body_share"), value_type=float),
            "body_ff_gain": ParameterValue(LaunchConfiguration("body_ff_gain"), value_type=float),
            "lost_frame_threshold": ParameterValue(LaunchConfiguration("lost_frame_threshold"), value_type=int),
            "angular_tolerance_deg": ParameterValue(LaunchConfiguration("angular_tolerance_deg"), value_type=float),
            "max_angular_z": ParameterValue(LaunchConfiguration("max_angular_z"), value_type=float),
            "reverse_body_direction": ParameterValue(LaunchConfiguration("reverse_body_direction"), value_type=bool),
            "enable_distance_follow": ParameterValue(LaunchConfiguration("enable_distance_follow"), value_type=bool),
            "target_hand_size_px": ParameterValue(LaunchConfiguration("target_hand_size_px"), value_type=float),
            "distance_tolerance_hand_size_px": ParameterValue(LaunchConfiguration("distance_tolerance_hand_size_px"), value_type=float),
            "distance_kp": ParameterValue(LaunchConfiguration("distance_kp"), value_type=float),
            "distance_kd": ParameterValue(LaunchConfiguration("distance_kd"), value_type=float),
            "max_linear_x": ParameterValue(LaunchConfiguration("max_linear_x"), value_type=float),
            "reverse_linear_direction": ParameterValue(LaunchConfiguration("reverse_linear_direction"), value_type=bool),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(args + [node])
