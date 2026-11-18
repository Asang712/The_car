#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch file for april_tag_tracker_ptz_node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = "microros_v2_ptz_visual_control_pkg"
EXE = "april_tag_tracker_ptz_node"
NAME = "april_tag_tracker_ptz_node"


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("window_name", default_value="april_tag_tracker_ptz"),
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
        DeclareLaunchArgument("pd_kp", default_value="1.0"),
        DeclareLaunchArgument("pd_kd", default_value="0.25"),
        DeclareLaunchArgument("ptz_step_smooth_alpha", default_value="0.35"),
        DeclareLaunchArgument("max_ptz_step", default_value="1.2"),
        DeclareLaunchArgument("ptz_smooth_alpha", default_value="0.85"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true"),
        DeclareLaunchArgument("initial_publish_count", default_value="20"),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2"),
        DeclareLaunchArgument("tag_family", default_value="tag36h11"),
        DeclareLaunchArgument("tag_size", default_value="0.048"),
        DeclareLaunchArgument("quad_decimate", default_value="2.0"),
        DeclareLaunchArgument("tag_select_strategy", default_value="largest"),
        DeclareLaunchArgument("camera_yaml_path",
            default_value="/home/yahboom/workspaces/MicroROS-V2-ROS2-SRC/microros_car_v2_ws/src/microros_v2_opencv_visual_pkg/config/camera.yaml"),
        DeclareLaunchArgument("use_camera_yaml", default_value="true"),
        DeclareLaunchArgument("print_ptz_angle", default_value="false"),
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
        package=PKG, executable=EXE, name=NAME, output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "processing_width": ParameterValue(LaunchConfiguration("processing_width"), value_type=int),
            "processing_height": ParameterValue(LaunchConfiguration("processing_height"), value_type=int),
            "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
            "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "overlay_show_legend": ParameterValue(LaunchConfiguration("overlay_show_legend"), value_type=bool),
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
            "ptz_smooth_alpha": ParameterValue(LaunchConfiguration("ptz_smooth_alpha"), value_type=float),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
            "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
            "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
            "tag_family": LaunchConfiguration("tag_family"),
            "tag_size": ParameterValue(LaunchConfiguration("tag_size"), value_type=float),
            "quad_decimate": ParameterValue(LaunchConfiguration("quad_decimate"), value_type=float),
            "tag_select_strategy": LaunchConfiguration("tag_select_strategy"),
            "camera_yaml_path": LaunchConfiguration("camera_yaml_path"),
            "use_camera_yaml": ParameterValue(LaunchConfiguration("use_camera_yaml"), value_type=bool),
            "print_ptz_angle": ParameterValue(LaunchConfiguration("print_ptz_angle"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )
    return LaunchDescription(args + [node])
