#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch QR detection with KCF-style PTZ and chassis follow control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from microros_v2_ptz_visual_control_pkg.kcf_style_follow_launch import (
    control_launch_arguments,
    control_launch_parameters,
)


PKG = "microros_v2_ptz_visual_control_pkg"


def generate_launch_description() -> LaunchDescription:
    defaults = {
        "image_topic": "/camera/image_raw",
        "ptz_command_topic": "/cmd_ptz_angle",
        "cmd_vel_topic": "/cmd_vel",
        "window_name": "qr_follow_ptz",
        "show_debug_window": "true",
        "window_width": "640",
        "window_height": "480",
        "overlay_font_scale": "0.38",
        "overlay_line_height": "16",
        "overlay_margin_x": "8",
        "overlay_margin_y": "16",
        "overlay_show_background": "true",
        "enable_horizontal_flip": "false",
        "enable_vertical_flip": "false",
        "rotate_image_code": "none",
        "image_processing_rate_hz": "30.0",
        "lost_timeout": "0.5",
        "follow_stable_frames": "3",
        "lost_frame_threshold": "5",
        "qr_detector_backend": "pyzbar",
        "qr_select_strategy": "largest",
    }
    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ] + control_launch_arguments()

    parameters = control_launch_parameters()
    parameters.update(
        {
            "image_topic": LaunchConfiguration("image_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(
                LaunchConfiguration("show_debug_window"), value_type=bool
            ),
            "window_width": ParameterValue(
                LaunchConfiguration("window_width"), value_type=int
            ),
            "window_height": ParameterValue(
                LaunchConfiguration("window_height"), value_type=int
            ),
            "overlay_font_scale": ParameterValue(
                LaunchConfiguration("overlay_font_scale"), value_type=float
            ),
            "overlay_line_height": ParameterValue(
                LaunchConfiguration("overlay_line_height"), value_type=int
            ),
            "overlay_margin_x": ParameterValue(
                LaunchConfiguration("overlay_margin_x"), value_type=int
            ),
            "overlay_margin_y": ParameterValue(
                LaunchConfiguration("overlay_margin_y"), value_type=int
            ),
            "overlay_show_background": ParameterValue(
                LaunchConfiguration("overlay_show_background"), value_type=bool
            ),
            "enable_horizontal_flip": ParameterValue(
                LaunchConfiguration("enable_horizontal_flip"), value_type=bool
            ),
            "enable_vertical_flip": ParameterValue(
                LaunchConfiguration("enable_vertical_flip"), value_type=bool
            ),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "image_processing_rate_hz": ParameterValue(
                LaunchConfiguration("image_processing_rate_hz"), value_type=float
            ),
            "lost_timeout": ParameterValue(
                LaunchConfiguration("lost_timeout"), value_type=float
            ),
            "follow_stable_frames": ParameterValue(
                LaunchConfiguration("follow_stable_frames"), value_type=int
            ),
            "lost_frame_threshold": ParameterValue(
                LaunchConfiguration("lost_frame_threshold"), value_type=int
            ),
            "qr_detector_backend": LaunchConfiguration("qr_detector_backend"),
            "qr_select_strategy": LaunchConfiguration("qr_select_strategy"),
        }
    )
    node = Node(
        package=PKG,
        executable="qr_follow_ptz_node",
        name="qr_follow_ptz_node",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [node])
