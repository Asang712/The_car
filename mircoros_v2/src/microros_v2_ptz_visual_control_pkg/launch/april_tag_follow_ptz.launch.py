#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch AprilTag detection with KCF-style PTZ and chassis control."""

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
        "window_name": "april_tag_follow_ptz",
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
        "lost_frame_threshold": "5",
        "tag_family": "tag36h11",
        "tag_size": "0.048",
        "quad_decimate": "2.0",
        "tag_select_strategy": "largest",
        "camera_yaml_path": (
            "/home/yahboom/workspaces/MicroROS-V2-ROS2-SRC/"
            "microros_car_v2_ws/src/microros_v2_opencv_visual_pkg/config/camera.yaml"
        ),
        "use_camera_yaml": "true",
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
            "lost_frame_threshold": ParameterValue(
                LaunchConfiguration("lost_frame_threshold"), value_type=int
            ),
            "tag_family": LaunchConfiguration("tag_family"),
            "tag_size": ParameterValue(
                LaunchConfiguration("tag_size"), value_type=float
            ),
            "quad_decimate": ParameterValue(
                LaunchConfiguration("quad_decimate"), value_type=float
            ),
            "tag_select_strategy": LaunchConfiguration("tag_select_strategy"),
            "camera_yaml_path": LaunchConfiguration("camera_yaml_path"),
            "use_camera_yaml": ParameterValue(
                LaunchConfiguration("use_camera_yaml"), value_type=bool
            ),
        }
    )
    node = Node(
        package=PKG,
        executable="april_tag_follow_ptz_node",
        name="april_tag_follow_ptz_node",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [node])
