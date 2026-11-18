#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch file for pose_follow (No-PTZ)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "microros_v2_no_ptz_visual_control_pkg"

def generate_launch_description():
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("window_name", default_value="pose_tracker"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("processing_width", default_value="320"),
        DeclareLaunchArgument("processing_height", default_value="240"),
        DeclareLaunchArgument("display_width", default_value="960"),
        DeclareLaunchArgument("display_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.32"),
        DeclareLaunchArgument("overlay_line_height", default_value="13"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("pose_detection_confidence", default_value="0.5"),
        DeclareLaunchArgument("pose_tracking_confidence", default_value="0.5"),
        DeclareLaunchArgument("deadband_x_pixel", default_value="60.0"),
        DeclareLaunchArgument("angular_kp", default_value="1.8"),
        DeclareLaunchArgument("angular_kd", default_value="0.8"),
        DeclareLaunchArgument("max_angular_z", default_value="0.6"),
        DeclareLaunchArgument("lost_frame_threshold", default_value="5"),
        DeclareLaunchArgument("reverse_body_direction", default_value="true"),
        DeclareLaunchArgument("enable_distance_follow", default_value="false"),
        DeclareLaunchArgument("target_pose_size_px", default_value="320.0"),
        DeclareLaunchArgument("distance_tolerance_pose_size_px", default_value="10.0"),
        DeclareLaunchArgument("distance_kp", default_value="0.02"),
        DeclareLaunchArgument("distance_kd", default_value="0.01"),
        DeclareLaunchArgument("max_linear_x", default_value="0.30"),
        DeclareLaunchArgument("reverse_linear_direction", default_value="false"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("body_control_rate", default_value="10.0"),
        DeclareLaunchArgument("show_deadband_lines", default_value="false"),
        DeclareLaunchArgument("show_center_line", default_value="false"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
    ]
    params = {
        "image_topic": lc("image_topic"),
        "cmd_vel_topic": lc("cmd_vel_topic"),
        "window_name": lc("window_name"),
        "show_debug_window": lc("show_debug_window"),
        "processing_width": lc("processing_width"),
        "processing_height": lc("processing_height"),
        "display_width": lc("display_width"),
        "display_height": lc("display_height"),
        "overlay_font_scale": lc("overlay_font_scale"),
        "overlay_line_height": lc("overlay_line_height"),
        "overlay_show_background": lc("overlay_show_background"),
        "enable_horizontal_flip": lc("enable_horizontal_flip"),
        "enable_vertical_flip": lc("enable_vertical_flip"),
        "rotate_image_code": lc("rotate_image_code"),
        "pose_detection_confidence": lc("pose_detection_confidence"),
        "pose_tracking_confidence": lc("pose_tracking_confidence"),
        "deadband_x_pixel": lc("deadband_x_pixel"),
        "angular_kp": lc("angular_kp"),
        "angular_kd": lc("angular_kd"),
        "max_angular_z": lc("max_angular_z"),
        "lost_frame_threshold": lc("lost_frame_threshold"),
        "reverse_body_direction": lc("reverse_body_direction"),
        "enable_distance_follow": lc("enable_distance_follow"),
        "target_pose_size_px": lc("target_pose_size_px"),
        "distance_tolerance_pose_size_px": lc("distance_tolerance_pose_size_px"),
        "distance_kp": lc("distance_kp"),
        "distance_kd": lc("distance_kd"),
        "max_linear_x": lc("max_linear_x"),
        "reverse_linear_direction": lc("reverse_linear_direction"),
        "image_processing_rate_hz": lc("image_processing_rate_hz"),
        "body_control_rate": lc("body_control_rate"),
        "show_deadband_lines": lc("show_deadband_lines"),
        "show_center_line": lc("show_center_line"),
        "enable_rqt_dynamic_parameters": lc("enable_rqt_dynamic_parameters")
    }
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

    node = Node(package=PKG, executable="pose_tracker_node", name="pose_tracker_node", output="screen", parameters=[params], additional_env={"PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath})
    return LaunchDescription(args + [node])
