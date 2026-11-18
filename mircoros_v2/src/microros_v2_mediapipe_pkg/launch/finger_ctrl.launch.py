#!/usr/bin/env python3
# encoding: utf-8

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/image_raw",
        description="Input camera image topic.",
    )

    static_image_mode_arg = DeclareLaunchArgument(
        "static_image_mode",
        default_value="false",
        description="MediaPipe static image mode.",
    )

    max_hands_arg = DeclareLaunchArgument(
        "max_hands",
        default_value="1",
        description="Maximum number of hands to detect. 1 is recommended for stable control.",
    )

    detector_con_arg = DeclareLaunchArgument(
        "detector_con",
        default_value="0.7",
        description="Minimum hand detection confidence.",
    )

    track_con_arg = DeclareLaunchArgument(
        "track_con",
        default_value="0.7",
        description="Minimum hand tracking confidence.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="finger_ctrl",
        description="OpenCV display window name.",
    )

    resize_display_arg = DeclareLaunchArgument(
        "resize_display",
        default_value="true",
        description="Whether to resize OpenCV display image.",
    )

    keep_aspect_ratio_arg = DeclareLaunchArgument(
        "keep_aspect_ratio",
        default_value="true",
        description="Keep aspect ratio and pad with black border to show the whole image.",
    )

    display_width_arg = DeclareLaunchArgument(
        "display_width",
        default_value="1280",
        description="OpenCV display width. 0 means using original combined image width.",
    )

    display_height_arg = DeclareLaunchArgument(
        "display_height",
        default_value="480",
        description="OpenCV display height. 0 means using original combined image height.",
    )

    use_distance_control_arg = DeclareLaunchArgument(
        "use_distance_control",
        default_value="true",
        description="Use thumb-index distance ratio instead of angle for more stable control.",
    )

    control_deadzone_percent_arg = DeclareLaunchArgument(
        "control_deadzone_percent",
        default_value="1.0",
        description="Percent deadzone. Values below this are forced to 0.",
    )

    zero_snap_percent_arg = DeclareLaunchArgument(
        "zero_snap_percent",
        default_value="0.0",
        description="Filtered percent below this value is snapped to 0.",
    )

    smooth_alpha_arg = DeclareLaunchArgument(
        "smooth_alpha",
        default_value="0.3",
        description="Low-pass filter coefficient. Smaller is smoother, larger is faster.",
    )

    min_dist_ratio_arg = DeclareLaunchArgument(
        "min_dist_ratio",
        default_value="0.23",
        description="Thumb-index distance ratio mapped to 0 percent.",
    )

    max_dist_ratio_arg = DeclareLaunchArgument(
        "max_dist_ratio",
        default_value="1.20",
        description="Thumb-index distance ratio mapped to 100 percent.",
    )

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

    finger_ctrl_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="finger_ctrl_node",
        name="finger_ctrl_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "static_image_mode": LaunchConfiguration("static_image_mode"),
                "max_hands": LaunchConfiguration("max_hands"),
                "detector_con": LaunchConfiguration("detector_con"),
                "track_con": LaunchConfiguration("track_con"),
                "window_name": LaunchConfiguration("window_name"),
                "resize_display": LaunchConfiguration("resize_display"),
                "keep_aspect_ratio": LaunchConfiguration("keep_aspect_ratio"),
                "display_width": LaunchConfiguration("display_width"),
                "display_height": LaunchConfiguration("display_height"),
                "use_distance_control": LaunchConfiguration("use_distance_control"),
                "control_deadzone_percent": LaunchConfiguration("control_deadzone_percent"),
                "zero_snap_percent": LaunchConfiguration("zero_snap_percent"),
                "smooth_alpha": LaunchConfiguration("smooth_alpha"),
                "min_dist_ratio": LaunchConfiguration("min_dist_ratio"),
                "max_dist_ratio": LaunchConfiguration("max_dist_ratio"),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg,
            static_image_mode_arg,
            max_hands_arg,
            detector_con_arg,
            track_con_arg,
            window_name_arg,
            resize_display_arg,
            keep_aspect_ratio_arg,
            display_width_arg,
            display_height_arg,
            use_distance_control_arg,
            control_deadzone_percent_arg,
            zero_snap_percent_arg,
            smooth_alpha_arg,
            min_dist_ratio_arg,
            max_dist_ratio_arg,
            finger_ctrl_node,
        ]
    )