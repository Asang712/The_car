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
        description="Maximum number of hands to detect. 1 is recommended for gesture stability.",
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
        default_value="mediapipe_gesture",
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
        default_value="640",
        description="OpenCV display width. 0 means using original image width.",
    )

    display_height_arg = DeclareLaunchArgument(
        "display_height",
        default_value="480",
        description="OpenCV display height. 0 means using original image height.",
    )

    print_gesture_arg = DeclareLaunchArgument(
        "print_gesture",
        default_value="false",
        description="Whether to print gesture result in terminal.",
    )

    print_interval_arg = DeclareLaunchArgument(
        "print_interval",
        default_value="0.5",
        description="Gesture print interval in seconds.",
    )

    flip_image_arg = DeclareLaunchArgument(
        "flip_image",
        default_value="false",
        description="Whether to horizontally flip image.",
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

    mediapipe_gesture_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="mediapipe_gesture_node",
        name="mediapipe_gesture_node",
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
                "print_gesture": LaunchConfiguration("print_gesture"),
                "print_interval": LaunchConfiguration("print_interval"),
                "flip_image": LaunchConfiguration("flip_image"),
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
            print_gesture_arg,
            print_interval_arg,
            flip_image_arg,
            mediapipe_gesture_node,
        ]
    )
