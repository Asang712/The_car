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
        default_value="2",
        description="Maximum number of hands to detect.",
    )

    detector_con_arg = DeclareLaunchArgument(
        "detector_con",
        default_value="0.5",
        description="Minimum hand detection confidence.",
    )

    track_con_arg = DeclareLaunchArgument(
        "track_con",
        default_value="0.5",
        description="Minimum hand tracking confidence.",
    )

    draw_on_source_arg = DeclareLaunchArgument(
        "draw_on_source",
        default_value="false",
        description="Whether to draw hand landmarks on source image.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="hand_detector",
        description="OpenCV display window name.",
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

    hand_detector_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="hand_detector_node",
        name="hand_detector_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "static_image_mode": LaunchConfiguration("static_image_mode"),
                "max_hands": LaunchConfiguration("max_hands"),
                "detector_con": LaunchConfiguration("detector_con"),
                "track_con": LaunchConfiguration("track_con"),
                "draw_on_source": LaunchConfiguration("draw_on_source"),
                "window_name": LaunchConfiguration("window_name"),
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
            draw_on_source_arg,
            window_name_arg,
            hand_detector_node,
        ]
    )
