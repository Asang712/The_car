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

    smooth_landmarks_arg = DeclareLaunchArgument(
        "smooth_landmarks",
        default_value="true",
        description="Whether to smooth pose landmarks.",
    )

    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.5",
        description="Minimum pose detection confidence.",
    )

    track_con_arg = DeclareLaunchArgument(
        "track_con",
        default_value="0.5",
        description="Minimum pose tracking confidence.",
    )

    draw_on_source_arg = DeclareLaunchArgument(
        "draw_on_source",
        default_value="false",
        description="Whether to draw pose landmarks on source image.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="pose_detector",
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

    pose_detector_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="pose_detector_node",
        name="pose_detector_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "static_image_mode": LaunchConfiguration("static_image_mode"),
                "smooth_landmarks": LaunchConfiguration("smooth_landmarks"),
                "detection_con": LaunchConfiguration("detection_con"),
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
            smooth_landmarks_arg,
            detection_con_arg,
            track_con_arg,
            draw_on_source_arg,
            window_name_arg,
            pose_detector_node,
        ]
    )
