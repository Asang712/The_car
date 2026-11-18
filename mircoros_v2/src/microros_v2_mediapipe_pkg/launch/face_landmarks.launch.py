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
    draw_indexes_arg = DeclareLaunchArgument(
        "draw_indexes",
        default_value="false",
        description="Whether to draw landmark indexes.",
    )
    eye_arg = DeclareLaunchArgument(
        "eye",
        default_value="true",
        description="Whether to prettify eyes.",
    )
    lips_arg = DeclareLaunchArgument(
        "lips",
        default_value="true",
        description="Whether to prettify lips.",
    )
    eyebrow_arg = DeclareLaunchArgument(
        "eyebrow",
        default_value="true",
        description="Whether to prettify eyebrows.",
    )
    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="face_landmarks",
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

    node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="face_landmarks_node",
        name="face_landmarks_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "draw_indexes": LaunchConfiguration("draw_indexes"),
                "eye": LaunchConfiguration("eye"),
                "lips": LaunchConfiguration("lips"),
                "eyebrow": LaunchConfiguration("eyebrow"),
                "window_name": LaunchConfiguration("window_name"),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg, draw_indexes_arg, eye_arg, lips_arg, eyebrow_arg, window_name_arg, node
        ]
    )
