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
    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.5",
        description="Minimum face detection confidence.",
    )
    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="face_detection",
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
        executable="face_detection_node",
        name="face_detection_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "detection_con": LaunchConfiguration("detection_con"),
                "window_name": LaunchConfiguration("window_name"),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg, detection_con_arg, window_name_arg, node
        ]
    )
