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
        default_value="true",
        description="Run object detection on every frame when true. Slower but easier for testing.",
    )

    max_objects_arg = DeclareLaunchArgument(
        "max_objects",
        default_value="5",
        description="Maximum number of objects to detect.",
    )

    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.5",
        description="Minimum object detection confidence.",
    )

    tracking_con_arg = DeclareLaunchArgument(
        "tracking_con",
        default_value="0.5",
        description="Minimum object tracking confidence.",
    )

    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="Cup",
        description="Objectron model name. Supported: Shoe, Chair, Cup, Camera.",
    )

    flip_image_arg = DeclareLaunchArgument(
        "flip_image",
        default_value="false",
        description="Whether to horizontally flip the display image.",
    )

    show_help_arg = DeclareLaunchArgument(
        "show_help",
        default_value="true",
        description="Show help text on OpenCV image.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="objectron",
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

    objectron_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="objectron_node",
        name="objectron_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "static_image_mode": LaunchConfiguration("static_image_mode"),
                "max_objects": LaunchConfiguration("max_objects"),
                "detection_con": LaunchConfiguration("detection_con"),
                "tracking_con": LaunchConfiguration("tracking_con"),
                "model_name": LaunchConfiguration("model_name"),
                "flip_image": LaunchConfiguration("flip_image"),
                "show_help": LaunchConfiguration("show_help"),
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
            max_objects_arg,
            detection_con_arg,
            tracking_con_arg,
            model_name_arg,
            flip_image_arg,
            show_help_arg,
            window_name_arg,
            objectron_node,
        ]
    )
