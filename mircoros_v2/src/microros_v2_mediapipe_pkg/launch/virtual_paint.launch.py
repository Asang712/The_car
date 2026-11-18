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
    max_hands_arg = DeclareLaunchArgument(
        "max_hands",
        default_value="2",
        description="Maximum number of hands.",
    )
    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.85",
        description="Minimum hand detection confidence.",
    )
    tracking_con_arg = DeclareLaunchArgument(
        "tracking_con",
        default_value="0.5",
        description="Minimum hand tracking confidence.",
    )
    brush_thickness_arg = DeclareLaunchArgument(
        "brush_thickness",
        default_value="5",
        description="Brush thickness.",
    )
    eraser_thickness_arg = DeclareLaunchArgument(
        "eraser_thickness",
        default_value="100",
        description="Eraser thickness.",
    )
    top_height_arg = DeclareLaunchArgument(
        "top_height",
        default_value="50",
        description="Palette height.",
    )
    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="virtual_paint",
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
        executable="virtual_paint_node",
        name="virtual_paint_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "max_hands": LaunchConfiguration("max_hands"),
                "detection_con": LaunchConfiguration("detection_con"),
                "tracking_con": LaunchConfiguration("tracking_con"),
                "brush_thickness": LaunchConfiguration("brush_thickness"),
                "eraser_thickness": LaunchConfiguration("eraser_thickness"),
                "top_height": LaunchConfiguration("top_height"),
                "window_name": LaunchConfiguration("window_name"),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg, max_hands_arg, detection_con_arg, tracking_con_arg, brush_thickness_arg, eraser_thickness_arg, top_height_arg, window_name_arg, node
        ]
    )
