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

    max_faces_arg = DeclareLaunchArgument(
        "max_faces",
        default_value="2",
        description="Maximum number of faces to detect.",
    )

    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.5",
        description="Minimum face mesh detection confidence.",
    )

    tracking_con_arg = DeclareLaunchArgument(
        "tracking_con",
        default_value="0.5",
        description="Minimum face mesh tracking confidence.",
    )

    draw_on_source_arg = DeclareLaunchArgument(
        "draw_on_source",
        default_value="false",
        description="Whether to draw face mesh landmarks on source image.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="face_mesh",
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

    face_mesh_node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="face_mesh_node",
        name="face_mesh_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "static_image_mode": LaunchConfiguration("static_image_mode"),
                "max_faces": LaunchConfiguration("max_faces"),
                "detection_con": LaunchConfiguration("detection_con"),
                "tracking_con": LaunchConfiguration("tracking_con"),
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
            max_faces_arg,
            detection_con_arg,
            tracking_con_arg,
            draw_on_source_arg,
            window_name_arg,
            face_mesh_node,
        ]
    )
