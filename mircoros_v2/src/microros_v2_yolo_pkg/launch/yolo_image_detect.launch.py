#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


BASE = "/home/yahboom"
PYTHON_EXECUTABLE = PathJoinSubstitution([
    EnvironmentVariable("VIRTUAL_ENV", default_value="/usr"),
    "bin",
    "python3",
])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_name", default_value="yolo11n"),
        DeclareLaunchArgument(
            "model_path",
            default_value=f"{BASE}/MODELS/yolo/yolo11n.pt",
        ),

        DeclareLaunchArgument(
            "input_dir",
            default_value=f"{BASE}/MODELS/yolo/assets",
        ),
        DeclareLaunchArgument("image_name", default_value="bus.jpg"),
        DeclareLaunchArgument("input_image", default_value=""),

        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/assets/output",
        ),
        DeclareLaunchArgument("output_image", default_value=""),

        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.45"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("max_det", default_value="300"),

        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("window_name", default_value="YOLO Image Detection"),

        Node(
            package="microros_v2_yolo_pkg",
            executable="yolo_image_detect_node",
            prefix=PYTHON_EXECUTABLE,
            name="yolo_image_detect_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),

                "input_dir": LaunchConfiguration("input_dir"),
                "image_name": LaunchConfiguration("image_name"),
                "input_image": LaunchConfiguration("input_image"),

                "output_dir": LaunchConfiguration("output_dir"),
                "output_image": LaunchConfiguration("output_image"),

                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "imgsz": LaunchConfiguration("imgsz"),
                "device": LaunchConfiguration("device"),
                "max_det": LaunchConfiguration("max_det"),

                "show_window": LaunchConfiguration("show_window"),
                "window_name": LaunchConfiguration("window_name"),
            }],
        ),
    ])
