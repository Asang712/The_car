#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


BASE = "/home/yahboom"
PYTHON_EXECUTABLE = PathJoinSubstitution([
    EnvironmentVariable("VIRTUAL_ENV", default_value="/usr"),
    "bin",
    "python3",
])


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "model_dir",
            default_value=f"{BASE}/MODELS/yolo",
            description="YOLO classification model directory.",
        ),
        DeclareLaunchArgument(
            "model_name",
            default_value="yolo11n-cls.pt",
            description="YOLO classification model filename. Example: yolo11s-cls.pt or yolo11s-cls.",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="Optional model path. If only a filename is given, model_dir will be used.",
        ),
        DeclareLaunchArgument(
            "raw_image_dir",
            default_value=f"{BASE}/MODELS/yolo/assets",
            description="Raw image directory.",
        ),
        DeclareLaunchArgument(
            "image_name",
            default_value="dog.jpg",
            description="Image filename under raw_image_dir.",
        ),
        DeclareLaunchArgument(
            "image_path",
            default_value="",
            description="Optional image path. If only a filename is given, raw_image_dir will be used.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/assets/output",
            description="Output directory for classified images.",
        ),
        DeclareLaunchArgument("imgsz", default_value="224"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("topk", default_value="5"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save", default_value="true"),
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="1.1",
            description="Classification overlay font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_thickness",
            default_value="3",
            description="Classification overlay text thickness in pixels.",
        ),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_cls_image_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_cls_image_node",
        output="screen",
        parameters=[
            {
                "model_dir": LaunchConfiguration("model_dir"),
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),
                "raw_image_dir": LaunchConfiguration("raw_image_dir"),
                "image_name": LaunchConfiguration("image_name"),
                "image_path": LaunchConfiguration("image_path"),
                "output_dir": LaunchConfiguration("output_dir"),
                "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                "device": LaunchConfiguration("device"),
                "topk": ParameterValue(LaunchConfiguration("topk"), value_type=int),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
                "save": ParameterValue(LaunchConfiguration("save"), value_type=bool),
                "window_width": ParameterValue(
                    LaunchConfiguration("window_width"), value_type=int
                ),
                "window_height": ParameterValue(
                    LaunchConfiguration("window_height"), value_type=int
                ),
                "overlay_font_scale": ParameterValue(
                    LaunchConfiguration("overlay_font_scale"), value_type=float
                ),
                "overlay_thickness": ParameterValue(
                    LaunchConfiguration("overlay_thickness"), value_type=int
                ),
            }
        ],
    )

    return LaunchDescription(args + [node])
