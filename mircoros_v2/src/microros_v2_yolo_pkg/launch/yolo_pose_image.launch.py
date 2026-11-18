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
        DeclareLaunchArgument("model_dir", default_value=f"{BASE}/MODELS/yolo"),
        DeclareLaunchArgument(
            "model_name",
            default_value="yolo11n-pose.pt",
            description="YOLO pose model filename. Example: yolo11s-pose.pt or yolo11s-pose.",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value="",
            description="Optional model path. If only a filename is given, model_dir will be used.",
        ),
        DeclareLaunchArgument("raw_image_dir", default_value=f"{BASE}/MODELS/yolo/assets"),
        DeclareLaunchArgument("image_name", default_value="bus.jpg"),
        DeclareLaunchArgument(
            "image_path",
            default_value="",
            description="Optional image path. If only a filename is given, raw_image_dir will be used.",
        ),
        DeclareLaunchArgument("output_dir", default_value=f"{BASE}/MODELS/yolo/assets/output"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("max_det", default_value="300"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("classes", default_value=""),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save", default_value="true"),
        DeclareLaunchArgument("line_width", default_value="2"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="540"),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_pose_image_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_pose_image_node",
        output="screen",
        parameters=[{
            "model_dir": LaunchConfiguration("model_dir"),
            "model_name": LaunchConfiguration("model_name"),
            "model_path": LaunchConfiguration("model_path"),
            "raw_image_dir": LaunchConfiguration("raw_image_dir"),
            "image_name": LaunchConfiguration("image_name"),
            "image_path": LaunchConfiguration("image_path"),
            "output_dir": LaunchConfiguration("output_dir"),
            "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
            "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
            "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
            "max_det": ParameterValue(LaunchConfiguration("max_det"), value_type=int),
            "device": LaunchConfiguration("device"),
            "classes": LaunchConfiguration("classes"),
            "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
            "save": ParameterValue(LaunchConfiguration("save"), value_type=bool),
            "line_width": ParameterValue(LaunchConfiguration("line_width"), value_type=int),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
        }],
    )

    return LaunchDescription(args + [node])
