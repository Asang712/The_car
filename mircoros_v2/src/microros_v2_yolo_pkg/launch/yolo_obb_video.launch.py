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
            default_value="yolo11n-obb.pt",
            description="YOLO OBB model filename. Example: yolo11s-obb.pt or yolo11s-obb.",
        ),
        DeclareLaunchArgument("model_path", default_value=""),
        DeclareLaunchArgument("raw_video_dir", default_value=f"{BASE}/MODELS/yolo/videos"),
        DeclareLaunchArgument("video_name", default_value="street.mp4"),
        DeclareLaunchArgument("video_path", default_value=""),
        DeclareLaunchArgument("output_dir", default_value=f"{BASE}/MODELS/yolo/videos/output"),
        DeclareLaunchArgument("imgsz", default_value="1024"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("max_det", default_value="1000"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("classes", default_value=""),
        DeclareLaunchArgument("vid_stride", default_value="1"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save", default_value="true"),
        DeclareLaunchArgument("output_fps", default_value="0.0"),
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("line_width", default_value="2"),
        DeclareLaunchArgument("print_classes", default_value="true"),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_obb_video_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_obb_video_node",
        output="screen",
        parameters=[
            {
                "model_dir": LaunchConfiguration("model_dir"),
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),
                "raw_video_dir": LaunchConfiguration("raw_video_dir"),
                "video_name": LaunchConfiguration("video_name"),
                "video_path": LaunchConfiguration("video_path"),
                "output_dir": LaunchConfiguration("output_dir"),
                "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
                "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
                "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
                "max_det": ParameterValue(LaunchConfiguration("max_det"), value_type=int),
                "device": LaunchConfiguration("device"),
                "classes": LaunchConfiguration("classes"),
                "vid_stride": ParameterValue(LaunchConfiguration("vid_stride"), value_type=int),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
                "save": ParameterValue(LaunchConfiguration("save"), value_type=bool),
                "output_fps": ParameterValue(LaunchConfiguration("output_fps"), value_type=float),
                "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
                "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
                "line_width": ParameterValue(LaunchConfiguration("line_width"), value_type=int),
                "print_classes": ParameterValue(LaunchConfiguration("print_classes"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(args + [node])
