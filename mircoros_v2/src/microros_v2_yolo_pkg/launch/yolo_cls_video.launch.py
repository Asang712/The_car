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
            "raw_video_dir",
            default_value=f"{BASE}/MODELS/yolo/videos",
            description="Raw video directory.",
        ),
        DeclareLaunchArgument(
            "video_name",
            default_value="cup.mp4",
            description="Video filename under raw_video_dir.",
        ),
        DeclareLaunchArgument(
            "video_path",
            default_value="",
            description="Optional video path. If only a filename is given, raw_video_dir will be used.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/videos/output",
            description="Output directory for classified videos.",
        ),
        DeclareLaunchArgument("imgsz", default_value="224"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("topk", default_value="5"),
        DeclareLaunchArgument("vid_stride", default_value="1"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save", default_value="true"),
        DeclareLaunchArgument("output_fps", default_value="0.0"),
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_cls_video_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_cls_video_node",
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
                "device": LaunchConfiguration("device"),
                "topk": ParameterValue(LaunchConfiguration("topk"), value_type=int),
                "vid_stride": ParameterValue(LaunchConfiguration("vid_stride"), value_type=int),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
                "save": ParameterValue(LaunchConfiguration("save"), value_type=bool),
                "output_fps": ParameterValue(LaunchConfiguration("output_fps"), value_type=float),
                "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
                "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            }
        ],
    )

    return LaunchDescription(args + [node])
