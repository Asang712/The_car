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
            default_value=f"{BASE}/MODELS/yolo/videos",
        ),
        DeclareLaunchArgument("video_name", default_value="street.mp4"),
        DeclareLaunchArgument("input_video", default_value=""),

        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/videos/output",
        ),
        DeclareLaunchArgument("output_video", default_value=""),

        # CPU-friendly defaults for ordinary object detection.
        DeclareLaunchArgument("conf", default_value="0.05"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("max_det", default_value="1000"),

        DeclareLaunchArgument("vid_stride", default_value="1"),
        DeclareLaunchArgument("max_frames", default_value="0"),

        DeclareLaunchArgument("use_tracking", default_value="false"),
        DeclareLaunchArgument("tracker", default_value="bytetrack.yaml"),
        DeclareLaunchArgument("persist", default_value="true"),

        DeclareLaunchArgument("show_window", default_value="false"),
        DeclareLaunchArgument("window_name", default_value="YOLO Video Detection"),
        DeclareLaunchArgument("display_width", default_value="960"),
        DeclareLaunchArgument("display_height", default_value="540"),

        DeclareLaunchArgument("fourcc", default_value="mp4v"),
        DeclareLaunchArgument("output_fps", default_value="0.0"),

        Node(
            package="microros_v2_yolo_pkg",
            executable="yolo_video_detect_node",
            prefix=PYTHON_EXECUTABLE,
            name="yolo_video_detect_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),

                "input_dir": LaunchConfiguration("input_dir"),
                "video_name": LaunchConfiguration("video_name"),
                "input_video": LaunchConfiguration("input_video"),

                "output_dir": LaunchConfiguration("output_dir"),
                "output_video": LaunchConfiguration("output_video"),

                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "imgsz": LaunchConfiguration("imgsz"),
                "device": LaunchConfiguration("device"),
                "max_det": LaunchConfiguration("max_det"),

                "vid_stride": LaunchConfiguration("vid_stride"),
                "max_frames": LaunchConfiguration("max_frames"),

                "use_tracking": LaunchConfiguration("use_tracking"),
                "tracker": LaunchConfiguration("tracker"),
                "persist": LaunchConfiguration("persist"),

                "show_window": LaunchConfiguration("show_window"),
                "window_name": LaunchConfiguration("window_name"),
                "display_width": LaunchConfiguration("display_width"),
                "display_height": LaunchConfiguration("display_height"),

                "fourcc": LaunchConfiguration("fourcc"),
                "output_fps": LaunchConfiguration("output_fps"),
            }],
        ),
    ])
