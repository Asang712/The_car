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

        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("output_topic", default_value="/yolo/detect/image"),

        # CPU-friendly defaults for ordinary object detection.
        DeclareLaunchArgument("conf", default_value="0.3"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("max_det", default_value="5"),

        DeclareLaunchArgument("use_tracking", default_value="false"),
        DeclareLaunchArgument("tracker", default_value="bytetrack.yaml"),
        DeclareLaunchArgument("persist", default_value="true"),

        DeclareLaunchArgument("process_every_n_frames", default_value="1"),

        DeclareLaunchArgument("show_window", default_value="true"),
        DeclareLaunchArgument("window_name", default_value="YOLO Realtime Detection"),
        DeclareLaunchArgument("display_width", default_value="960"),
        DeclareLaunchArgument("display_height", default_value="720"),

        DeclareLaunchArgument("save_video", default_value="false"),
        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/videos/output",
        ),
        DeclareLaunchArgument("output_video_name", default_value="realtime_yolo_detect.mp4"),
        DeclareLaunchArgument("save_video_fps", default_value="15.0"),
        DeclareLaunchArgument("fourcc", default_value="mp4v"),

        Node(
            package="microros_v2_yolo_pkg",
            executable="yolo_realtime_detect_node",
            prefix=PYTHON_EXECUTABLE,
            name="yolo_realtime_detect_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),

                "image_topic": LaunchConfiguration("image_topic"),
                "output_topic": LaunchConfiguration("output_topic"),

                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "imgsz": LaunchConfiguration("imgsz"),
                "device": LaunchConfiguration("device"),
                "max_det": LaunchConfiguration("max_det"),

                "use_tracking": LaunchConfiguration("use_tracking"),
                "tracker": LaunchConfiguration("tracker"),
                "persist": LaunchConfiguration("persist"),

                "process_every_n_frames": LaunchConfiguration("process_every_n_frames"),

                "show_window": LaunchConfiguration("show_window"),
                "window_name": LaunchConfiguration("window_name"),
                "display_width": LaunchConfiguration("display_width"),
                "display_height": LaunchConfiguration("display_height"),

                "save_video": LaunchConfiguration("save_video"),
                "output_dir": LaunchConfiguration("output_dir"),
                "output_video_name": LaunchConfiguration("output_video_name"),
                "save_video_fps": LaunchConfiguration("save_video_fps"),
                "fourcc": LaunchConfiguration("fourcc"),
            }],
        ),
    ])
