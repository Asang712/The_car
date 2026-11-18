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
        DeclareLaunchArgument("model_path", default_value=f"{BASE}/MODELS/yolo/yolo11n-seg.pt"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("annotated_topic", default_value="/yolo_seg/annotated_image"),
        DeclareLaunchArgument("status_topic", default_value="/yolo_seg/status"),
        DeclareLaunchArgument("output_dir", default_value=f"{BASE}/MODELS/yolo/videos/output"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("conf", default_value="0.15"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("max_det", default_value="5"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("tracker", default_value="bytetrack.yaml"),
        DeclareLaunchArgument("use_tracking", default_value="false"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save_video", default_value="true"),
        DeclareLaunchArgument("output_fps", default_value="20.0"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("qos_depth", default_value="1"),
        DeclareLaunchArgument("process_every_n_frames", default_value="1"),
        DeclareLaunchArgument("line_width", default_value="2"),
        DeclareLaunchArgument("retina_masks", default_value="false"),
        DeclareLaunchArgument("publish_status", default_value="true"),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_seg_realtime_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_seg_realtime_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "image_topic": LaunchConfiguration("image_topic"),
            "annotated_topic": LaunchConfiguration("annotated_topic"),
            "status_topic": LaunchConfiguration("status_topic"),
            "output_dir": LaunchConfiguration("output_dir"),
            "imgsz": ParameterValue(LaunchConfiguration("imgsz"), value_type=int),
            "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
            "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
            "max_det": ParameterValue(LaunchConfiguration("max_det"), value_type=int),
            "device": LaunchConfiguration("device"),
            "tracker": LaunchConfiguration("tracker"),
            "use_tracking": ParameterValue(LaunchConfiguration("use_tracking"), value_type=bool),
            "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
            "save_video": ParameterValue(LaunchConfiguration("save_video"), value_type=bool),
            "output_fps": ParameterValue(LaunchConfiguration("output_fps"), value_type=float),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            "qos_reliability": LaunchConfiguration("qos_reliability"),
            "qos_depth": ParameterValue(LaunchConfiguration("qos_depth"), value_type=int),
            "process_every_n_frames": ParameterValue(LaunchConfiguration("process_every_n_frames"), value_type=int),
            "line_width": ParameterValue(LaunchConfiguration("line_width"), value_type=int),
            "retina_masks": ParameterValue(LaunchConfiguration("retina_masks"), value_type=bool),
            "publish_status": ParameterValue(LaunchConfiguration("publish_status"), value_type=bool),
        }],
    )

    return LaunchDescription(args + [node])
