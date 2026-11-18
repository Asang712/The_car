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
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("annotated_topic", default_value="/yolo_obb/annotated_image"),
        DeclareLaunchArgument("status_topic", default_value="/yolo_obb/status"),
        DeclareLaunchArgument("output_dir", default_value=f"{BASE}/MODELS/yolo/videos/output"),
        DeclareLaunchArgument("imgsz", default_value="960"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.7"),
        DeclareLaunchArgument("max_det", default_value="20"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("classes", default_value=""),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save_video", default_value="true"),
        DeclareLaunchArgument("output_fps", default_value="10.0"),
        DeclareLaunchArgument("window_width", default_value="960"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("qos_depth", default_value="1"),
        DeclareLaunchArgument("process_every_n_frames", default_value="1"),
        DeclareLaunchArgument("line_width", default_value="2"),
        DeclareLaunchArgument("publish_status", default_value="true"),
        DeclareLaunchArgument("draw_fps_text", default_value="true"),
        DeclareLaunchArgument("text_scale", default_value="0.65"),
        DeclareLaunchArgument("text_thickness", default_value="2"),
        DeclareLaunchArgument("print_classes", default_value="true"),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_obb_realtime_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_obb_realtime_node",
        output="screen",
        parameters=[
            {
                "model_dir": LaunchConfiguration("model_dir"),
                "model_name": LaunchConfiguration("model_name"),
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
                "classes": LaunchConfiguration("classes"),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
                "save_video": ParameterValue(LaunchConfiguration("save_video"), value_type=bool),
                "output_fps": ParameterValue(LaunchConfiguration("output_fps"), value_type=float),
                "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
                "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
                "qos_reliability": LaunchConfiguration("qos_reliability"),
                "qos_depth": ParameterValue(LaunchConfiguration("qos_depth"), value_type=int),
                "process_every_n_frames": ParameterValue(LaunchConfiguration("process_every_n_frames"), value_type=int),
                "line_width": ParameterValue(LaunchConfiguration("line_width"), value_type=int),
                "publish_status": ParameterValue(LaunchConfiguration("publish_status"), value_type=bool),
                "draw_fps_text": ParameterValue(LaunchConfiguration("draw_fps_text"), value_type=bool),
                "text_scale": ParameterValue(LaunchConfiguration("text_scale"), value_type=float),
                "text_thickness": ParameterValue(LaunchConfiguration("text_thickness"), value_type=int),
                "print_classes": ParameterValue(LaunchConfiguration("print_classes"), value_type=bool),
            }
        ],
    )

    return LaunchDescription(args + [node])
