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
            "image_topic",
            default_value="/camera/image_raw",
            description="Input ROS2 image topic.",
        ),
        DeclareLaunchArgument(
            "annotated_topic",
            default_value="/yolo_cls/annotated_image",
            description="Output annotated image topic.",
        ),
        DeclareLaunchArgument(
            "status_topic",
            default_value="/yolo_cls/status",
            description="Output classification status topic.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=f"{BASE}/MODELS/yolo/videos/output",
            description="Output directory for realtime classification video.",
        ),
        DeclareLaunchArgument("imgsz", default_value="224"),
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("topk", default_value="5"),
        DeclareLaunchArgument("display_topk", default_value="3"),
        DeclareLaunchArgument("show", default_value="true"),
        DeclareLaunchArgument("save_video", default_value="true"),
        DeclareLaunchArgument("output_fps", default_value="20.0"),
        DeclareLaunchArgument("window_width", default_value="1280"),
        DeclareLaunchArgument("window_height", default_value="720"),
        DeclareLaunchArgument("qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("qos_depth", default_value="1"),
        DeclareLaunchArgument("process_every_n_frames", default_value="1"),
        DeclareLaunchArgument("publish_status", default_value="true"),

        # 小字号显示参数
        DeclareLaunchArgument(
            "text_scale",
            default_value="0.45",
            description="Small text scale on realtime view.",
        ),
        DeclareLaunchArgument(
            "text_thickness",
            default_value="1",
            description="Text thickness on realtime view.",
        ),
        DeclareLaunchArgument(
            "text_x",
            default_value="10",
            description="Text start x position.",
        ),
        DeclareLaunchArgument(
            "text_y",
            default_value="22",
            description="Text start y position.",
        ),
        DeclareLaunchArgument(
            "text_line_gap",
            default_value="18",
            description="Vertical gap between text lines.",
        ),
        DeclareLaunchArgument(
            "show_fps_text",
            default_value="true",
            description="Show FPS text on image.",
        ),
        DeclareLaunchArgument(
            "show_model_text",
            default_value="true",
            description="Show model name on image.",
        ),
        DeclareLaunchArgument(
            "draw_text_shadow",
            default_value="true",
            description="Draw black shadow for readable small text.",
        ),
    ]

    node = Node(
        package="microros_v2_yolo_pkg",
        executable="yolo_cls_realtime_node",
        prefix=PYTHON_EXECUTABLE,
        name="yolo_cls_realtime_node",
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
                "device": LaunchConfiguration("device"),
                "topk": ParameterValue(LaunchConfiguration("topk"), value_type=int),
                "display_topk": ParameterValue(
                    LaunchConfiguration("display_topk"),
                    value_type=int,
                ),
                "show": ParameterValue(LaunchConfiguration("show"), value_type=bool),
                "save_video": ParameterValue(
                    LaunchConfiguration("save_video"),
                    value_type=bool,
                ),
                "output_fps": ParameterValue(
                    LaunchConfiguration("output_fps"),
                    value_type=float,
                ),
                "window_width": ParameterValue(
                    LaunchConfiguration("window_width"),
                    value_type=int,
                ),
                "window_height": ParameterValue(
                    LaunchConfiguration("window_height"),
                    value_type=int,
                ),
                "qos_reliability": LaunchConfiguration("qos_reliability"),
                "qos_depth": ParameterValue(
                    LaunchConfiguration("qos_depth"),
                    value_type=int,
                ),
                "process_every_n_frames": ParameterValue(
                    LaunchConfiguration("process_every_n_frames"),
                    value_type=int,
                ),
                "publish_status": ParameterValue(
                    LaunchConfiguration("publish_status"),
                    value_type=bool,
                ),
                "text_scale": ParameterValue(
                    LaunchConfiguration("text_scale"),
                    value_type=float,
                ),
                "text_thickness": ParameterValue(
                    LaunchConfiguration("text_thickness"),
                    value_type=int,
                ),
                "text_x": ParameterValue(
                    LaunchConfiguration("text_x"),
                    value_type=int,
                ),
                "text_y": ParameterValue(
                    LaunchConfiguration("text_y"),
                    value_type=int,
                ),
                "text_line_gap": ParameterValue(
                    LaunchConfiguration("text_line_gap"),
                    value_type=int,
                ),
                "show_fps_text": ParameterValue(
                    LaunchConfiguration("show_fps_text"),
                    value_type=bool,
                ),
                "show_model_text": ParameterValue(
                    LaunchConfiguration("show_model_text"),
                    value_type=bool,
                ),
                "draw_text_shadow": ParameterValue(
                    LaunchConfiguration("draw_text_shadow"),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription(args + [node])
