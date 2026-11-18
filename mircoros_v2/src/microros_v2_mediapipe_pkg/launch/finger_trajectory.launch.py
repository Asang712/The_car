#!/usr/bin/env python3
# encoding: utf-8

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic_arg = DeclareLaunchArgument(
        "rgb_topic",
        default_value="/camera/image_raw",
        description="Input camera image topic.",
    )

    detection_con_arg = DeclareLaunchArgument(
        "detection_con",
        default_value="0.65",
        description="Minimum hand detection confidence. Recommended 0.45~0.55 for 5~10 FPS camera.",
    )

    tracking_con_arg = DeclareLaunchArgument(
        "tracking_con",
        default_value="0.05",
        description="Minimum hand tracking confidence. Low value is better for low FPS camera.",
    )

    start_count_threshold_arg = DeclareLaunchArgument(
        "start_count_threshold",
        default_value="15",
        description="Frames required before tracking starts. Recommended 3~7 for 5~10 FPS camera.",
    )

    lost_timeout_arg = DeclareLaunchArgument(
        "lost_timeout",
        default_value="3.0",
        description="Timeout when finger is lost. Recommended 3.0~4.0 for low FPS camera.",
    )

    window_name_arg = DeclareLaunchArgument(
        "window_name",
        default_value="finger_trajectory",
        description="OpenCV main display window name.",
    )

    track_window_name_arg = DeclareLaunchArgument(
        "track_window_name",
        default_value="finger_track",
        description="OpenCV trajectory result window name.",
    )

    show_fps_arg = DeclareLaunchArgument(
        "show_fps",
        default_value="true",
        description="Show estimated FPS on image.",
    )

    start_prompt_duration_arg = DeclareLaunchArgument(
        "start_prompt_duration",
        default_value="1.2",
        description="Seconds to show START DRAWING prompt after tracking starts.",
    )

    overlay_font_scale_arg = DeclareLaunchArgument(
        "overlay_font_scale",
        default_value="0.45",
        description="Small overlay font scale. Increase slightly if text is too small.",
    )

    overlay_thickness_arg = DeclareLaunchArgument(
        "overlay_thickness",
        default_value="1",
        description="Overlay text thickness.",
    )

    overlay_alpha_arg = DeclareLaunchArgument(
        "overlay_alpha",
        default_value="0.35",
        description="Overlay background alpha. Lower value blocks less image.",
    )

    center_prompt_font_scale_arg = DeclareLaunchArgument(
        "center_prompt_font_scale",
        default_value="0.55",
        description="Bottom prompt font scale.",
    )

    display_scale_arg = DeclareLaunchArgument(
        "display_scale",
        default_value="1.8",
        description="Main camera display scale. Increase this if the OpenCV window is too small.",
    )

    track_display_scale_arg = DeclareLaunchArgument(
        "track_display_scale",
        default_value="2.2",
        description="Trajectory result display scale.",
    )

    display_width_arg = DeclareLaunchArgument(
        "display_width",
        default_value="0",
        description="Fixed main display width. 0 means use display_scale.",
    )

    display_height_arg = DeclareLaunchArgument(
        "display_height",
        default_value="0",
        description="Fixed main display height. 0 means use display_scale.",
    )

    track_display_width_arg = DeclareLaunchArgument(
        "track_display_width",
        default_value="0",
        description="Fixed trajectory result display width. 0 means use track_display_scale.",
    )

    track_display_height_arg = DeclareLaunchArgument(
        "track_display_height",
        default_value="0",
        description="Fixed trajectory result display height. 0 means use track_display_scale.",
    )

    min_point_distance_arg = DeclareLaunchArgument(
        "min_point_distance",
        default_value="5.0",
        description="Minimum pixel distance between two recorded trajectory points.",
    )

    smooth_alpha_arg = DeclareLaunchArgument(
        "smooth_alpha",
        default_value="0.55",
        description="Finger point smoothing coefficient. Higher means faster response, lower means smoother.",
    )

    track_line_thickness_arg = DeclareLaunchArgument(
        "track_line_thickness",
        default_value="3",
        description="Trajectory line thickness in main image.",
    )

    contour_area_threshold_arg = DeclareLaunchArgument(
        "contour_area_threshold",
        default_value="200.0",
        description="Minimum contour area for valid trajectory recognition.",
    )

    morph_iterations_arg = DeclareLaunchArgument(
        "morph_iterations",
        default_value="2",
        description="Erode and dilate iterations for trajectory shape cleanup.",
    )

    approx_epsilon_ratio_arg = DeclareLaunchArgument(
        "approx_epsilon_ratio",
        default_value="0.026",
        description="Polygon approximation ratio for shape recognition.",
    )

    venv_pythonpath = os.path.join(
        os.path.expanduser("~"),
        "microros_ws",
        "python_env",
        "mircoros_v2_env",
        ".venv",
        "lib",
        "python3.10",
        "site-packages",
    )
    original_pythonpath = os.environ.get("PYTHONPATH", "")

    node = Node(
        package="microros_v2_mediapipe_pkg",
        executable="finger_trajectory_node",
        name="finger_trajectory_node",
        output="screen",
        parameters=[
            {
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "detection_con": LaunchConfiguration("detection_con"),
                "tracking_con": LaunchConfiguration("tracking_con"),
                "start_count_threshold": LaunchConfiguration("start_count_threshold"),
                "lost_timeout": LaunchConfiguration("lost_timeout"),
                "window_name": LaunchConfiguration("window_name"),
                "track_window_name": LaunchConfiguration("track_window_name"),
                "show_fps": LaunchConfiguration("show_fps"),
                "start_prompt_duration": LaunchConfiguration("start_prompt_duration"),
                "overlay_font_scale": LaunchConfiguration("overlay_font_scale"),
                "overlay_thickness": LaunchConfiguration("overlay_thickness"),
                "overlay_alpha": LaunchConfiguration("overlay_alpha"),
                "center_prompt_font_scale": LaunchConfiguration("center_prompt_font_scale"),
                "display_scale": LaunchConfiguration("display_scale"),
                "track_display_scale": LaunchConfiguration("track_display_scale"),
                "display_width": LaunchConfiguration("display_width"),
                "display_height": LaunchConfiguration("display_height"),
                "track_display_width": LaunchConfiguration("track_display_width"),
                "track_display_height": LaunchConfiguration("track_display_height"),
                "min_point_distance": LaunchConfiguration("min_point_distance"),
                "smooth_alpha": LaunchConfiguration("smooth_alpha"),
                "track_line_thickness": LaunchConfiguration("track_line_thickness"),
                "contour_area_threshold": LaunchConfiguration("contour_area_threshold"),
                "morph_iterations": LaunchConfiguration("morph_iterations"),
                "approx_epsilon_ratio": LaunchConfiguration("approx_epsilon_ratio"),
            }
        ],
        additional_env={
            "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
        },
    )

    return LaunchDescription(
        [
            rgb_topic_arg,
            detection_con_arg,
            tracking_con_arg,
            start_count_threshold_arg,
            lost_timeout_arg,
            window_name_arg,
            track_window_name_arg,
            show_fps_arg,
            start_prompt_duration_arg,
            overlay_font_scale_arg,
            overlay_thickness_arg,
            overlay_alpha_arg,
            center_prompt_font_scale_arg,
            display_scale_arg,
            track_display_scale_arg,
            display_width_arg,
            display_height_arg,
            track_display_width_arg,
            track_display_height_arg,
            min_point_distance_arg,
            smooth_alpha_arg,
            track_line_thickness_arg,
            contour_area_threshold_arg,
            morph_iterations_arg,
            approx_epsilon_ratio_arg,
            node,
        ]
    )
