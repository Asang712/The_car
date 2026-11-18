#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gesture_action_group_control_ptz.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"
EXECUTABLE_NAME = "gesture_action_group_control_ptz_node"
NODE_NAME = "gesture_action_group_control_ptz_node"
WORKSPACE_ROOT = Path(get_package_prefix(PACKAGE_NAME)).parents[1]
DEFAULT_PYTHON_EXECUTABLE = str(
    WORKSPACE_ROOT / "python_env" / "mircoros_v2_env" / ".venv" / "bin" / "python3"
)


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "python_executable",
            default_value=DEFAULT_PYTHON_EXECUTABLE,
            description="Python interpreter containing MediaPipe and ROS 2 Python packages.",
        ),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("odom_topic", default_value="/odom_raw"),
        DeclareLaunchArgument("window_name", default_value="gesture_action_ctrl"),
        DeclareLaunchArgument("show_debug_window", default_value="true"),
        DeclareLaunchArgument("processing_width", default_value="320"),
        DeclareLaunchArgument("processing_height", default_value="240"),
        DeclareLaunchArgument("display_width", default_value="960"),
        DeclareLaunchArgument("display_height", default_value="720"),
        DeclareLaunchArgument("overlay_font_scale", default_value="0.32"),
        DeclareLaunchArgument("overlay_line_height", default_value="13"),
        DeclareLaunchArgument("overlay_show_background", default_value="false"),
        DeclareLaunchArgument("enable_horizontal_flip", default_value="false"),
        DeclareLaunchArgument("enable_vertical_flip", default_value="false"),
        DeclareLaunchArgument("rotate_image_code", default_value="none"),
        DeclareLaunchArgument("waiting_horizontal_deg", default_value="0.0"),
        DeclareLaunchArgument("waiting_pitch_deg", default_value="-15.0"),
        DeclareLaunchArgument("action_horizontal_deg", default_value="0.0"),
        DeclareLaunchArgument("action_pitch_deg", default_value="-45.0"),
        DeclareLaunchArgument("square_forward_distance", default_value="0.5"),
        DeclareLaunchArgument("square_turn_angle_deg", default_value="90.0"),
        DeclareLaunchArgument("square_cycles", default_value="1"),
        DeclareLaunchArgument("backforth_distance", default_value="0.5"),
        DeclareLaunchArgument("circle_total_angle_deg", default_value="360.0"),
        DeclareLaunchArgument("circle_kp", default_value="0.05"),
        DeclareLaunchArgument("circle_kd", default_value="0.005"),
        DeclareLaunchArgument("circle_max_angular_z", default_value="0.5"),
        DeclareLaunchArgument("circle_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("circle_settle_frames", default_value="3"),
        DeclareLaunchArgument("s_turn_angles", default_value="[30.0, -60.0, 60.0, -60.0, 30.0]"),
        DeclareLaunchArgument("s_straight_distances", default_value="[0.288, 0.576, 0.576, 0.288, 0.5]"),
        DeclareLaunchArgument("s_linear_speed", default_value="0.15"),
        DeclareLaunchArgument("linear_kp", default_value="0.8"),
        DeclareLaunchArgument("linear_kd", default_value="0.25"),
        DeclareLaunchArgument("max_linear_x", default_value="0.2"),
        DeclareLaunchArgument("angular_kp", default_value="0.05"),
        DeclareLaunchArgument("angular_kd", default_value="0.005"),
        DeclareLaunchArgument("max_angular_z", default_value="0.6"),
        DeclareLaunchArgument("distance_tolerance", default_value="0.05"),
        DeclareLaunchArgument("angle_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("settle_frames", default_value="3"),
        DeclareLaunchArgument("phase_cooldown_ticks", default_value="10"),
        DeclareLaunchArgument("image_processing_rate_hz", default_value="15.0"),
        DeclareLaunchArgument("action_control_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("gesture_hold_confirm_frames", default_value="3"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
    ]

    node = Node(package=PACKAGE_NAME, executable=EXECUTABLE_NAME,
                name=NODE_NAME, output="screen",
                prefix=LaunchConfiguration("python_executable"), parameters=[{
        "image_topic": LaunchConfiguration("image_topic"),
        "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
        "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
        "odom_topic": LaunchConfiguration("odom_topic"),
        "window_name": LaunchConfiguration("window_name"),
        "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
        "processing_width": ParameterValue(LaunchConfiguration("processing_width"), value_type=int),
        "processing_height": ParameterValue(LaunchConfiguration("processing_height"), value_type=int),
        "display_width": ParameterValue(LaunchConfiguration("display_width"), value_type=int),
        "display_height": ParameterValue(LaunchConfiguration("display_height"), value_type=int),
        "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
        "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
        "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
        "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
        "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
        "rotate_image_code": LaunchConfiguration("rotate_image_code"),
        "waiting_horizontal_deg": ParameterValue(LaunchConfiguration("waiting_horizontal_deg"), value_type=float),
        "waiting_pitch_deg": ParameterValue(LaunchConfiguration("waiting_pitch_deg"), value_type=float),
        "action_horizontal_deg": ParameterValue(LaunchConfiguration("action_horizontal_deg"), value_type=float),
        "action_pitch_deg": ParameterValue(LaunchConfiguration("action_pitch_deg"), value_type=float),
        "square_forward_distance": ParameterValue(LaunchConfiguration("square_forward_distance"), value_type=float),
        "square_turn_angle_deg": ParameterValue(LaunchConfiguration("square_turn_angle_deg"), value_type=float),
        "square_cycles": ParameterValue(LaunchConfiguration("square_cycles"), value_type=int),
        "backforth_distance": ParameterValue(LaunchConfiguration("backforth_distance"), value_type=float),
        "circle_total_angle_deg": ParameterValue(LaunchConfiguration("circle_total_angle_deg"), value_type=float),
        "circle_kp": ParameterValue(LaunchConfiguration("circle_kp"), value_type=float),
        "circle_kd": ParameterValue(LaunchConfiguration("circle_kd"), value_type=float),
        "circle_max_angular_z": ParameterValue(LaunchConfiguration("circle_max_angular_z"), value_type=float),
        "circle_tolerance_deg": ParameterValue(LaunchConfiguration("circle_tolerance_deg"), value_type=float),
        "circle_settle_frames": ParameterValue(LaunchConfiguration("circle_settle_frames"), value_type=int),
        "s_turn_angles": LaunchConfiguration("s_turn_angles"),
        "s_straight_distances": LaunchConfiguration("s_straight_distances"),
        "s_linear_speed": ParameterValue(LaunchConfiguration("s_linear_speed"), value_type=float),
        "linear_kp": ParameterValue(LaunchConfiguration("linear_kp"), value_type=float),
        "linear_kd": ParameterValue(LaunchConfiguration("linear_kd"), value_type=float),
        "max_linear_x": ParameterValue(LaunchConfiguration("max_linear_x"), value_type=float),
        "angular_kp": ParameterValue(LaunchConfiguration("angular_kp"), value_type=float),
        "angular_kd": ParameterValue(LaunchConfiguration("angular_kd"), value_type=float),
        "max_angular_z": ParameterValue(LaunchConfiguration("max_angular_z"), value_type=float),
        "distance_tolerance": ParameterValue(LaunchConfiguration("distance_tolerance"), value_type=float),
        "angle_tolerance_deg": ParameterValue(LaunchConfiguration("angle_tolerance_deg"), value_type=float),
        "settle_frames": ParameterValue(LaunchConfiguration("settle_frames"), value_type=int),
        "phase_cooldown_ticks": ParameterValue(LaunchConfiguration("phase_cooldown_ticks"), value_type=int),
        "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
        "action_control_rate_hz": ParameterValue(LaunchConfiguration("action_control_rate_hz"), value_type=float),
        "gesture_hold_confirm_frames": ParameterValue(LaunchConfiguration("gesture_hold_confirm_frames"), value_type=int),
        "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
    }])

    return LaunchDescription(args + [node])
