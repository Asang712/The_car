#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch standalone bbox follower."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "microros_v2_kcf_follow_pkg"
EXECUTABLE_NAME = "tracking_bbox_follow_node"
NODE_NAME = "tracking_bbox_follow_node"


def lc(name: str) -> LaunchConfiguration:
    return LaunchConfiguration(name)


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("tracking_bbox_topic", default_value="/tracking_bbox"),
        DeclareLaunchArgument("tracking_bbox_msg_type", default_value="auto"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("ptz_command_topic", default_value="/cmd_ptz_angle"),
        DeclareLaunchArgument("joy_state_topic", default_value="/JoyState"),
        DeclareLaunchArgument("enable_follow", default_value="true"),
        DeclareLaunchArgument("enable_ptz_follow", default_value="true"),
        DeclareLaunchArgument("enable_body_turn_follow", default_value="true"),
        DeclareLaunchArgument("enable_distance_follow", default_value="true"),
        DeclareLaunchArgument("enable_joy_override", default_value="true"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("tracking_confidence_threshold", default_value="0.1"),
        DeclareLaunchArgument("target_stale_timeout_sec", default_value="1.2"),
        DeclareLaunchArgument("follow_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("motion_log_interval_sec", default_value="0.0"),
        DeclareLaunchArgument("horizontal_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("horizontal_max_deg", default_value="90.0"),
        DeclareLaunchArgument("pitch_min_deg", default_value="-90.0"),
        DeclareLaunchArgument("pitch_max_deg", default_value="20.0"),
        DeclareLaunchArgument("initial_horizontal_deg", default_value="0.0"),
        DeclareLaunchArgument("initial_pitch_deg", default_value="-45.0"),
        DeclareLaunchArgument("publish_initial_ptz_command", default_value="true"),
        DeclareLaunchArgument("initial_publish_count", default_value="20"),
        DeclareLaunchArgument("initial_publish_interval_sec", default_value="0.2"),
        DeclareLaunchArgument("pixel_tolerance", default_value="15.0"),
        DeclareLaunchArgument("kp_x", default_value="0.6"),
        DeclareLaunchArgument("kd_x", default_value="0.15"),
        DeclareLaunchArgument("kp_y", default_value="0.6"),
        DeclareLaunchArgument("kd_y", default_value="0.15"),
        DeclareLaunchArgument("max_step_xy", default_value="5.0"),
        DeclareLaunchArgument("sign_x", default_value="1.0"),
        DeclareLaunchArgument("sign_y", default_value="-1.0"),
        DeclareLaunchArgument("body_kp", default_value="0.03"),
        DeclareLaunchArgument("body_kd", default_value="0.0"),
        DeclareLaunchArgument("body_tolerance_deg", default_value="3.0"),
        DeclareLaunchArgument("servo1_engage_deg", default_value="5.0"),
        DeclareLaunchArgument("max_angular_z", default_value="0.6"),
        DeclareLaunchArgument("body_dir", default_value="-1.0"),
        DeclareLaunchArgument("body_ff_gain", default_value="1.0"),
        DeclareLaunchArgument("fixed_camera_pixel_tolerance", default_value="15.0"),
        DeclareLaunchArgument("fixed_camera_engage_px", default_value="30.0"),
        DeclareLaunchArgument("fixed_camera_body_kp", default_value="0.6"),
        DeclareLaunchArgument("fixed_camera_body_kd", default_value="0.15"),
        DeclareLaunchArgument("fixed_camera_body_dir", default_value="-1.0"),
        DeclareLaunchArgument("target_bbox_size_px", default_value="0.0"),
        DeclareLaunchArgument("auto_set_target_bbox_size", default_value="true"),
        DeclareLaunchArgument("distance_tolerance_ratio", default_value="0.06"),
        DeclareLaunchArgument("distance_kp", default_value="0.18"),
        DeclareLaunchArgument("distance_ki", default_value="0.0"),
        DeclareLaunchArgument("distance_kd", default_value="0.02"),
        DeclareLaunchArgument("distance_integral_limit", default_value="0.5"),
        DeclareLaunchArgument("max_linear_x", default_value="0.12"),
        DeclareLaunchArgument("linear_dir", default_value="1.0"),
        DeclareLaunchArgument("print_cmd_vel", default_value="false"),
        DeclareLaunchArgument("print_interval_sec", default_value="0.5"),
        DeclareLaunchArgument("enable_rqt_dynamic_parameters", default_value="true"),
    ]

    params = {
        "tracking_bbox_topic": lc("tracking_bbox_topic"),
        "tracking_bbox_msg_type": lc("tracking_bbox_msg_type"),
        "cmd_vel_topic": lc("cmd_vel_topic"),
        "ptz_command_topic": lc("ptz_command_topic"),
        "joy_state_topic": lc("joy_state_topic"),
        "enable_follow": ParameterValue(lc("enable_follow"), value_type=bool),
        "enable_ptz_follow": ParameterValue(lc("enable_ptz_follow"), value_type=bool),
        "enable_body_turn_follow": ParameterValue(
            lc("enable_body_turn_follow"), value_type=bool
        ),
        "enable_distance_follow": ParameterValue(
            lc("enable_distance_follow"), value_type=bool
        ),
        "enable_joy_override": ParameterValue(lc("enable_joy_override"), value_type=bool),
        "image_width": ParameterValue(lc("image_width"), value_type=int),
        "image_height": ParameterValue(lc("image_height"), value_type=int),
        "tracking_confidence_threshold": ParameterValue(
            lc("tracking_confidence_threshold"), value_type=float
        ),
        "target_stale_timeout_sec": ParameterValue(
            lc("target_stale_timeout_sec"), value_type=float
        ),
        "follow_rate_hz": ParameterValue(lc("follow_rate_hz"), value_type=float),
        "motion_log_interval_sec": ParameterValue(
            lc("motion_log_interval_sec"), value_type=float
        ),
        "horizontal_min_deg": ParameterValue(lc("horizontal_min_deg"), value_type=float),
        "horizontal_max_deg": ParameterValue(lc("horizontal_max_deg"), value_type=float),
        "pitch_min_deg": ParameterValue(lc("pitch_min_deg"), value_type=float),
        "pitch_max_deg": ParameterValue(lc("pitch_max_deg"), value_type=float),
        "initial_horizontal_deg": ParameterValue(
            lc("initial_horizontal_deg"), value_type=float
        ),
        "initial_pitch_deg": ParameterValue(lc("initial_pitch_deg"), value_type=float),
        "publish_initial_ptz_command": ParameterValue(
            lc("publish_initial_ptz_command"), value_type=bool
        ),
        "initial_publish_count": ParameterValue(
            lc("initial_publish_count"), value_type=int
        ),
        "initial_publish_interval_sec": ParameterValue(
            lc("initial_publish_interval_sec"), value_type=float
        ),
        "pixel_tolerance": ParameterValue(lc("pixel_tolerance"), value_type=float),
        "kp_x": ParameterValue(lc("kp_x"), value_type=float),
        "kd_x": ParameterValue(lc("kd_x"), value_type=float),
        "kp_y": ParameterValue(lc("kp_y"), value_type=float),
        "kd_y": ParameterValue(lc("kd_y"), value_type=float),
        "max_step_xy": ParameterValue(lc("max_step_xy"), value_type=float),
        "sign_x": ParameterValue(lc("sign_x"), value_type=float),
        "sign_y": ParameterValue(lc("sign_y"), value_type=float),
        "body_kp": ParameterValue(lc("body_kp"), value_type=float),
        "body_kd": ParameterValue(lc("body_kd"), value_type=float),
        "body_tolerance_deg": ParameterValue(
            lc("body_tolerance_deg"), value_type=float
        ),
        "servo1_engage_deg": ParameterValue(lc("servo1_engage_deg"), value_type=float),
        "max_angular_z": ParameterValue(lc("max_angular_z"), value_type=float),
        "body_dir": ParameterValue(lc("body_dir"), value_type=float),
        "body_ff_gain": ParameterValue(lc("body_ff_gain"), value_type=float),
        "fixed_camera_pixel_tolerance": ParameterValue(
            lc("fixed_camera_pixel_tolerance"), value_type=float
        ),
        "fixed_camera_engage_px": ParameterValue(
            lc("fixed_camera_engage_px"), value_type=float
        ),
        "fixed_camera_body_kp": ParameterValue(
            lc("fixed_camera_body_kp"), value_type=float
        ),
        "fixed_camera_body_kd": ParameterValue(
            lc("fixed_camera_body_kd"), value_type=float
        ),
        "fixed_camera_body_dir": ParameterValue(
            lc("fixed_camera_body_dir"), value_type=float
        ),
        "target_bbox_size_px": ParameterValue(
            lc("target_bbox_size_px"), value_type=float
        ),
        "auto_set_target_bbox_size": ParameterValue(
            lc("auto_set_target_bbox_size"), value_type=bool
        ),
        "distance_tolerance_ratio": ParameterValue(
            lc("distance_tolerance_ratio"), value_type=float
        ),
        "distance_kp": ParameterValue(lc("distance_kp"), value_type=float),
        "distance_ki": ParameterValue(lc("distance_ki"), value_type=float),
        "distance_kd": ParameterValue(lc("distance_kd"), value_type=float),
        "distance_integral_limit": ParameterValue(
            lc("distance_integral_limit"), value_type=float
        ),
        "max_linear_x": ParameterValue(lc("max_linear_x"), value_type=float),
        "linear_dir": ParameterValue(lc("linear_dir"), value_type=float),
        "print_cmd_vel": ParameterValue(lc("print_cmd_vel"), value_type=bool),
        "print_interval_sec": ParameterValue(lc("print_interval_sec"), value_type=float),
        "enable_rqt_dynamic_parameters": ParameterValue(
            lc("enable_rqt_dynamic_parameters"), value_type=bool
        ),
    }

    node = Node(
        package=PACKAGE_NAME,
        executable=EXECUTABLE_NAME,
        name=NODE_NAME,
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(args + [node])
