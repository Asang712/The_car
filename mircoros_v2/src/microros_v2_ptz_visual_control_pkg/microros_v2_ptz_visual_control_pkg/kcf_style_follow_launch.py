"""Launch arguments shared by KCF-style visual followers."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


BOOL_PARAMETERS = (
    "enable_follow",
    "enable_ptz_follow",
    "enable_body_turn_follow",
    "enable_distance_follow",
    "enable_joy_override",
    "publish_initial_ptz_command",
    "auto_set_target_bbox_size",
    "print_cmd_vel",
    "enable_rqt_dynamic_parameters",
)

INT_PARAMETERS = ("image_width", "image_height", "initial_publish_count")

FLOAT_PARAMETERS = (
    "target_stale_timeout_sec",
    "follow_rate_hz",
    "motion_log_interval_sec",
    "horizontal_min_deg",
    "horizontal_max_deg",
    "pitch_min_deg",
    "pitch_max_deg",
    "initial_horizontal_deg",
    "initial_pitch_deg",
    "initial_publish_interval_sec",
    "pixel_tolerance",
    "kp_x",
    "kd_x",
    "kp_y",
    "kd_y",
    "max_step_xy",
    "sign_x",
    "sign_y",
    "body_kp",
    "body_kd",
    "body_tolerance_deg",
    "servo1_engage_deg",
    "max_angular_z",
    "body_dir",
    "body_ff_gain",
    "fixed_camera_pixel_tolerance",
    "fixed_camera_engage_px",
    "fixed_camera_body_kp",
    "fixed_camera_body_kd",
    "fixed_camera_body_dir",
    "target_bbox_size_px",
    "distance_tolerance_ratio",
    "distance_kp",
    "distance_ki",
    "distance_kd",
    "distance_integral_limit",
    "max_linear_x",
    "linear_dir",
    "print_interval_sec",
)

DEFAULTS = {
    "joy_state_topic": "/JoyState",
    "enable_follow": "true",
    "enable_ptz_follow": "true",
    "enable_body_turn_follow": "true",
    "enable_distance_follow": "true",
    "enable_joy_override": "true",
    "image_width": "640",
    "image_height": "480",
    "target_stale_timeout_sec": "1.2",
    "follow_rate_hz": "10.0",
    "motion_log_interval_sec": "0.0",
    "horizontal_min_deg": "-90.0",
    "horizontal_max_deg": "90.0",
    "pitch_min_deg": "-90.0",
    "pitch_max_deg": "20.0",
    "initial_horizontal_deg": "0.0",
    "initial_pitch_deg": "-45.0",
    "publish_initial_ptz_command": "true",
    "initial_publish_count": "20",
    "initial_publish_interval_sec": "0.2",
    "pixel_tolerance": "15.0",
    "kp_x": "0.6",
    "kd_x": "0.15",
    "kp_y": "0.45",
    "kd_y": "0.15",
    "max_step_xy": "5.0",
    "sign_x": "1.0",
    "sign_y": "-1.0",
    "body_kp": "0.03",
    "body_kd": "0.0",
    "body_tolerance_deg": "3.0",
    "servo1_engage_deg": "5.0",
    "max_angular_z": "0.6",
    "body_dir": "-1.0",
    "body_ff_gain": "1.0",
    "fixed_camera_pixel_tolerance": "15.0",
    "fixed_camera_engage_px": "30.0",
    "fixed_camera_body_kp": "0.6",
    "fixed_camera_body_kd": "0.15",
    "fixed_camera_body_dir": "-1.0",
    "target_bbox_size_px": "0.0",
    "auto_set_target_bbox_size": "true",
    "distance_tolerance_ratio": "0.06",
    "distance_kp": "0.18",
    "distance_ki": "0.0",
    "distance_kd": "0.02",
    "distance_integral_limit": "0.5",
    "max_linear_x": "0.12",
    "linear_dir": "1.0",
    "print_cmd_vel": "false",
    "print_interval_sec": "0.5",
    "enable_rqt_dynamic_parameters": "true",
}


def control_launch_arguments():
    return [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in DEFAULTS.items()
    ]


def control_launch_parameters():
    parameters = {
        "joy_state_topic": LaunchConfiguration("joy_state_topic"),
    }
    for name in BOOL_PARAMETERS:
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=bool)
    for name in INT_PARAMETERS:
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=int)
    for name in FLOAT_PARAMETERS:
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    return parameters
