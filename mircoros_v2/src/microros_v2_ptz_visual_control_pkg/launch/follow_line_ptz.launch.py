#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follow_line_ptz.launch.py

中文说明:
    ROS2 Humble 巡线底盘控制节点启动文件。
    本启动文件将原始图像输入、HSV 巡线参数、底盘 PID、避障参数、云台初始角度和调试显示参数全部参数化。
    默认云台 pitch 初始角度为 -90 deg，默认巡线线速度为 0.30 m/s。

English Description:
    Launch file for ROS2 Humble line following control node.
    This launch file parameterizes raw image input, HSV line detection parameters, chassis PID,
    obstacle stop parameters, initial PTZ angle, and debug display options.
    The default initial pitch angle is -90 deg, and the default line following speed is 0.30 m/s.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# VISUAL_CONTROL_PACKAGE_NAME: 视觉控制功能包名称 / Visual control package name.
VISUAL_CONTROL_PACKAGE_NAME = "microros_v2_ptz_visual_control_pkg"

# FOLLOW_LINE_EXECUTABLE_NAME: 巡线节点可执行文件名称 / Follow line node executable name.
FOLLOW_LINE_EXECUTABLE_NAME = "follow_line_ptz_node"

# FOLLOW_LINE_NODE_NAME: 巡线节点运行名称 / Follow line node runtime name.
FOLLOW_LINE_NODE_NAME = "follow_line_ptz_node"


def generate_launch_description() -> LaunchDescription:
    """
    生成 launch 描述 / Generate launch description.

    Returns:
        LaunchDescription: ROS2 launch 描述对象 / ROS2 launch description object.
    """
    # launch_arguments: launch 参数声明列表 / Launch argument declaration list.
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="原始图像输入话题 / Raw image input topic.",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="底盘速度输出话题 / Chassis velocity output topic.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="激光雷达输入话题 / Laser scan input topic.",
        ),
        DeclareLaunchArgument(
            "beep_topic",
            default_value="/beep",
            description="蜂鸣器输出话题 / Buzzer output topic.",
        ),
        DeclareLaunchArgument(
            "joy_state_topic",
            default_value="/JoyState",
            description="手柄接管状态输入话题 / Joystick override state input topic.",
        ),
        DeclareLaunchArgument(
            "ptz_command_topic",
            default_value="/cmd_ptz_angle",
            description="云台角度输出话题 / PTZ angle output topic.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="follow_line_ptz",
            description="OpenCV 调试窗口名称 / OpenCV debug window name.",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true",
            description="是否显示 OpenCV 调试窗口 / Whether to show OpenCV debug window.",
        ),
        DeclareLaunchArgument(
            "window_width",
            default_value="960",
            description="OpenCV 调试窗口宽度 / OpenCV debug window width.",
        ),
        DeclareLaunchArgument(
            "window_height",
            default_value="720",
            description="OpenCV 调试窗口高度 / OpenCV debug window height.",
        ),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="0.38",
            description="画面叠加文字大小 / Overlay text font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="15",
            description="画面叠加文字行高 / Overlay text line height.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_x",
            default_value="6",
            description="画面叠加文字左边距 / Overlay text left margin.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_y",
            default_value="16",
            description="画面叠加文字上边距 / Overlay text top margin.",
        ),
        DeclareLaunchArgument(
            "overlay_show_background",
            default_value="false",
            description="是否显示文字背景框 / Whether to show overlay text background.",
        ),
        DeclareLaunchArgument(
            "show_binary_window",
            default_value="true",
            description="是否显示原图和二值图并排窗口 / Whether to show frame and binary mask side by side.",
        ),
        DeclareLaunchArgument(
            "enable_horizontal_flip",
            default_value="true",
            description="是否水平翻转图像 / Whether to horizontally flip image.",
        ),
        DeclareLaunchArgument(
            "enable_vertical_flip",
            default_value="false",
            description="是否垂直翻转图像 / Whether to vertically flip image.",
        ),
        DeclareLaunchArgument(
            "rotate_image_code",
            default_value="none",
            description="图像旋转: none/90_clockwise/90_counterclockwise/180 / Image rotation code.",
        ),
        DeclareLaunchArgument(
            "Hmin",
            default_value="0",
            description="HSV 色相下限 / HSV hue minimum.",
        ),
        DeclareLaunchArgument(
            "Smin",
            default_value="85",
            description="HSV 饱和度下限 / HSV saturation minimum.",
        ),
        DeclareLaunchArgument(
            "Vmin",
            default_value="126",
            description="HSV 亮度下限 / HSV value minimum.",
        ),
        DeclareLaunchArgument(
            "Hmax",
            default_value="9",
            description="HSV 色相上限 / HSV hue maximum.",
        ),
        DeclareLaunchArgument(
            "Smax",
            default_value="253",
            description="HSV 饱和度上限 / HSV saturation maximum.",
        ),
        DeclareLaunchArgument(
            "Vmax",
            default_value="253",
            description="HSV 亮度上限 / HSV value maximum.",
        ),
        DeclareLaunchArgument(
            "h_expand",
            default_value="8",
            description="鼠标 ROI 自动采样 H 扩展量 / H expansion for mouse ROI sampling.",
        ),
        DeclareLaunchArgument(
            "s_expand",
            default_value="40",
            description="鼠标 ROI 自动采样 S 扩展量 / S expansion for mouse ROI sampling.",
        ),
        DeclareLaunchArgument(
            "v_expand",
            default_value="40",
            description="鼠标 ROI 自动采样 V 扩展量 / V expansion for mouse ROI sampling.",
        ),
        DeclareLaunchArgument(
            "morph_kernel_size",
            default_value="5",
            description="二值图形态学核尺寸 / Binary mask morphology kernel size.",
        ),
        DeclareLaunchArgument(
            "hsv_config_file",
            default_value="",
            description="HSV 参数保存/加载文件路径，留空则不使用 / HSV save/load file path, empty means disabled.",
        ),
        DeclareLaunchArgument(
            "save_hsv_to_file",
            default_value="false",
            description="鼠标采样后是否保存 HSV 到文件 / Whether to save sampled HSV to file.",
        ),
        DeclareLaunchArgument(
            "load_hsv_from_file",
            default_value="false",
            description="启动时是否从文件加载 HSV / Whether to load HSV from file on startup.",
        ),
        DeclareLaunchArgument(
            "line_roi_top_ratio",
            default_value="0.55",
            description="巡线识别 ROI 顶部比例 / Line detection ROI top ratio.",
        ),
        DeclareLaunchArgument(
            "line_roi_bottom_ratio",
            default_value="2.00",
            description="巡线识别 ROI 底部比例 / Line detection ROI bottom ratio.",
        ),
        DeclareLaunchArgument(
            "line_min_area",
            default_value="150.0",
            description="有效路线最小轮廓面积 / Minimum valid line contour area.",
        ),
        DeclareLaunchArgument(
            "line_lost_timeout_sec",
            default_value="1.5",
            description="路线丢失超时时间 / Line lost timeout in seconds.",
        ),
        DeclareLaunchArgument(
            "target_smooth_alpha",
            default_value="0.55",
            description="路线中心滤波系数 / Line center smoothing alpha.",
        ),
        DeclareLaunchArgument(
            "line_pid_kp",
            default_value="22.0",
            description="巡线角速度 PID Kp / Line angular PID Kp.",
        ),
        DeclareLaunchArgument(
            "line_pid_ki",
            default_value="0.0",
            description="巡线角速度 PID Ki / Line angular PID Ki.",
        ),
        DeclareLaunchArgument(
            "line_pid_kd",
            default_value="5.0",
            description="巡线角速度 PID Kd / Line angular PID Kd.",
        ),
        DeclareLaunchArgument(
            "pid_scale",
            default_value="1000.0",
            description="PID 缩放系数 / PID scale.",
        ),
        DeclareLaunchArgument(
            "pid_integral_limit",
            default_value="100.0",
            description="PID 积分限幅 / PID integral clamp limit.",
        ),
        DeclareLaunchArgument(
            "line_error_divisor",
            default_value="16.0",
            description="路线像素误差缩放除数 / Line pixel error divisor.",
        ),
        DeclareLaunchArgument(
            "line_linear_speed",
            default_value="0.15",
            description="巡线默认线速度 m/s / Default line following linear speed in m/s.",
        ),
        DeclareLaunchArgument(
            "line_center_deadband_pixel",
            default_value="40.0",
            description="路线中心死区像素 / Line center deadband in pixels.",
        ),
        DeclareLaunchArgument(
            "max_angular_speed",
            default_value="1.2",
            description="最大角速度 rad/s / Maximum angular speed in rad/s.",
        ),
        DeclareLaunchArgument(
            "reverse_angular_direction",
            default_value="true",
            description="是否反转角速度方向 / Whether to reverse angular direction.",
        ),
        DeclareLaunchArgument(
            "control_rate_hz",
            default_value="20.0",
            description="底盘控制频率 / Chassis control rate.",
        ),
        DeclareLaunchArgument(
            "image_processing_rate_hz",
            default_value="30.0",
            description="图像处理频率 / Image processing rate.",
        ),
        DeclareLaunchArgument(
            "enable_obstacle_stop",
            default_value="true",
            description="是否启用激光避障停车 / Whether to enable obstacle stop from laser scan.",
        ),
        DeclareLaunchArgument(
            "obstacle_min_valid_distance",
            default_value="0.20",
            description="障碍物检测最小有效距离 / Minimum valid obstacle distance.",
        ),
        DeclareLaunchArgument(
            "obstacle_response_distance",
            default_value="0.35",
            description="障碍物停车响应距离 / Obstacle response distance.",
        ),
        DeclareLaunchArgument(
            "obstacle_check_angle_deg",
            default_value="60.0",
            description="障碍物检测前方总角度，60 表示正前方左右各 30 度 / Total front obstacle checking angle.",
        ),
        DeclareLaunchArgument(
            "obstacle_warning_count_threshold",
            default_value="10",
            description="触发停车的障碍点数量阈值 / Obstacle warning count threshold.",
        ),
        DeclareLaunchArgument(
            "enable_beep",
            default_value="true",
            description="障碍物停车时是否蜂鸣 / Whether to beep on obstacle stop.",
        ),
        DeclareLaunchArgument(
            "publish_initial_ptz_command",
            default_value="true",
            description="启动时是否发布初始云台角度 / Whether to publish initial PTZ command.",
        ),
        DeclareLaunchArgument(
            "initial_horizontal_deg",
            default_value="0.0",
            description="初始水平角度 / Initial horizontal angle.",
        ),
        DeclareLaunchArgument(
            "initial_pitch_deg",
            default_value="-90.0",
            description="初始俯仰角度，巡线默认 -90 deg / Initial pitch angle, default -90 deg for line following.",
        ),
        DeclareLaunchArgument(
            "initial_publish_count",
            default_value="20",
            description="初始云台角度重复发布次数 / Initial PTZ repeated publish count.",
        ),
        DeclareLaunchArgument(
            "initial_publish_interval_sec",
            default_value="0.2",
            description="初始云台角度重复发布间隔 / Initial PTZ repeated publish interval.",
        ),
        DeclareLaunchArgument(
            "auto_start_tracking",
            default_value="false",
            description="启动后是否自动开始巡线控制 / Whether to auto-start line following control.",
        ),
        DeclareLaunchArgument(
            "lost_line_stop",
            default_value="true",
            description="路线丢失时是否停车 / Whether to stop when line is lost.",
        ),
        DeclareLaunchArgument(
            "debug_log_interval_sec",
            default_value="2.0",
            description="调试日志打印间隔 / Debug log interval.",
        ),
        DeclareLaunchArgument(
            "print_cmd_vel",
            default_value="false",
            description="是否打印底盘速度命令 / Whether to print cmd_vel.",
        ),
        DeclareLaunchArgument(
            "print_interval_sec",
            default_value="0.5",
            description="速度命令打印间隔 / Cmd_vel print interval.",
        ),
        DeclareLaunchArgument(
            "enable_rqt_dynamic_parameters",
            default_value="true",
            description="是否允许 rqt/ros2 param 动态调参 / Whether to enable rqt/ros2 param dynamic parameters.",
        ),
    ]

    # follow_line_node: 巡线节点 / Follow line node.
    follow_line_node = Node(
        package=VISUAL_CONTROL_PACKAGE_NAME,
        executable=FOLLOW_LINE_EXECUTABLE_NAME,
        name=FOLLOW_LINE_NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "beep_topic": LaunchConfiguration("beep_topic"),
            "joy_state_topic": LaunchConfiguration("joy_state_topic"),
            "ptz_command_topic": LaunchConfiguration("ptz_command_topic"),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_margin_x": ParameterValue(LaunchConfiguration("overlay_margin_x"), value_type=int),
            "overlay_margin_y": ParameterValue(LaunchConfiguration("overlay_margin_y"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "show_binary_window": ParameterValue(LaunchConfiguration("show_binary_window"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "Hmin": ParameterValue(LaunchConfiguration("Hmin"), value_type=int),
            "Smin": ParameterValue(LaunchConfiguration("Smin"), value_type=int),
            "Vmin": ParameterValue(LaunchConfiguration("Vmin"), value_type=int),
            "Hmax": ParameterValue(LaunchConfiguration("Hmax"), value_type=int),
            "Smax": ParameterValue(LaunchConfiguration("Smax"), value_type=int),
            "Vmax": ParameterValue(LaunchConfiguration("Vmax"), value_type=int),
            "h_expand": ParameterValue(LaunchConfiguration("h_expand"), value_type=int),
            "s_expand": ParameterValue(LaunchConfiguration("s_expand"), value_type=int),
            "v_expand": ParameterValue(LaunchConfiguration("v_expand"), value_type=int),
            "morph_kernel_size": ParameterValue(LaunchConfiguration("morph_kernel_size"), value_type=int),
            "hsv_config_file": LaunchConfiguration("hsv_config_file"),
            "save_hsv_to_file": ParameterValue(LaunchConfiguration("save_hsv_to_file"), value_type=bool),
            "load_hsv_from_file": ParameterValue(LaunchConfiguration("load_hsv_from_file"), value_type=bool),
            "line_roi_top_ratio": ParameterValue(LaunchConfiguration("line_roi_top_ratio"), value_type=float),
            "line_roi_bottom_ratio": ParameterValue(LaunchConfiguration("line_roi_bottom_ratio"), value_type=float),
            "line_min_area": ParameterValue(LaunchConfiguration("line_min_area"), value_type=float),
            "line_lost_timeout_sec": ParameterValue(LaunchConfiguration("line_lost_timeout_sec"), value_type=float),
            "target_smooth_alpha": ParameterValue(LaunchConfiguration("target_smooth_alpha"), value_type=float),
            "line_pid_kp": ParameterValue(LaunchConfiguration("line_pid_kp"), value_type=float),
            "line_pid_ki": ParameterValue(LaunchConfiguration("line_pid_ki"), value_type=float),
            "line_pid_kd": ParameterValue(LaunchConfiguration("line_pid_kd"), value_type=float),
            "pid_scale": ParameterValue(LaunchConfiguration("pid_scale"), value_type=float),
            "pid_integral_limit": ParameterValue(LaunchConfiguration("pid_integral_limit"), value_type=float),
            "line_error_divisor": ParameterValue(LaunchConfiguration("line_error_divisor"), value_type=float),
            "line_linear_speed": ParameterValue(LaunchConfiguration("line_linear_speed"), value_type=float),
            "line_center_deadband_pixel": ParameterValue(LaunchConfiguration("line_center_deadband_pixel"), value_type=float),
            "max_angular_speed": ParameterValue(LaunchConfiguration("max_angular_speed"), value_type=float),
            "reverse_angular_direction": ParameterValue(LaunchConfiguration("reverse_angular_direction"), value_type=bool),
            "control_rate_hz": ParameterValue(LaunchConfiguration("control_rate_hz"), value_type=float),
            "image_processing_rate_hz": ParameterValue(LaunchConfiguration("image_processing_rate_hz"), value_type=float),
            "enable_obstacle_stop": ParameterValue(LaunchConfiguration("enable_obstacle_stop"), value_type=bool),
            "obstacle_min_valid_distance": ParameterValue(LaunchConfiguration("obstacle_min_valid_distance"), value_type=float),
            "obstacle_response_distance": ParameterValue(LaunchConfiguration("obstacle_response_distance"), value_type=float),
            "obstacle_check_angle_deg": ParameterValue(LaunchConfiguration("obstacle_check_angle_deg"), value_type=float),
            "obstacle_warning_count_threshold": ParameterValue(LaunchConfiguration("obstacle_warning_count_threshold"), value_type=int),
            "enable_beep": ParameterValue(LaunchConfiguration("enable_beep"), value_type=bool),
            "publish_initial_ptz_command": ParameterValue(LaunchConfiguration("publish_initial_ptz_command"), value_type=bool),
            "initial_horizontal_deg": ParameterValue(LaunchConfiguration("initial_horizontal_deg"), value_type=float),
            "initial_pitch_deg": ParameterValue(LaunchConfiguration("initial_pitch_deg"), value_type=float),
            "initial_publish_count": ParameterValue(LaunchConfiguration("initial_publish_count"), value_type=int),
            "initial_publish_interval_sec": ParameterValue(LaunchConfiguration("initial_publish_interval_sec"), value_type=float),
            "auto_start_tracking": ParameterValue(LaunchConfiguration("auto_start_tracking"), value_type=bool),
            "lost_line_stop": ParameterValue(LaunchConfiguration("lost_line_stop"), value_type=bool),
            "debug_log_interval_sec": ParameterValue(LaunchConfiguration("debug_log_interval_sec"), value_type=float),
            "print_cmd_vel": ParameterValue(LaunchConfiguration("print_cmd_vel"), value_type=bool),
            "print_interval_sec": ParameterValue(LaunchConfiguration("print_interval_sec"), value_type=float),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
    )

    return LaunchDescription(launch_arguments + [follow_line_node])
