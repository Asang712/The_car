#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
color_follow_ptz_dify.launch.py

Dify 专用颜色跟随节点启动文件 - PTZ 版本
- 接受颜色名称参数
- 从配置文件读取 HSV 值
- 无 GUI，后台运行
- 用于 Dify 工作流自动化场景
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([
        # 颜色选择参数（必需）
        DeclareLaunchArgument(
            'color',
            default_value='red',
            description='跟随颜色 (red/green/blue/yellow)'
        ),

        # HSV 配置文件
        DeclareLaunchArgument(
            'hsv_config_file',
            default_value='~/microros_car_v2_ws/config/color_hsv.txt',
            description='HSV 颜色配置文件路径'
        ),

        # 图像话题
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='原始图像话题'
        ),

        # 云台控制话题
        DeclareLaunchArgument(
            'ptz_command_topic',
            default_value='/cmd_ptz_angle',
            description='云台控制话题'
        ),

        # 车身控制话题
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='车身速度控制话题'
        ),

        # 图像尺寸
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='图像宽度'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='图像高度'
        ),

        # 云台角度限制
        DeclareLaunchArgument(
            'horizontal_min_deg',
            default_value='-90.0',
            description='水平最小角度'
        ),
        DeclareLaunchArgument(
            'horizontal_max_deg',
            default_value='90.0',
            description='水平最大角度'
        ),
        DeclareLaunchArgument(
            'pitch_min_deg',
            default_value='-90.0',
            description='俯仰最小角度'
        ),
        DeclareLaunchArgument(
            'pitch_max_deg',
            default_value='20.0',
            description='俯仰最大角度'
        ),

        # 初始角度
        DeclareLaunchArgument(
            'initial_horizontal_deg',
            default_value='0.0',
            description='初始水平角度'
        ),
        DeclareLaunchArgument(
            'initial_pitch_deg',
            default_value='-45.0',
            description='初始俯仰角度'
        ),

        # PTZ PD 控制参数
        DeclareLaunchArgument(
            'pd_kp',
            default_value='0.7',
            description='PTZ PD 控制比例系数'
        ),
        DeclareLaunchArgument(
            'pd_kd',
            default_value='0.25',
            description='PTZ PD 控制微分系数'
        ),
        DeclareLaunchArgument(
            'deadband_x',
            default_value='20.0',
            description='X 轴死区（像素）'
        ),
        DeclareLaunchArgument(
            'deadband_y',
            default_value='20.0',
            description='Y 轴死区（像素）'
        ),

        # 车身控制参数
        DeclareLaunchArgument(
            'angular_kp',
            default_value='0.006',
            description='车身角速度 PD 比例系数'
        ),
        DeclareLaunchArgument(
            'angular_kd',
            default_value='0.0',
            description='车身角速度 PD 微分系数'
        ),
        DeclareLaunchArgument(
            'angular_tolerance_deg',
            default_value='5.0',
            description='车身角度容差（度）'
        ),
        DeclareLaunchArgument(
            'max_angular_z',
            default_value='0.06',
            description='最大角速度（rad/s）'
        ),
        DeclareLaunchArgument(
            'body_share',
            default_value='0.2',
            description='车身跟随云台分担比例'
        ),

        # 检测参数
        DeclareLaunchArgument(
            'min_target_radius',
            default_value='10.0',
            description='最小目标半径（像素）'
        ),
        DeclareLaunchArgument(
            'min_target_area',
            default_value='500.0',
            description='最小目标面积（像素²）'
        ),
        DeclareLaunchArgument(
            'morph_kernel_size',
            default_value='5',
            description='形态学核大小'
        ),
        DeclareLaunchArgument(
            'lost_frame_threshold',
            default_value='5',
            description='目标丢失帧数阈值'
        ),

        # 控制频率
        DeclareLaunchArgument(
            'ptz_control_rate',
            default_value='20.0',
            description='PTZ 控制频率（Hz）'
        ),
        DeclareLaunchArgument(
            'body_control_rate',
            default_value='10.0',
            description='车身控制频率（Hz）'
        ),

        # 颜色跟随节点
        Node(
            package='microros_v2_ptz_visual_control_pkg',
            executable='color_follow_ptz_dify_node',
            name='color_follow_ptz_dify_node',
            output='screen',
            parameters=[{
                'color': LaunchConfiguration('color'),
                'hsv_config_file': LaunchConfiguration('hsv_config_file'),
                'image_topic': LaunchConfiguration('image_topic'),
                'ptz_command_topic': LaunchConfiguration('ptz_command_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'horizontal_min_deg': LaunchConfiguration('horizontal_min_deg'),
                'horizontal_max_deg': LaunchConfiguration('horizontal_max_deg'),
                'pitch_min_deg': LaunchConfiguration('pitch_min_deg'),
                'pitch_max_deg': LaunchConfiguration('pitch_max_deg'),
                'initial_horizontal_deg': LaunchConfiguration('initial_horizontal_deg'),
                'initial_pitch_deg': LaunchConfiguration('initial_pitch_deg'),
                'reverse_horizontal_direction': False,
                'reverse_pitch_direction': True,
                'pd_kp': LaunchConfiguration('pd_kp'),
                'pd_kd': LaunchConfiguration('pd_kd'),
                'deadband_x': LaunchConfiguration('deadband_x'),
                'deadband_y': LaunchConfiguration('deadband_y'),
                'ptz_step_smooth_alpha': 0.35,
                'max_ptz_step': 1.2,
                'ptz_smooth_alpha': 0.85,
                'angular_kp': LaunchConfiguration('angular_kp'),
                'angular_kd': LaunchConfiguration('angular_kd'),
                'angular_tolerance_deg': LaunchConfiguration('angular_tolerance_deg'),
                'max_angular_z': LaunchConfiguration('max_angular_z'),
                'body_share': LaunchConfiguration('body_share'),
                'reverse_body_direction': False,
                'min_target_radius': LaunchConfiguration('min_target_radius'),
                'min_target_area': LaunchConfiguration('min_target_area'),
                'morph_kernel_size': LaunchConfiguration('morph_kernel_size'),
                'lost_frame_threshold': LaunchConfiguration('lost_frame_threshold'),
                'ptz_control_rate': LaunchConfiguration('ptz_control_rate'),
                'body_control_rate': LaunchConfiguration('body_control_rate'),
                'publish_initial_ptz_command': True,
                'initial_publish_count': 20,
                'initial_publish_interval_sec': 0.2,
            }],
            additional_env={
                "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
            },
        ),
    ])
