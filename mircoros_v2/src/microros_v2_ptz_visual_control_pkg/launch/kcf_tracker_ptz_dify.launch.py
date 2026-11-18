#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kcf_tracker_ptz_dify.launch.py

Dify 专用 KCF 追踪节点启动文件 - PTZ 版本
- 通过 /start_track 服务传入 ROI，保留启动参数兼容入口
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
        # ROI 坐标参数（兼容旧启动方式；新流程通过 /start_track 服务传入）
        DeclareLaunchArgument(
            'auto_init_roi_x1',
            default_value='-1',
            description='ROI 左上角 X 坐标（像素）'
        ),
        DeclareLaunchArgument(
            'auto_init_roi_y1',
            default_value='-1',
            description='ROI 左上角 Y 坐标（像素）'
        ),
        DeclareLaunchArgument(
            'auto_init_roi_x2',
            default_value='-1',
            description='ROI 右下角 X 坐标（像素）'
        ),
        DeclareLaunchArgument(
            'auto_init_roi_y2',
            default_value='-1',
            description='ROI 右下角 Y 坐标（像素）'
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
        DeclareLaunchArgument(
            'tracking_bbox_topic',
            default_value='/kcf_tracking_bbox',
            description='KCF 每帧目标框反馈话题'
        ),
        DeclareLaunchArgument(
            'enable_internal_control',
            default_value='false',
            description='是否由 KCF 节点内部直接控制云台'
        ),

        # 图像尺寸
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='KCF/Dify ROI 统一坐标系宽度'
        ),
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='KCF/Dify ROI 统一坐标系高度'
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

        # PD 控制参数
        DeclareLaunchArgument(
            'pd_kp',
            default_value='0.7',
            description='PD 控制比例系数'
        ),
        DeclareLaunchArgument(
            'pd_kd',
            default_value='0.25',
            description='PD 控制微分系数'
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

        # 控制频率
        DeclareLaunchArgument(
            'control_rate_hz',
            default_value='20.0',
            description='控制频率（Hz）'
        ),
        DeclareLaunchArgument(
            'image_processing_rate_hz',
            default_value='15.0',
            description='图像处理频率（Hz）'
        ),

        # 追踪器类型
        DeclareLaunchArgument(
            'tracker_type',
            default_value='KCF',
            description='追踪器类型 (KCF/CSRT/MOSSE)'
        ),

        # KCF 追踪节点
        Node(
            package='microros_v2_ptz_visual_control_pkg',
            executable='kcf_tracker_ptz_dify_node',
            name='kcf_tracker_ptz_dify_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'ptz_command_topic': LaunchConfiguration('ptz_command_topic'),
                'tracking_bbox_topic': LaunchConfiguration('tracking_bbox_topic'),
                'enable_internal_control': LaunchConfiguration('enable_internal_control'),
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
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'image_processing_rate_hz': LaunchConfiguration('image_processing_rate_hz'),
                'publish_initial_ptz_command': True,
                'initial_publish_count': 20,
                'initial_publish_interval_sec': 0.2,
                'tracker_type': LaunchConfiguration('tracker_type'),
                'auto_init_roi_x1': LaunchConfiguration('auto_init_roi_x1'),
                'auto_init_roi_y1': LaunchConfiguration('auto_init_roi_y1'),
                'auto_init_roi_x2': LaunchConfiguration('auto_init_roi_x2'),
                'auto_init_roi_y2': LaunchConfiguration('auto_init_roi_y2'),
            }],
            additional_env={
                "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
            },
        ),
    ])
