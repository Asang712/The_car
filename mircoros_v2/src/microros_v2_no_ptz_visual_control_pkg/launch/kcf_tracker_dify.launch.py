#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kcf_tracker_dify.launch.py

Dify 专用 KCF 追踪节点启动文件 - No-PTZ 版本（车身控制）
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

        # 车身控制话题
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='车身速度控制话题'
        ),
        DeclareLaunchArgument(
            'tracking_bbox_topic',
            default_value='/kcf_tracking_bbox',
            description='KCF 每帧目标框反馈话题'
        ),
        DeclareLaunchArgument(
            'enable_internal_control',
            default_value='false',
            description='是否由 KCF 节点内部直接控制车身'
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

        # PD 控制参数
        DeclareLaunchArgument(
            'pd_kp',
            default_value='0.005',
            description='PD 控制比例系数'
        ),
        DeclareLaunchArgument(
            'pd_kd',
            default_value='0.002',
            description='PD 控制微分系数'
        ),
        DeclareLaunchArgument(
            'deadband_x',
            default_value='30.0',
            description='X 轴死区（像素）'
        ),

        # 速度限制
        DeclareLaunchArgument(
            'max_angular_speed',
            default_value='0.8',
            description='最大角速度（rad/s）'
        ),
        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.0',
            description='线速度（m/s，0表示仅转向）'
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
            package='microros_v2_no_ptz_visual_control_pkg',
            executable='kcf_tracker_dify_node',
            name='kcf_tracker_dify_node',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'tracking_bbox_topic': LaunchConfiguration('tracking_bbox_topic'),
                'enable_internal_control': LaunchConfiguration('enable_internal_control'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pd_kp': LaunchConfiguration('pd_kp'),
                'pd_kd': LaunchConfiguration('pd_kd'),
                'deadband_x': LaunchConfiguration('deadband_x'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'image_processing_rate_hz': LaunchConfiguration('image_processing_rate_hz'),
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
