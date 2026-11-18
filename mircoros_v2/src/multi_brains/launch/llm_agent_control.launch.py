#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
llm_agent_control.launch.py

中文说明:
    LLM Agent 智能体启动文件。
    启动 llm_agent_node 单节点，提供文字交互 + Dify 意图理解 + 动作执行能力。
    注意：本启动文件不包含相机驱动和导航层，需确保以下前提已满足：
      1. 状态估计层已启动（TF树: odom→base_footprint）
      2. 导航层已启动（Nav2 /navigate_to_pose Action 可用）
      3. 相机已启动（/camera/image_raw/compressed 话题有数据）

English Description:
    Launch file for the LLM Agent node.
    Starts the llm_agent_node providing text interaction + Dify intent understanding + action execution.
    Prerequisites: state estimation layer, navigation layer, and camera must already be running.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取 multi_brains 功能包的 share 目录
    pkg_share_dir = get_package_share_directory('multi_brains')

    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share_dir, 'config', 'llm_agent_config.yaml')

    # 声明启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config_file,
            description='LLM Agent 配置文件路径 / LLM Agent config file path',
        ),
    ]

    config_file = LaunchConfiguration('config_file')

    # LLM Agent 节点
    llm_agent_node = Node(
        package='multi_brains',
        executable='llm_agent_node',
        name='llm_agent_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'config_file': config_file,
        }],
    )

    return LaunchDescription(declared_arguments + [llm_agent_node])
