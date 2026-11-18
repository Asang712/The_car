#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
terminal_client.launch.py
终端交互客户端启动文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成启动描述"""
    
    terminal_client_node = Node(
        package='multi_brains',
        executable='terminal_client',
        name='terminal_client',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        terminal_client_node
    ])
