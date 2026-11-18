#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    simple_ar_node = Node(
        package='microros_v2_opencv_visual_pkg',
        executable='simple_ar_node',
        name='simple_ar_node',
        output='screen'
    )

    return LaunchDescription([
        simple_ar_node
    ])