#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detect_object_node = Node(
        package='microros_v2_opencv_visual_pkg',
        executable='detect_object_node',
        name='object_detection_node',
        output='screen'
    )

    return LaunchDescription([
        detect_object_node
    ])