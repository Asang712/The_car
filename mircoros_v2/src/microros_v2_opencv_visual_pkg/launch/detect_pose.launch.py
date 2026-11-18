#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    detect_pose_node = Node(
        package='microros_v2_opencv_visual_pkg',
        executable='detect_pose_node',
        name='openpose_detection_node',
        output='screen'
    )

    return LaunchDescription([
        detect_pose_node
    ])