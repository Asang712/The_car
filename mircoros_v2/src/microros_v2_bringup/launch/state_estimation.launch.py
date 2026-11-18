
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('microros_v2_bringup'),'rviz','state_estimation.rviz')
    carbase=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
            launch_arguments={'log_level': 'info',
                              'use_camera': 'false'
                              }.items())

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name='rviz2',
        arguments=['-d', rviz_config,
                   '--ros-args', '--log-level', 'info'],
        output='screen'
    )

    # Create launch description
    return LaunchDescription([
        carbase,
        rviz_node
])