
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    # Car Base Driver (with camera)
    carbase=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
            launch_arguments={'log_level': 'info',
                              'use_camera': 'true'
                              }.items())

    colorSelect = Node(
            package='multi_brains_pre',
            executable='colorSelect',
            name='colorSelect',
            output='screen',
            parameters=[{'hsv_text_file': os.path.join(get_package_share_directory('microros_v2_bringup'), 'config', 'HSV.text')}],
        )

    # Create launch description
    return LaunchDescription([
        carbase,
        colorSelect
])