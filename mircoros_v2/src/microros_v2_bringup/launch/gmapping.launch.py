import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
def generate_launch_description():

    slam_gmapping_config_file = os.path.join(get_package_share_directory("microros_v2_bringup"), "config", "slam_gmapping.yaml")

    rviz_cmd=DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Start Rviz2 automatically ?",
    )

    bringup_car_base=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("microros_v2_bringup"),'launch','car_base.launch.py')),
        launch_arguments={'log_level': 'info',
                            'use_camera': 'false'
                            }.items())

    gmapping_node=Node(
    package='slam_gmapping', 
    executable='slam_gmapping', 
    output='screen', 
    parameters=[slam_gmapping_config_file]
    )
    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('microros_v2_bringup'), 'rviz', 'gmapping_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
            rviz_cmd,                       
            rviz_node,                       
            bringup_car_base,                 
            gmapping_node,                 
    ])


