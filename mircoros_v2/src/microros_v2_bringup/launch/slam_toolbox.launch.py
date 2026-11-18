import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,LogInfo,TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        "rviz",
        default_value='true',
        description="Start Rviz2 automatically ?"
        )
    )
    
    slam_params_file = os.path.join(get_package_share_directory('microros_v2_bringup'), 'config', 'mapper_params_online_async.yaml')
    
    bringup_car_base=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
            launch_arguments={
                              'use_camera': 'False',
                              }.items())
        
    rviz=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('microros_v2_bringup'), 'rviz', 'slam_toolbox_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    return LaunchDescription([
        *declared_arguments,
        bringup_car_base,
        start_async_slam_toolbox_node,
        rviz
    ])
