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
    configuration_directory = os.path.join(get_package_share_directory('microros_v2_bringup'),'config')
    # lua file
    configuration_basename = LaunchConfiguration('configuration_basename', default='cartographer_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.5')

    #Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
        "rviz",
        default_value='true',
        description="Start Rviz2 automatically ?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period')
    )

    bringup_car_base=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("microros_v2_bringup"),'launch','car_base.launch.py')),
        launch_arguments={'log_level': 'info',
                            'use_camera': 'false'
                            }.items())

    cartographer = GroupAction(
        actions=[
            Node(
                    package='cartographer_ros',
                    executable='cartographer_node',
                    name='cartographer_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=['-configuration_directory', configuration_directory,
                                '-configuration_basename', configuration_basename]),
            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
                arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
        ]
    )

    rviz_node=Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('microros_v2_bringup'), 'rviz', 'slam_cartographer_rviz.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        *declared_arguments,
        bringup_car_base,
        rviz_node,
        cartographer
    ])

