import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType,GetCameraUrl

def generate_launch_description():


    carbase=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
            launch_arguments={'log_level': 'info',
                              'use_camera': 'true',
                              'micro_ros_agent_verbose': '4'
                              }.items())


    topic_test_node = Node(
        package='microros_v2_bringup',
        executable='topic_test',
        name='topic_test_node',
        output='screen',
        parameters=[{
            'imu_topic': '/imu',
            'scan_topic': '/scan',
        }])

    return LaunchDescription([
        carbase,
        topic_test_node
    ])
