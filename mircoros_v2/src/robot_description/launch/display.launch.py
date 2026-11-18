from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_type = LaunchConfiguration('camera_type')

    pkg_share = FindPackageShare('robot_description')

    xacro_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'microros_v2.urdf.xacro'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'microros_v2.rviz'
    ])

    robot_description_content = Command([
        'xacro ',
        xacro_file,
        ' camera_type:=', camera_type
    ])

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'camera_type',
            default_value='ptz',
            description='Camera type: ptz | no_ptz | basic_car'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])