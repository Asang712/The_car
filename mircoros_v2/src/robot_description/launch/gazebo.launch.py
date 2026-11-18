import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    camera_type = LaunchConfiguration('camera_type')

    pkg_name = 'microros_v2_description_pkg'

    pkg_share_path = get_package_share_directory(pkg_name)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Gazebo 看到 model://microros_v2_description_pkg/meshes/xxx.STL 时，
    # 会在 GAZEBO_MODEL_PATH 下寻找 microros_v2_description_pkg 目录。
    # 因此这里要设置到 share 的上一级：
    # .../install/microros_v2_description_pkg/share
    # gazebo_model_path = os.path.dirname(pkg_share_path)
    gazebo_model_path = os.pathsep.join([
        os.path.dirname(pkg_share_path),
        # os.environ.get('GAZEBO_MODEL_PATH', '')
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'urdf',
        'microros_v2.urdf.xacro'
    ])

    default_world = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'worlds',
        'empty.world'
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

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'true'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_microros_v2',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'microros_v2',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.10'
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=''
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock'
        ),

        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world file'
        ),

        DeclareLaunchArgument(
            'camera_type',
            default_value='ptz',
            description='Camera type: ptz | no_ptz | basic_car'
        ),

        gazebo_launch,
        robot_state_publisher,
        spawn_entity
    ])