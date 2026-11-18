import os
import sys
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.parameter_descriptions import ParameterValue

def is_process_running(pattern: str) -> bool:
    try:
        result = subprocess.run(['pgrep', '-f', pattern], capture_output=True)
        return result.returncode == 0
    except Exception:
        return False


def launch_setup(context, *args, **kwargs):
    # robot name list, e.g. "robot1,robot2"
    robot_names = [n.strip().strip('/') for n in
                   LaunchConfiguration('robot_names').perform(context).split(',') if n.strip()]

    # camera type per robot: ptz | no_ptz | basic_car
    camera_types = [c.strip() for c in
                    LaunchConfiguration('camera_types').perform(context).split(',') if c.strip()]
    if len(camera_types) == 1:
        camera_types = camera_types * len(robot_names)
    if len(camera_types) != len(robot_names):
        raise RuntimeError(
            f'camera_types count ({len(camera_types)}) does not match robot_names count ({len(robot_names)})')

    robot_urdf=os.path.join(get_package_share_directory('robot_description'),'urdf','microros_v2.urdf.xacro')
    ekf_params=os.path.join(get_package_share_directory('microros_v2_bringup'),'config','ekf.yaml')

    # single micro_ros_agent shared by all robots
    mircoros_agent=Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8090', '-v', LaunchConfiguration('micro_ros_agent_verbose')],
    )

    # single command-sync node, fans out cmd_vel/beep/cmd_ptz_angle to every robot
    multi_robot_sync=Node(
        package='common_demo',
        executable='multi_robot_sync',
        name='multi_robot_sync',
        output='screen',
        parameters=[{'robot_names': robot_names}],
    )

    actions = [
        LogInfo(msg='micro_ros_agent already running, skip.') if is_process_running('micro_ros_agent udp4') else mircoros_agent,
        multi_robot_sync,
    ]

    # per-robot nodes: robot_state_publisher / joint_state_publisher / ekf
    for name, camera_type in zip(robot_names, camera_types):
        prefix = f'{name}/'
        robot_description = Command(
            ["xacro", " ", robot_urdf," ", "camera_type:=", camera_type," ", "namespace:=", name] )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
            arguments=['--ros-args','--log-level','warn']
        )
        joint_publisher_node=Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=name,
            parameters=[{'source_list': ["ptz_status"]}],
            arguments=['--ros-args','--log-level','warn']
        )

        #ekf_odom
        ekf_odom=Node(
            package='robot_localization',
            executable='ekf_node',
            name="ekf_filter_node",
            namespace=name,
            parameters=[ekf_params,
            {
                'odom_frame': f'{prefix}odom',
                'base_link_frame': f'{prefix}base_footprint',
                'world_frame': f'{prefix}odom',
                # chassis firmware topics are already namespaced
                'odom0': f'/{prefix}odom_raw',
                'imu0': f'/{prefix}imu',
            }],
            remappings=[('odometry/filtered', 'odom')]
        )

        actions += [
            robot_state_publisher_node,
            joint_publisher_node,
            ekf_odom,
        ]

    return actions


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_names',
            default_value='robot1,robot2',
            description='Comma-separated robot namespaces, e.g. robot1,robot2'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_types',
            default_value='ptz',
            description='Camera type per robot (ptz | no_ptz | basic_car): single value for all robots, '
                        'or comma-separated list matching robot_names, e.g. ptz,basic_car'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'micro_ros_agent_verbose',
            default_value='4',
            description='micro-ROS agent verbosity level (0-6)'
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup),
    ])
