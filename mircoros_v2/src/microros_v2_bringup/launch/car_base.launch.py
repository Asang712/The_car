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
    #bring camera and select robot  model 
    camera_type=GetRobotType()    
    # ptz | no_ptz | basic_car
    robot_urdf=os.path.join(get_package_share_directory('robot_description'),'urdf','microros_v2.urdf.xacro')
    robot_description = Command(
        ["xacro", " ", robot_urdf," ", "camera_type:=", camera_type] )
    # Declare arguments
    declared_arguments = []
    # Sensor control arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            description='Whether to launch camera'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'micro_ros_agent_verbose',
            default_value='3',
            description='micro-ROS agent verbosity level (0-6)'
        )
    )

    # camera_url format: http://<camera_ip>:81/stream
    common_params = os.path.join(get_package_share_directory('microros_v2_bringup'),'config','common.yaml')
    fastdds_profile = os.path.join(get_package_share_directory('microros_v2_bringup'),'config','fastdds_shm.xml')

    camrea_drive=Node(
        package='microros_v2_bringup',
        executable='camera_driver',
        output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': fastdds_profile},
        parameters=[{
        'camera_url': GetCameraUrl(),
        'img_save_path':os.path.join(os.path.expanduser('~'),'microros_ws','cache','image.png'),
        'ost_file': os.path.join(get_package_share_directory('microros_v2_bringup'),'config','ost.yaml')}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_camera'), "' == 'true' and '", camera_type, "' != 'basic_car'"]))
        )

    mircoros_agent=Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8090', '-v', LaunchConfiguration('micro_ros_agent_verbose')],
    )

    base_driver=Node(
        package='microros_v2_bringup',
        executable='base_driver',
        name='base_driver',
        parameters=[common_params, 
        {'camera_type': camera_type}],
        output='screen',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        arguments=['--ros-args','--log-level','warn']
    )
    joint_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'source_list': ["ptz_status"]}],
        arguments=['--ros-args','--log-level','warn']
    )
    #ekf_odom
    ekf_odom=Node(
        package='robot_localization',
        executable='ekf_node',
        name="ekf_filter_node",
        parameters=[os.path.join(get_package_share_directory('microros_v2_bringup'),'config','ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')]
    )

    return LaunchDescription([
        *declared_arguments,
        mircoros_agent,
        camrea_drive,
        robot_state_publisher_node,
        joint_publisher_node,
        base_driver,
        ekf_odom,
    ])
