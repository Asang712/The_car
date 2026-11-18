
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType
from launch.substitutions import Command, LaunchConfiguration,PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():
    rviz_config = os.path.join(get_package_share_directory('microros_v2_bringup'),'rviz','display.rviz')
    #bring camera and select robot  model 
    camera_type=GetRobotType()    
    # ptz | no_ptz | basic_car
    robot_urdf=os.path.join(get_package_share_directory('robot_description'),'urdf','microros_v2.urdf.xacro')
    robot_description = Command(
        ["xacro", " ", robot_urdf," ", "camera_type:=", camera_type] )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        arguments=['--ros-args','--log-level','warn']
    )
    joint_publisher_node=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'source_list': ["ptz_status"]}],
        arguments=['--ros-args','--log-level','warn']
    )

    model_display=Node(
        package = 'common_demo',
        executable = 'model_display',
        name='model_display',
        output='screen'
    )
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'info'],
        output='screen'
    )
    mircoros_agent=Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8090', '-v', '4'],
    )
    # Create launch description
    return LaunchDescription([
        mircoros_agent,
        robot_state_publisher_node,
        joint_publisher_node,
        model_display,
        rviz_node
])