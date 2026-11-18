import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration,PythonExpression
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,LogInfo,GroupAction
from launch_ros.parameter_descriptions import ParameterValue
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType

def is_node_running(node_name: str) -> bool:
    try:
        result = subprocess.run(['ros2', 'node', 'list'],
                                capture_output=True, text=True, timeout=10)
        return f'/{node_name}' in result.stdout.split()
    except Exception:
        return False

def is_process_running(pattern: str) -> bool:
    try:
        result = subprocess.run(['pgrep', '-f', pattern], capture_output=True)
        return result.returncode == 0
    except Exception:
        return False

def generate_launch_description():
    common_params = os.path.join(get_package_share_directory('microros_v2_bringup'),'config','common.yaml')
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'control_mode',
            default_value='keyboard',
            description='use keyboard control or joystick control (keyboard/joystick)'
        )
    )
    mircoros_agent=Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '--port', '8090', '-v', '3'],
    )

    keyboard=Node(
        package='common_demo',
        executable='keyboard_control',
        name='keyboard_control',
        parameters=[common_params],
        output='screen',
        prefix='gnome-terminal --title="Keyboard UI" --',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'keyboard'"])),
    )

    joystick=GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('control_mode'), "' == 'joystick'"])),
        actions=[
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
            ),
            Node(
                package='common_demo',
                executable='joy_control_node',
                name='joy_control',
                parameters=[common_params],
                output='screen',
                # prefix='gnome-terminal --title="Joystick UI" --',
            ),
        ],
    )

    return LaunchDescription([
        *declared_arguments,
        LogInfo(msg='micro_ros_agent already running, skip.') if is_process_running('micro_ros_agent udp4') else mircoros_agent,
        keyboard,
        joystick,
        
    ])
