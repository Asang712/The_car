
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():        

    commom_param_file=os.path.join(get_package_share_directory('microros_v2_bringup'),'config','common.yaml')
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "display_windows_only_track",
            default_value='false',
            description="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "display_windows",
            default_value='true',
            description="",
        )
    )
    
    # tracker node
    tracker_node = Node(
        package='smart_tracker',
        executable='super_tracker',
        name='super_tracker',
        parameters=[commom_param_file,
        {
            'display_windows_only_track': LaunchConfiguration('display_windows_only_track'),
            'display_windows': LaunchConfiguration('display_windows'),
            'model_path': os.path.join(os.path.expanduser('~'),'MODELS','tracker','super_track.onnx'),
            "inference_backend" : "cpu"
        }],
        output='screen',
        # prefix='gdb -ex run --args'
    )

    # Create launch description
    return LaunchDescription([
        *declared_arguments,
        tracker_node
])