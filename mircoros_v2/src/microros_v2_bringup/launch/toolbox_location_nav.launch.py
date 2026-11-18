import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,LogInfo,RegisterEventHandler,OpaqueFunction
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace,ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression,LocalSubstitution,PathJoinSubstitution
def generate_launch_description():
    # Map folder, file detection
    map_folder_dir = os.path.join(os.path.expanduser('~'), 'map')

    # Declare launch arguments
    declared_arguments = []
    bringup_dir=os.path.join(get_package_share_directory('microros_v2_bringup'))
    location_params_file = os.path.join(bringup_dir,'config','mapper_params_localization.yaml')

    #map file
    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value="map.yaml",
            description="map name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='True',
            description="if rviz is used, set to True. If False, rviz will not be launched.",
        )
    )
    # slam_toolbox pose map file
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_map",
            default_value="map",
            description="",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'detection_enabled',
            default_value='false'
        )
    )

    # slam_toolbox navigation parameters file
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="log level"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="normal",
            description="rviz config file"
        )
    )
    
    # Initialize Arguments 
    map_file=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("map")])
    pose_map=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("pose_map")])

    params_file=os.path.join(get_package_share_directory("microros_v2_bringup"), "config", "nav2_params.yaml")
    use_sim_time=LaunchConfiguration("use_sim_time",default="false")
    log_level=LaunchConfiguration("log_level")

    carbase_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
                launch_arguments={'log_level': log_level,
                                    'use_camera': 'False'
                                  }.items())

    #The modified startup file of navigation2 blocks the AMCL positioning function
    nav2_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','bringup_mod.launch.py')),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time":use_sim_time,
                                "log_level": log_level,
                                }.items())
    
    #slam_toolbox Localization Node
    slam_toolbox_localization_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
          location_params_file,
          {'use_sim_time': False},
          {'map_file_name': pose_map}
        ],
        output='screen')

    def create_rviz_node(context, *args, **kwargs):
        rviz_config_value = LaunchConfiguration('rviz_config').perform(context)
        if rviz_config_value == 'roadnet_edit':
            rviz_config_file = os.path.join(bringup_dir, 'rviz', 'roadnet_edit.rviz')
        else:
            rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2.rviz')
        
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )
        return [rviz_node]

    return LaunchDescription(
        [
            *declared_arguments,                 
            OpaqueFunction(function=create_rviz_node),
            carbase_launch,
            slam_toolbox_localization_slam_toolbox_node,
            nav2_launch
                 
        ]
    )


