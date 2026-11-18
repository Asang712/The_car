import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription
from launch_ros.actions import  Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression,PathJoinSubstitution

def generate_launch_description():
    map_folder_dir = os.path.join(os.path.expanduser('~'), 'map')
    bringup_dir=os.path.join(get_package_share_directory('microros_v2_bringup'))
    declared_arguments = []
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_carbase",
            default_value='True',
            description="Whether to launch Carbse is determined by the navigation node, for integration with other launchers",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value='info',
            description="ROS log level: debug, info, warn, error, fatal",
        )
    )

    #cartographer state file
    declared_arguments.append(
        DeclareLaunchArgument(
            "state_filename",
            default_value="map.pbstream",
            description="==============",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "localization_mode",
            default_value="amcl",
            description="localization_mode ,amcl or cartographer",
        )
    )
    
    #namespace
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Add namespace",
        )
    )
    #slam mode
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam",
            default_value="False",
            description="Start SLAM simultaneously with navigation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pose_map",
            default_value="map",
            description="",
        )
    )
    # 初始化变量Initialize Arguments 
    map = LaunchConfiguration("map")
    map_file = PathJoinSubstitution([map_folder_dir, map])
    pose_map=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("pose_map")])
    rviz = LaunchConfiguration("rviz")
    slam = LaunchConfiguration('slam')
    localization_mode=LaunchConfiguration("localization_mode")
    state_filename_file = PathJoinSubstitution([map_folder_dir, LaunchConfiguration("state_filename")])
    namespace=LaunchConfiguration("namespace")
    log_level=LaunchConfiguration('log_level')
    
    nav_rviz_config=os.path.join(bringup_dir,'rviz','nav2.rviz')
    params_file=os.path.join(bringup_dir, "config","nav2_params.yaml")

    #navigation2 startup file
    nav2_launch_file = os.path.join(get_package_share_directory("nav2_bringup"), 
                                    "launch",
                                    'bringup_launch.py')
    
    #The modified startup file of navigation2 blocks the AMCL positioning function
    nav2_launch_mod = os.path.join(bringup_dir,
                                    'launch',
                                    'bringup_mod.launch.py'
                                    )
    
    carbase=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','car_base.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_carbase')),
            launch_arguments={'log_level': 'info',
                              'use_camera': 'true'
                              }.items())

    #AMCL positioning method navigation
    amcl_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", slam, "' == 'False' "])),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    "map": map_file,
                    "params_file": params_file,
                    "use_sim_time": "False",
                    "namespace": namespace,
                    "log_level": log_level,
                }.items(),
            )
        ]
    )

    #Cartographer positioning navigation
    cartographer_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'cartographer' and '", slam, "' == 'False' "])),
        actions=[
            #Start navigation2 navigation file (no global positioning)
            PushRosNamespace(namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_mod),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time": "False",
                                "namespace": namespace,
                                "log_level": log_level,
                                }.items()),
            Node(
                package =   'cartographer_ros',
                executable = 'cartographer_node',
                parameters = [{'use_sim_time': False}],
                arguments = [
                            '-configuration_directory', os.path.join(get_package_share_directory('microros_v2_bringup'),'config'),
                            '-configuration_basename', 'cartographer_2d_localization.lua',
                            '-load_state_filename', state_filename_file],
                remappings = [('imu', '/imu/data_raw')],
                output = 'screen'
                ),
            Node(
                package = 'cartographer_ros',
                executable = 'cartographer_occupancy_grid_node',
                parameters = [
                    {'use_sim_time': False},
                    {'resolution': 0.05}],
                )
        ]
    )

    slam_toolbox_localization_nav = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'slam_toolbox' and '", slam, "' == 'False' "])),
        actions=[
            #Start navigation2 navigation file (no global positioning)
            PushRosNamespace(namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_mod),
                launch_arguments={
                                "map": map_file,
                                "params_file": params_file,
                                "use_sim_time": "False",
                                "namespace": namespace,
                                "log_level": log_level,
                                }.items()),
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                  os.path.join(bringup_dir,'config','mapper_params_localization.yaml'),
                  {'use_sim_time': False},
                  {'map_file_name': pose_map}
                ],
                arguments=['--ros-args','--log-level','warn'],
                output='screen')
        ]
    )


    #rviz
    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name='rviz2',
        arguments=['-d', nav_rviz_config,
                   '--ros-args', '--log-level', log_level],
        output='screen',
        condition=IfCondition(PythonExpression(["'", rviz, "' == 'True' "]))
    )

    #Navigate while building the map
    nav_and_slam = GroupAction(
        condition=IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl' and '", slam, "' == 'True' "])),
        actions=[
            # Navigation using AMCL positioning method
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": "False",
                    "params_file": params_file,
                    'slam': 'True',
                    "namespace": namespace,
                    "log_level": log_level,
                }.items(),
            )
        ]
    )

    return LaunchDescription(
        [
            *declared_arguments,                
            rviz_node,                          
            carbase,                           
            amcl_localization_nav,              
            cartographer_localization_nav,
            slam_toolbox_localization_nav,         
            nav_and_slam                      
        ]
    )


