import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    map_folder_dir=os.path.join(os.path.expanduser('~'), 'map')
    bringup_dir = get_package_share_directory('microros_v2_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'roadnet_nav.yaml')
    location_params_file = os.path.join(bringup_dir,'config','mapper_params_localization.yaml')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    graph_filepath = PathJoinSubstitution([map_folder_dir, LaunchConfiguration("graph")])
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    log_level = LaunchConfiguration('log_level')
    map_yaml_file=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("map")])
    pose_map=PathJoinSubstitution([map_folder_dir, LaunchConfiguration("pose_map")])

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'map_server'
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    param_substitutions = {'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='True', description='if rviz is used, set to True. If False, rviz will not be launched'
    )
    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='map.geojson', 
        description='geojson map file name'
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value="map.yaml",
        description='map name'
    )

    declare_pose_map_cmd = DeclareLaunchArgument(
        'pose_map',
        default_value="map",
        description='poase map name'
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value="roadnet",
        description='poase map name'
    )

    declare_start_carbase_cmd = DeclareLaunchArgument(
        'start_carbase',
        default_value="True",
        description='Whether to launch Carbse is determined by the navigation node, for integration with other launchers'
    )

    car_base_launch=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('microros_v2_bringup'),'launch','car_base.launch.py')),
                condition=IfCondition(LaunchConfiguration('start_carbase')),
                launch_arguments={'log_level': 'info',
                                    'use_camera': 'False'
                                  }.items())

    nav2_container=Node(
            namespace=LaunchConfiguration('namespace'),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("microros_v2_bringup"), "rviz", "roadnet.rviz"),
                       '--ros-args', '--log-level', log_level],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz'))
        )

    slam_toolbox_localization = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
          location_params_file,
          {'use_sim_time': False},
          {'map_file_name': pose_map}
        ],
        arguments=['--ros-args','--log-level',log_level],
        output='screen')

    load_composable_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', False),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='roadnet',
                        plugin='roadnet::RouteServer',
                        name='route_server',
                        parameters=[configured_params, {'graph_filepath': graph_filepath}],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[
                            configured_params,
                            {'yaml_filename': map_yaml_file},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                    )
                ],
            ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_pose_map_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_start_carbase_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(car_base_launch)
    ld.add_action(nav2_container)
    ld.add_action(load_composable_nodes)
    ld.add_action(slam_toolbox_localization)
    ld.add_action(rviz_node)

    return ld
