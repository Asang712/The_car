import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,GroupAction,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml, ReplaceString


def navigation_setup(context, *args, **kwargs):
    # per robot; all nodes publish to the global /tf, frames are prefixed 'robotN/'
    robot_names = [n.strip().strip('/') for n in
                   LaunchConfiguration('robot_names').perform(context).split(',') if n.strip()]
    map_file = os.path.join(os.path.expanduser('~'), 'map',
                            LaunchConfiguration('map').perform(context))
    bringup_dir = get_package_share_directory('microros_v2_bringup')
    params_template = os.path.join(bringup_dir,'config','nav2_multi_robot_params.yaml')

    # single global map server shared by all robots
    actions = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': False}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server']}],
        ),
    ]

    lifecycle_nodes = ['amcl', 'controller_server', 'planner_server',
                       'behavior_server', 'bt_navigator', 'waypoint_follower']
    for name in robot_names:
        # rewrite '<robot_ns>' TF-frame placeholders, then wrap the file under the
        # namespace root key so params match /robotN/xxx node names
        params_file = ParameterFile(
            RewrittenYaml(
                source_file=ReplaceString(
                    source_file=params_template,
                    replacements={'<robot_ns>': name}),
                root_key=name,
                param_rewrites={},
                convert_types=True),
            allow_substs=True)

        # NOTE: no /tf remapping here, all robots share the global TF tree
        actions += [
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace=name,
                output='screen',
                parameters=[params_file],
                # humble amcl has no map_topic param, remap to the shared global map
                remappings=[('map', '/map')],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                namespace=name,
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                namespace=name,
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                namespace=name,
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                namespace=name,
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                namespace=name,
                output='screen',
                parameters=[params_file],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                namespace=name,
                output='screen',
                parameters=[{'autostart': True, 'node_names': lifecycle_nodes}],
            ),
        ]
    return actions


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "demo",
            default_value="keyboard",
            description="use keyboard control or joystick control (keyboard/joystick/navigation)",
        )
    ) 
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
            default_value='ptz,ptz',
            description='Camera type per robot (ptz | no_ptz | basic_car): single value for all robots, '
                        'or comma-separated list matching robot_names, e.g. ptz,basic_car'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'map',
            default_value='map.yaml',
            description='Map name under ~/map, shared by all robots'
        )
    )
    
    demo=LaunchConfiguration('demo')
    bringup_dir = get_package_share_directory('microros_v2_bringup')
    common_params = os.path.join(bringup_dir,'config','common.yaml')
    multi_robot_base=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','multi_robot_base.launch.py')),
        launch_arguments={'robot_names': LaunchConfiguration('robot_names'),
                            'camera_types': LaunchConfiguration('camera_types')
                            }.items())
    
    keyboard=Node(
        package='common_demo',
        executable='keyboard_control',
        name='keyboard_control',
        parameters=[common_params],
        output='screen',
        prefix='gnome-terminal --title="Keyboard UI" --',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('demo'), "' == 'keyboard'"])),
    )

    joystick=GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('demo'), "' == 'joystick'"])),
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
            ),
        ],
    )

    navigation=GroupAction(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('demo'), "' == 'navigation'"])),
        actions=[
            OpaqueFunction(function=navigation_setup),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                output='screen',
                arguments=['-d', os.path.join(bringup_dir, 'rviz', 'nav2_multi_robot.rviz')],
            ),
        ],
    )



    return LaunchDescription([
        *declared_arguments,
        multi_robot_base,
        keyboard,
        joystick,
        navigation
    ])


