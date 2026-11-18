import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import  Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PythonExpression
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType,GetCameraUrl
def generate_launch_description():
    bringup_dir=os.path.join(get_package_share_directory('microros_v2_bringup'))

    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "slam_mode",
            default_value="slam_toolbox",
            description="gmapping、cartographer、slam_toolbox",
        )
    ) 
    declared_arguments.append(
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="False",
            description=" select navigation mode:False、True",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "map_file",
            default_value= 'map.yaml',
            description="Select the map file.",
        )
    ) 
    slam_mode=LaunchConfiguration('slam_mode', default="slam_toolbox")
    navigation_mode=LaunchConfiguration('navigation_mode', default="False")

    bringup_camera=Node(
        package='microros_v2_bringup',
        executable='camera_driver',
        output='screen',
        additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': os.path.join(bringup_dir,'config','fastdds_shm.xml')},
        parameters=[{
            'camera_url': GetCameraUrl(),
            'img_save_path':os.path.join(os.path.expanduser('~'),'microros_ws','cache','image.png'),
        'img_save_path':os.path.join(os.path.expanduser('~'),'microros_ws','cache','image.png'),
        'ost_file': os.path.join(get_package_share_directory('microros_v2_bringup'),'config','ost.yaml')}]
        )

    #gmapping
    gmapping_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                        'launch',
                        'gmapping.launch.py')),
        launch_arguments={
                        "rviz": "False",
                         }.items(),
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'gmapping' and '", navigation_mode, "' == 'False' "]))
    )
    #slam_toolbox
    slam_toolbox_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                        'launch',
                        'slam_toolbox.launch.py')),
        launch_arguments={
                        "rviz": "False",
                         }.items(),
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'slam_toolbox' and '", navigation_mode, "' == 'False' "]))
    )

    #Start Cartographer mapping mode.
    cartographer_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                        'launch',
                        'cartographer.launch.py')),
        launch_arguments={
                        "rviz": "False",
        }.items(),
        condition=IfCondition(PythonExpression(["'", slam_mode, "' == 'cartographer' and '", navigation_mode, "' == 'False' "]))
        
        )

    #Start the rosbridge node.
    bringup_rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    transmission_node = Node(
            package='ros_robot_app',
            executable='transmission',
            name='transmission',
            parameters=[
                {'flip_laser_points': False}
            ],
            output='screen',
    )

    #Robot pose publishing node
    robot_pose_publisher=Node(
        package="ros_robot_app",
        executable="robot_pose_publisher",
        name="robot_pose_publisher",
        output="screen",
        parameters=[
                {"use_sim_time": False},
                {"is_stamped": True},
                {"map_frame": "map"},
                {"base_frame": "base_link"}
        ]
    )
    
    #App map saving service node
    app_save_map_node = Node(
        package='ros_robot_app',
        executable="app_save_map",
        name="app_save_map",
        output="screen",
    )

    app_nav= IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir, 
                                    "launch",
                                    'nav2.launch.py')),
                launch_arguments={
                    "map": LaunchConfiguration("map_file"),
                    "rviz": "False",
                    "localization_mode": "amcl",
                    "slam": "False",
                }.items(),
                condition=IfCondition(PythonExpression(["'", navigation_mode, "' == 'True'" ]))
            )

    return LaunchDescription([
        *declared_arguments,
        bringup_camera,
        app_nav,                                #app导航节点
        bringup_rosbridge_server,               #rosbridge节点
        transmission_node,                      #publish rgb image and laser scan point
        robot_pose_publisher,                   #机器人位姿发布节点
        gmapping_launch,                        #gmapping slam 
        slam_toolbox_launch,                    #slam_toolbox slam
        cartographer_launch,                    #cartographer slam
        app_save_map_node
    ])

