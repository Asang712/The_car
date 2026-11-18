import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType, GetRobotInfo
def generate_launch_description():
    #find yolo model according to different machine
    camera_type=GetRobotType()  
    bringup_dir=os.path.join(get_package_share_directory('microros_v2_bringup'))
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_nav",
            default_value='false',
            description="",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_track",
            default_value='false',
            description="",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "nav_mode",
            default_value='nav2',
            description="nav2 or roadnet",
        )
    )


    # Car Base Driver (with camera)
    carbase=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','car_base.launch.py')),
            launch_arguments={'log_level': 'info',
                              'use_camera': 'true'
                              }.items())

    # Navigation-nav2
    nav2=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','nav2.launch.py')),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_nav'), "' == 'true' and '", LaunchConfiguration('nav_mode'), "' == 'nav2'"])),
            launch_arguments={'log_level': 'warn',
                              'start_carbase': 'false',
                              'localization_mode':'slam_toolbox'
                              }.items())

    #Navigation-roadnet
    roadnet_nav=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','roadnet_nav.launch.py')),
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_nav'), "' == 'true' and '", LaunchConfiguration('nav_mode'), "' == 'roadnet'"])),
            launch_arguments={'log_level': 'warn',
                              'start_carbase': 'false'
                              }.items())

    # MCP Service
    commom_param_file=os.path.join(get_package_share_directory('microros_v2_bringup'),'config','common.yaml')
    venv_pythonpath = os.path.join(os.path.expanduser('~'), 'microros_ws', 'python_env','mircoros_v2_env','.venv', 'lib', 'python3.10', 'site-packages')
    original_pythonpath = os.environ.get('PYTHONPATH', '')
    mcp_service = Node(
        package="multi_brains_pre",
        executable="mcp_service",
        name="mcp_service",
        parameters=[commom_param_file, 
        {'nav_mode': LaunchConfiguration('nav_mode'),
        'use_nav': LaunchConfiguration('use_nav'),
        'verify_image_path': os.path.join(os.path.expanduser('~'),'microros_ws','cache','verify_image.png'),
        'grid_images_path': os.path.join(os.path.expanduser('~'),'microros_ws','cache','grid_images.png'),
        'Vision_API_KEY': GetRobotInfo('Vision_API_KEY') or '',
        'Vision_BASE_URL': GetRobotInfo('Vision_BASE_URL') or '',
        'Vision_MODEL': GetRobotInfo('Vision_MODEL') or '',
        }],
        additional_env={
            'PYTHONPATH': venv_pythonpath + os.pathsep + original_pythonpath
        },
        emulate_tty=True,
        output='screen'
    )

    # tracker node
    super_tracker = Node(
        package='smart_tracker',
        executable='super_tracker',
        name='super_tracker',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_track'), "' == 'true' and '", camera_type, "' != 'basic_car'"])),
        parameters=[commom_param_file,
        {
            'display_windows_only_track': True,
            'display_windows': False,
            'model_path': os.path.join(os.path.expanduser('~'),'MODELS','tracker','super_track.onnx'),
            "inference_backend" : "cpu"
        }],
        output='screen',
    )
    visual_track = Node(
        package='microros_v2_bringup',
        executable='visual_track',
        name='visual_track',
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_track'), "' == 'true' and '", camera_type, "' != 'basic_car'"])),
        parameters=[commom_param_file,
        {'camera_type': camera_type}],
        output='screen',
    )

    return LaunchDescription([
        *declared_arguments,
        carbase,
        mcp_service,
        nav2,
        roadnet_nav,
        super_tracker,
        visual_track
    ])