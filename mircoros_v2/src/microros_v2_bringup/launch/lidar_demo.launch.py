import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration,PythonExpression

def generate_launch_description():

    # 声明变量Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "demo",
            default_value="laser_Avoidance",
            description="",
        )
    ) 

    demo=LaunchConfiguration('demo')
    bringup_dir = get_package_share_directory('microros_v2_bringup')
    common_params = os.path.join(bringup_dir,'config','common.yaml')
    carbase=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch','car_base.launch.py')),
        launch_arguments={'log_level': 'info',
                            'use_camera': 'false'
                            }.items())
    
    #雷达避障
    laser_Avoidance=Node(
        package='common_demo',
        executable='laser_Avoidance',
        name='laser_Avoidance',
        output='screen',
        condition=IfCondition(PythonExpression(["'", demo, "' == 'laser_Avoidance'" ])),
        parameters=[common_params],

    )
    #雷达跟踪
    laser_Tracker=Node(
        package='common_demo',
        executable='laser_Tracker',
        name='laser_Tracker',
        output='screen',
        condition=IfCondition(PythonExpression(["'", demo, "' == 'laser_Tracker'" ])),
        parameters=[common_params],
    )
    #雷达警卫
    laser_Warning=Node(
        package='common_demo',
        executable='laser_Warning',
        name='laser_Warning',
        output='screen',
        parameters=[common_params],
        condition=IfCondition(PythonExpression(["'", demo, "' == 'laser_Warning'" ]))
    )

    #雷达巡逻
    patrol=GroupAction(
        condition=IfCondition(PythonExpression(["'", demo, "' == 'laser_patrol'" ])),
        actions=[
            Node(
                package='common_demo',
                executable='laser_patrol',
                name='laser_patrol',
                output='screen',
            ),
            Node(
                package='rqt_reconfigure',
                executable='rqt_reconfigure',
                name='rqt_reconfigure',
                output='screen'
            )
        ],
    )

    return LaunchDescription([
        *declared_arguments,
        carbase,
        patrol,
        laser_Avoidance,
        laser_Tracker,
        laser_Warning
    ])


