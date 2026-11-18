import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("yolo_onnx_follow_pkg")
    default_config = os.path.join(
        package_share,
        "config",
        "yolo_onnx_follow_config.yaml",
    )

    config_file = LaunchConfiguration("config_file")
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    target_class = LaunchConfiguration("target_class")
    target_id = LaunchConfiguration("target_id")

    venv_pythonpath = os.path.join(
        os.path.expanduser("~"),
        "microros_ws",
        "python_env",
        "mircoros_v2_env",
        ".venv",
        "lib",
        "python3.10",
        "site-packages",
    )
    original_pythonpath = os.environ.get("PYTHONPATH", "")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to yolo_onnx_follow_config.yaml",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value="",
                description="Path to YOLOv11 ONNX model",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="Camera image topic",
            ),
            DeclareLaunchArgument(
                "target_class",
                default_value="",
                description="Optional class name/id to auto follow, e.g. person or 0",
            ),
            DeclareLaunchArgument(
                "target_id",
                default_value="-1",
                description="Optional local track id to follow",
            ),
            Node(
                package="yolo_onnx_follow_pkg",
                executable="yolo_onnx_follow_node",
                name="yolo_onnx_follow_node",
                output="screen",
                parameters=[
                    {
                        "config_file": config_file,
                        "model_path": model_path,
                        "image_topic": image_topic,
                        "target_class": target_class,
                        "target_id": ParameterValue(target_id, value_type=int),
                    }
                ],
                additional_env={
                    "PYTHONPATH": venv_pythonpath + os.pathsep + original_pythonpath
                },
            ),
        ]
    )
