#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detect_apriltag.launch.py

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Current OpenCV visual package name.
VISUAL_PACKAGE_NAME = "microros_v2_opencv_visual_pkg"

# AprilTag detection executable name.
DETECT_APRILTAG_EXECUTABLE_NAME = "detect_apriltag_node"

# AprilTag detection runtime node name.
DETECT_APRILTAG_NODE_NAME = "detect_apriltag_node"

# Default camera calibration file path (package share directory).
DEFAULT_CAMERA_YAML_PATH = os.path.join(
    get_package_share_directory(VISUAL_PACKAGE_NAME),
    "config",
    "camera.yaml",
)


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description.
    """
    launch_arguments = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/image_raw",
            description="输入原始图像话题 / Input raw image topic.",
        ),
        DeclareLaunchArgument(
            "use_compressed_image",
            default_value="false",
            description="是否使用 sensor_msgs/msg/CompressedImage / Whether to use CompressedImage input.",
        ),
        DeclareLaunchArgument(
            "window_name",
            default_value="detect_apriltag",
            description="OpenCV 显示窗口名称 / OpenCV display window name.",
        ),
        DeclareLaunchArgument(
            "show_debug_window",
            default_value="true",
            description="是否显示 OpenCV 窗口 / Whether to show OpenCV window.",
        ),
        DeclareLaunchArgument(
            "window_width",
            default_value="960",
            description="OpenCV 窗口宽度 / OpenCV window width.",
        ),
        DeclareLaunchArgument(
            "window_height",
            default_value="720",
            description="OpenCV 窗口高度 / OpenCV window height.",
        ),
        DeclareLaunchArgument(
            "publish_debug_image",
            default_value="false",
            description="是否发布调试图像 / Whether to publish debug image.",
        ),
        DeclareLaunchArgument(
            "debug_image_topic",
            default_value="/detect_apriltag/debug_image",
            description="调试图像话题 / Debug image topic.",
        ),
        DeclareLaunchArgument(
            "image_width",
            default_value="320",
            description="处理图像宽度，默认匹配 camera.yaml 标定宽度 / Processing image width.",
        ),
        DeclareLaunchArgument(
            "image_height",
            default_value="240",
            description="处理图像高度，默认匹配 camera.yaml 标定高度 / Processing image height.",
        ),
        DeclareLaunchArgument(
            "overlay_font_scale",
            default_value="0.24",
            description="叠加文字大小 / Overlay font scale.",
        ),
        DeclareLaunchArgument(
            "overlay_line_height",
            default_value="14",
            description="叠加文字行高 / Overlay line height.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_x",
            default_value="8",
            description="叠加文字左边距 / Overlay left margin.",
        ),
        DeclareLaunchArgument(
            "overlay_margin_y",
            default_value="16",
            description="叠加文字上边距 / Overlay top margin.",
        ),
        DeclareLaunchArgument(
            "overlay_show_background",
            default_value="false",
            description="是否显示叠加文字背景 / Whether to show overlay text background.",
        ),
        DeclareLaunchArgument(
            "enable_horizontal_flip",
            default_value="false",
            description="是否水平镜像图像 / Whether to horizontally flip image.",
        ),
        DeclareLaunchArgument(
            "enable_vertical_flip",
            default_value="false",
            description="是否垂直翻转图像 / Whether to vertically flip image.",
        ),
        DeclareLaunchArgument(
            "rotate_image_code",
            default_value="none",
            description="图像旋转: none/90_clockwise/90_counterclockwise/180 / Image rotation code.",
        ),
        DeclareLaunchArgument(
            "tag_family",
            default_value="tag36h11",
            description="AprilTag 家族，例如 tag36h11/tag25h9/tag16h5 / AprilTag family.",
        ),
        DeclareLaunchArgument(
            "detector_nthreads",
            default_value="4",
            description="AprilTag 检测线程数 / AprilTag detector thread count.",
        ),
        DeclareLaunchArgument(
            "quad_decimate",
            default_value="2.0",
            description="AprilTag quad_decimate 参数，越大越快但越粗 / AprilTag quad_decimate.",
        ),
        DeclareLaunchArgument(
            "quad_sigma",
            default_value="0.0",
            description="AprilTag quad_sigma 参数 / AprilTag quad_sigma.",
        ),
        DeclareLaunchArgument(
            "refine_edges",
            default_value="true",
            description="是否细化边缘 / Whether to refine edges.",
        ),
        DeclareLaunchArgument(
            "decode_sharpening",
            default_value="0.25",
            description="AprilTag decode_sharpening 参数 / AprilTag decode_sharpening.",
        ),
        DeclareLaunchArgument(
            "detector_debug",
            default_value="false",
            description="是否启用检测器 debug / Whether to enable detector debug.",
        ),
        DeclareLaunchArgument(
            "estimate_tag_pose",
            default_value="true",
            description="是否估计 AprilTag 3D 位姿 / Whether to estimate tag 3D pose.",
        ),
        DeclareLaunchArgument(
            "tag_size",
            default_value="0.025",
            description="AprilTag 实际边长，单位米 / Tag size in meters.",
        ),
        DeclareLaunchArgument(
            "use_camera_yaml",
            default_value="true",
            description="是否从 camera_yaml_path 读取相机内参 / Whether to load intrinsics from camera_yaml_path.",
        ),
        DeclareLaunchArgument(
            "camera_yaml_path",
            default_value=DEFAULT_CAMERA_YAML_PATH,
            description="ros-humble-camera-calibration 生成的 camera.yaml 路径 / Camera calibration yaml path.",
        ),
        DeclareLaunchArgument(
            "use_undistort",
            default_value="false",
            description="是否使用 camera.yaml 畸变参数先去畸变 / Whether to undistort image before detection.",
        ),
        DeclareLaunchArgument(
            "camera_fx",
            default_value="0.0",
            description="手动相机 fx，不使用 camera.yaml 时有效 / Manual camera fx.",
        ),
        DeclareLaunchArgument(
            "camera_fy",
            default_value="0.0",
            description="手动相机 fy，不使用 camera.yaml 时有效 / Manual camera fy.",
        ),
        DeclareLaunchArgument(
            "camera_cx",
            default_value="0.0",
            description="手动相机 cx，不使用 camera.yaml 时有效 / Manual camera cx.",
        ),
        DeclareLaunchArgument(
            "camera_cy",
            default_value="0.0",
            description="手动相机 cy，不使用 camera.yaml 时有效 / Manual camera cy.",
        ),
        DeclareLaunchArgument(
            "draw_tag_axes",
            default_value="true",
            description="是否在画面上绘制 AprilTag 局部坐标系 / Whether to draw local tag coordinate axes.",
        ),
        DeclareLaunchArgument(
            "draw_axes_for_all_tags",
            default_value="true",
            description="是否给所有识别到的 Tag 绘制坐标系；false 时只绘制主 Tag / Draw axes for all tags or only main tag.",
        ),
        DeclareLaunchArgument(
            "tag_axis_length",
            default_value="0.0",
            description="局部坐标轴长度，单位米；0 表示自动使用 tag_size*0.60 / Axis length in meters; 0 means tag_size*0.60.",
        ),
        DeclareLaunchArgument(
            "tag_axis_thickness",
            default_value="2",
            description="局部坐标轴线宽 / Local coordinate axis line thickness.",
        ),
        DeclareLaunchArgument(
            "axis_label_font_scale",
            default_value="0.35",
            description="X/Y/Z 坐标轴标签字体大小 / X/Y/Z axis label font scale.",
        ),
        DeclareLaunchArgument(
            "draw_camera_axes",
            default_value="true",
            description="是否用相机坐标系替代画面中心十字 / Whether to draw camera coordinate axes instead of center cross.",
        ),
        DeclareLaunchArgument(
            "camera_axis_pixel_length",
            default_value="38",
            description="相机坐标系 X/Y 轴在画面中的像素长度 / Camera X/Y axis length in pixels.",
        ),
        DeclareLaunchArgument(
            "camera_axis_thickness",
            default_value="2",
            description="相机坐标系线宽 / Camera coordinate axis line thickness.",
        ),
        DeclareLaunchArgument(
            "camera_axis_label_font_scale",
            default_value="0.38",
            description="相机坐标系标签字体大小 / Camera coordinate axis label font scale.",
        ),
        DeclareLaunchArgument(
            "tag_select_strategy",
            default_value="smallest_id",
            description="主 tag 选择策略: smallest_id/largest/nearest_center / Main tag select strategy.",
        ),
        DeclareLaunchArgument(
            "print_detection_log",
            default_value="true",
            description="是否打印识别日志 / Whether to print detection log.",
        ),
        DeclareLaunchArgument(
            "detection_log_interval_sec",
            default_value="1.0",
            description="识别日志打印间隔 / Detection log interval.",
        ),
        DeclareLaunchArgument(
            "use_header_stamp_for_fps",
            default_value="true",
            description="是否使用 header 时间戳计算 FPS / Whether to use header stamp for FPS.",
        ),
        DeclareLaunchArgument(
            "show_image_stamp_delay",
            default_value="true",
            description="是否显示图像延迟 / Whether to show image stamp delay.",
        ),
        DeclareLaunchArgument(
            "enable_rqt_dynamic_parameters",
            default_value="true",
            description="是否允许 rqt/ros2 param 动态调参 / Whether to enable runtime parameter tuning.",
        ),
    ]

    detect_apriltag_node = Node(
        package=VISUAL_PACKAGE_NAME,
        executable=DETECT_APRILTAG_EXECUTABLE_NAME,
        name=DETECT_APRILTAG_NODE_NAME,
        output="screen",
        parameters=[{
            "image_topic": LaunchConfiguration("image_topic"),
            "use_compressed_image": ParameterValue(LaunchConfiguration("use_compressed_image"), value_type=bool),
            "window_name": LaunchConfiguration("window_name"),
            "show_debug_window": ParameterValue(LaunchConfiguration("show_debug_window"), value_type=bool),
            "window_width": ParameterValue(LaunchConfiguration("window_width"), value_type=int),
            "window_height": ParameterValue(LaunchConfiguration("window_height"), value_type=int),
            "publish_debug_image": ParameterValue(LaunchConfiguration("publish_debug_image"), value_type=bool),
            "debug_image_topic": LaunchConfiguration("debug_image_topic"),
            "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
            "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
            "overlay_font_scale": ParameterValue(LaunchConfiguration("overlay_font_scale"), value_type=float),
            "overlay_line_height": ParameterValue(LaunchConfiguration("overlay_line_height"), value_type=int),
            "overlay_margin_x": ParameterValue(LaunchConfiguration("overlay_margin_x"), value_type=int),
            "overlay_margin_y": ParameterValue(LaunchConfiguration("overlay_margin_y"), value_type=int),
            "overlay_show_background": ParameterValue(LaunchConfiguration("overlay_show_background"), value_type=bool),
            "enable_horizontal_flip": ParameterValue(LaunchConfiguration("enable_horizontal_flip"), value_type=bool),
            "enable_vertical_flip": ParameterValue(LaunchConfiguration("enable_vertical_flip"), value_type=bool),
            "rotate_image_code": LaunchConfiguration("rotate_image_code"),
            "tag_family": LaunchConfiguration("tag_family"),
            "detector_nthreads": ParameterValue(LaunchConfiguration("detector_nthreads"), value_type=int),
            "quad_decimate": ParameterValue(LaunchConfiguration("quad_decimate"), value_type=float),
            "quad_sigma": ParameterValue(LaunchConfiguration("quad_sigma"), value_type=float),
            "refine_edges": ParameterValue(LaunchConfiguration("refine_edges"), value_type=bool),
            "decode_sharpening": ParameterValue(LaunchConfiguration("decode_sharpening"), value_type=float),
            "detector_debug": ParameterValue(LaunchConfiguration("detector_debug"), value_type=bool),
            "estimate_tag_pose": ParameterValue(LaunchConfiguration("estimate_tag_pose"), value_type=bool),
            "tag_size": ParameterValue(LaunchConfiguration("tag_size"), value_type=float),
            "use_camera_yaml": ParameterValue(LaunchConfiguration("use_camera_yaml"), value_type=bool),
            "camera_yaml_path": LaunchConfiguration("camera_yaml_path"),
            "use_undistort": ParameterValue(LaunchConfiguration("use_undistort"), value_type=bool),
            "camera_fx": ParameterValue(LaunchConfiguration("camera_fx"), value_type=float),
            "camera_fy": ParameterValue(LaunchConfiguration("camera_fy"), value_type=float),
            "camera_cx": ParameterValue(LaunchConfiguration("camera_cx"), value_type=float),
            "camera_cy": ParameterValue(LaunchConfiguration("camera_cy"), value_type=float),
            "draw_tag_axes": ParameterValue(LaunchConfiguration("draw_tag_axes"), value_type=bool),
            "draw_axes_for_all_tags": ParameterValue(LaunchConfiguration("draw_axes_for_all_tags"), value_type=bool),
            "tag_axis_length": ParameterValue(LaunchConfiguration("tag_axis_length"), value_type=float),
            "tag_axis_thickness": ParameterValue(LaunchConfiguration("tag_axis_thickness"), value_type=int),
            "axis_label_font_scale": ParameterValue(LaunchConfiguration("axis_label_font_scale"), value_type=float),
            "draw_camera_axes": ParameterValue(LaunchConfiguration("draw_camera_axes"), value_type=bool),
            "camera_axis_pixel_length": ParameterValue(LaunchConfiguration("camera_axis_pixel_length"), value_type=int),
            "camera_axis_thickness": ParameterValue(LaunchConfiguration("camera_axis_thickness"), value_type=int),
            "camera_axis_label_font_scale": ParameterValue(LaunchConfiguration("camera_axis_label_font_scale"), value_type=float),
            "tag_select_strategy": LaunchConfiguration("tag_select_strategy"),
            "print_detection_log": ParameterValue(LaunchConfiguration("print_detection_log"), value_type=bool),
            "detection_log_interval_sec": ParameterValue(LaunchConfiguration("detection_log_interval_sec"), value_type=float),
            "use_header_stamp_for_fps": ParameterValue(LaunchConfiguration("use_header_stamp_for_fps"), value_type=bool),
            "show_image_stamp_delay": ParameterValue(LaunchConfiguration("show_image_stamp_delay"), value_type=bool),
            "enable_rqt_dynamic_parameters": ParameterValue(LaunchConfiguration("enable_rqt_dynamic_parameters"), value_type=bool),
        }],
    )

    return LaunchDescription(launch_arguments + [detect_apriltag_node])

