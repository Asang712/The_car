from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'microros_v2_no_ptz_visual_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhujunpeng',
    maintainer_email='2474250941@qq.com',
    description='MicroROS V2 No-PTZ Visual Control Package — fixed camera, body-rotation-based tracking',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 控制类（已完成）
            'qr_control_node = microros_v2_no_ptz_visual_control_pkg.qr_control_node:main',
            'follow_line_node = microros_v2_no_ptz_visual_control_pkg.follow_line_node:main',
            'gesture_control_node = microros_v2_no_ptz_visual_control_pkg.gesture_control_node:main',
            'gesture_action_group_control_node = microros_v2_no_ptz_visual_control_pkg.gesture_action_group_control_node:main',
            # 跟随类（已完成）
            'color_follow_node = microros_v2_no_ptz_visual_control_pkg.color_follow_node:main',
            'color_follow_dify_node = microros_v2_no_ptz_visual_control_pkg.color_follow_dify_node:main',
            'face_follow_node = microros_v2_no_ptz_visual_control_pkg.face_follow_node:main',
            'april_tag_follow_node = microros_v2_no_ptz_visual_control_pkg.april_tag_follow_node:main',
            'qr_follow_node = microros_v2_no_ptz_visual_control_pkg.qr_follow_node:main',
            'gesture_follow_node = microros_v2_no_ptz_visual_control_pkg.gesture_follow_node:main',
            'pose_follow_node = microros_v2_no_ptz_visual_control_pkg.pose_follow_node:main',
            'hand_follow_node = microros_v2_no_ptz_visual_control_pkg.hand_follow_node:main',
            # 跟踪类
            'color_tracker_node = microros_v2_no_ptz_visual_control_pkg.color_tracker_node:main',
            'face_tracker_node = microros_v2_no_ptz_visual_control_pkg.face_tracker_node:main',
            'april_tag_tracker_node = microros_v2_no_ptz_visual_control_pkg.april_tag_tracker_node:main',
            'qr_tracker_node = microros_v2_no_ptz_visual_control_pkg.qr_tracker_node:main',
            'gesture_tracker_node = microros_v2_no_ptz_visual_control_pkg.gesture_tracker_node:main',
            'pose_tracker_node = microros_v2_no_ptz_visual_control_pkg.pose_tracker_node:main',
            'hand_tracker_node = microros_v2_no_ptz_visual_control_pkg.hand_tracker_node:main',
            'kcf_tracker_node = microros_v2_no_ptz_visual_control_pkg.kcf_tracker_node:main',
            'kcf_tracker_dify_node = microros_v2_no_ptz_visual_control_pkg.kcf_tracker_dify_node:main',
            'colorSelect = microros_v2_no_ptz_visual_control_pkg.colorSelect:main',
            'apriltag_action_group_control_node = microros_v2_no_ptz_visual_control_pkg.apriltag_action_group_control_node:main',
        ],
    },
)
