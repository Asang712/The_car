from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'microros_v2_ptz_visual_control_pkg'

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
    maintainer='yahboom',
    maintainer_email='2474250941@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.color_tracker_ptz_node:main',
            'face_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.face_tracker_ptz_node:main',
            'object_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.object_tracker_ptz_node:main',
            'qr_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.qr_tracker_ptz_node:main',
            'gesture_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.gesture_tracker_ptz_node:main',
            'pose_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.pose_tracker_ptz_node:main',
            'april_tag_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.april_tag_tracker_ptz_node:main',
            'kcf_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.kcf_tracker_ptz_node:main',
            'kcf_tracker_ptz_dify_node = microros_v2_ptz_visual_control_pkg.kcf_tracker_ptz_dify_node:main',
            'color_follow_ptz_node = microros_v2_ptz_visual_control_pkg.color_follow_ptz_node:main',
            'color_follow_ptz_dify_node = microros_v2_ptz_visual_control_pkg.color_follow_ptz_dify_node:main',
            'face_follow_ptz_node = microros_v2_ptz_visual_control_pkg.face_follow_ptz_node:main',
            'object_follow_ptz_node = microros_v2_ptz_visual_control_pkg.object_follow_ptz_node:main',
            'april_tag_follow_ptz_node = microros_v2_ptz_visual_control_pkg.april_tag_follow_ptz_node:main',
            'qr_follow_ptz_node = microros_v2_ptz_visual_control_pkg.qr_follow_ptz_node:main',
            'qr_control_ptz_node = microros_v2_ptz_visual_control_pkg.qr_control_ptz_node:main',
            'follow_line_ptz_node = microros_v2_ptz_visual_control_pkg.follow_line_ptz_node:main',
            'gesture_follow_ptz_node = microros_v2_ptz_visual_control_pkg.gesture_follow_ptz_node:main',
            'gesture_control_ptz_node = microros_v2_ptz_visual_control_pkg.gesture_control_ptz_node:main',
            'hand_tracker_ptz_node = microros_v2_ptz_visual_control_pkg.hand_tracker_ptz_node:main',
            'hand_follow_ptz_node = microros_v2_ptz_visual_control_pkg.hand_follow_ptz_node:main',
            'gesture_action_group_control_ptz_node = microros_v2_ptz_visual_control_pkg.gesture_action_group_control_ptz_node:main',
            'pose_follow_ptz_node = microros_v2_ptz_visual_control_pkg.pose_follow_ptz_node:main',
            'colorSelect = microros_v2_ptz_visual_control_pkg.colorSelect:main',
            'apriltag_action_group_control_ptz_node = microros_v2_ptz_visual_control_pkg.apriltag_action_group_control_ptz_node:main',
        ],
    },
)
