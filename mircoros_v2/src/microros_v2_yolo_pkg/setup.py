from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'microros_v2_yolo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
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
            "yolo_image_detect_node = microros_v2_yolo_pkg.yolo_image_detect_node:main",
            "yolo_video_detect_node = microros_v2_yolo_pkg.yolo_video_detect_node:main",
            "yolo_realtime_detect_node = microros_v2_yolo_pkg.yolo_realtime_detect_node:main",
            
            'yolo_seg_image_node = microros_v2_yolo_pkg.yolo_seg_image_node:main',
            'yolo_seg_video_node = microros_v2_yolo_pkg.yolo_seg_video_node:main',
            'yolo_seg_realtime_node = microros_v2_yolo_pkg.yolo_seg_realtime_node:main',
            
            'yolo_pose_image_node = microros_v2_yolo_pkg.yolo_pose_image_node:main',
            'yolo_pose_video_node = microros_v2_yolo_pkg.yolo_pose_video_node:main',
            'yolo_pose_realtime_node = microros_v2_yolo_pkg.yolo_pose_realtime_node:main',
            
            'yolo_cls_image_node = microros_v2_yolo_pkg.yolo_cls_image_node:main',
            'yolo_cls_video_node = microros_v2_yolo_pkg.yolo_cls_video_node:main',
            'yolo_cls_realtime_node = microros_v2_yolo_pkg.yolo_cls_realtime_node:main',
            
            'yolo_obb_image_node = microros_v2_yolo_pkg.yolo_obb_image_node:main',
            'yolo_obb_video_node = microros_v2_yolo_pkg.yolo_obb_video_node:main',
            'yolo_obb_realtime_node = microros_v2_yolo_pkg.yolo_obb_realtime_node:main',

            
        ],
    },
)
