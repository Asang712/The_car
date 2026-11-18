from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'microros_v2_opencv_visual_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'detect_pose_node = microros_v2_opencv_visual_pkg.detect_pose:main',
            'detect_object_node = microros_v2_opencv_visual_pkg.detect_object:main',
            'simple_ar_node = microros_v2_opencv_visual_pkg.simple_ar:main',
            'detect_face_node = microros_v2_opencv_visual_pkg.detect_face:main',
            'detect_color_node = microros_v2_opencv_visual_pkg.detect_color:main',
            'detect_qr_node = microros_v2_opencv_visual_pkg.detect_qr:main',
            'create_qrcode_node = microros_v2_opencv_visual_pkg.create_qrcode:main',
            'parse_qrcode_node = microros_v2_opencv_visual_pkg.parse_qrcode:main',
            'detect_apriltag_node = microros_v2_opencv_visual_pkg.detect_apriltag:main',
        ],
    },
)
