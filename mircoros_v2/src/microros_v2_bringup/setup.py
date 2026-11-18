from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'microros_v2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_driver = microros_v2_bringup.camera_driver:main',
            'calib_verify = microros_v2_bringup.calib_verify:main',
            'base_driver = microros_v2_bringup.base_driver:main',
            'visual_track = microros_v2_bringup.visual_track:main',
            'topic_test = microros_v2_bringup.topic_test:main',
        ],
    },
)
