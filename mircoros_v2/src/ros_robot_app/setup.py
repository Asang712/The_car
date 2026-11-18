from setuptools import find_packages, setup

package_name = 'ros_robot_app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='orin.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'app_save_map = ros_robot_app.app_save_map:main',
            'transmission = ros_robot_app.transmission:main',
            'robot_pose_publisher = ros_robot_app.robot_pose_publisher:main',
        ],
    },
)
