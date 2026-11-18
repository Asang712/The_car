from setuptools import find_packages, setup

package_name = 'common_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
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
            'joy_control_node = common_demo.control.joy_control:main',
            'keyboard_control= common_demo.control.keyboard_contol:main',
            'calibrate_linear = common_demo.calibration.calibrate_linear:main',
            'calibrate_angular_z = common_demo.calibration.calibrate_angular_z:main',
            #-------------------------------------------------------------------------
            'laser_Avoidance = common_demo.lidar.laser_Avoidance:main',
            'laser_Tracker = common_demo.lidar.laser_Tracker:main',
            'laser_Warning = common_demo.lidar.laser_Warning:main',
            'laser_patrol = common_demo.lidar.laser_patrol:main',
            
            #-------------------------------------------------------------------------
            'model_display = common_demo.model_display.model_display:main',
            #-------------------------------------------------------------------------
            'multi_robot_sync = common_demo.multi_robot.multi_robot_sync:main'
        ],  
    },
)


