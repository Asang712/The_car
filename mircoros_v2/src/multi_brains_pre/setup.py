from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'multi_brains_pre'

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
    maintainer_email='yahboom@168.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mcp_service=multi_brains_pre.mcp_service:main',
            'follow_line=multi_brains_pre.external_action.follow_line:main',
            'colorSelect=multi_brains_pre.external_action.colorSelect:main'
        ],
    }, 
)
