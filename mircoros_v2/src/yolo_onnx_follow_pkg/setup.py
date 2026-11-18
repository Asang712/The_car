from glob import glob
import os

from setuptools import find_packages, setup


package_name = "yolo_onnx_follow_pkg"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson-nx@example.com",
    description="Standalone YOLOv11 ONNX visual follow package",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "yolo_onnx_follow_node=yolo_onnx_follow_pkg.yolo_onnx_follow_node:main",
        ],
    },
)
