from glob import glob
import os

from setuptools import find_packages, setup


package_name = "microros_v2_kcf_follow_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yahboom",
    maintainer_email="2474250941@qq.com",
    description="Standalone bbox-based KCF follow controller for MicroROS V2.",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "tracking_bbox_follow_node = microros_v2_kcf_follow_pkg.tracking_bbox_follow_node:main",
        ],
    },
)
