from setuptools import find_packages, setup
import os
from glob import glob

package_name = "multi_brains"


setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.txt"),
        ),
        (os.path.join("share", package_name, "language"), glob("language/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson-nx@example.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "llm_agent_node=multi_brains.llm_agent_node:main",
            "terminal_client=multi_brains.clients.terminal_client:main",
            "color_selector=multi_brains.utils.colorSelect:main",
        ],
    },
)

