from setuptools import setup, find_packages
import os
from glob import glob

package_name = "relocalization_manager"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Tejas",
    maintainer_email="tejas@farmience.com",
    description="ArUco-based relocalization for autonomous recovery and arbitrary start position",
    license="MIT",
    entry_points={
        "console_scripts": [
            "relocalization_manager = relocalization_manager.relocalization_manager:main",
        ],
    },
)
