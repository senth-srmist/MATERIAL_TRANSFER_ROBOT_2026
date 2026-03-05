from setuptools import find_packages, setup
import os
from glob import glob

package_name = "diff_drive_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Required index
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        # package.xml
        ("share/" + package_name, ["package.xml"]),
        # ✅ Install launch files
        (os.path.join("share", package_name,
                      "launch"), glob("launch/*.launch.py")),
        # ✅ Install config files (twist_mux.yaml etc.)
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lalithesh",
    maintainer_email="lk9092@srmist.edu.in",
    description=
    "Differential drive controllers for Cytron SmartDuo motor driver (Simple Serial)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller_node = diff_drive_controller.controller_node:main",
        ],
    },
)
