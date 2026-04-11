from setuptools import setup, find_packages
import os
from glob import glob

package_name = "nav2_tuning_tools"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "scenarios"), glob("config/scenarios/*.yaml")),
        (os.path.join("share", package_name, "config", "overrides"), glob("config/overrides/*.yaml")),
        (os.path.join("share", package_name, "config", "foxglove"), glob("config/foxglove/*")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Tejas",
    maintainer_email="tejas@farmience.com",
    description="Progressive benchmarking and live tuning tools for Nav2 + PID",
    license="MIT",
    entry_points={
        "console_scripts": [
            "metrics_node = nav2_tuning_tools.metrics_node:main",
            "system_monitor = nav2_tuning_tools.system_monitor:main",
            "param_tuner = nav2_tuning_tools.param_tuner:main",
            "test_runner = nav2_tuning_tools.test_runner:main",
            "export_config = nav2_tuning_tools.export_config:main",
        ],
    },
)
