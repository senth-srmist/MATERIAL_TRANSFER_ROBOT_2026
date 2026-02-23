from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sabertooth_diff_drive'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # ✅ Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # ✅ Install config files (twist_mux.yaml etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lalithesh',
    maintainer_email='lk9092@srmist.edu.in',
    description='Differential drive controllers for Sabertooth motor driver (simple + Nav2)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # OLD node
            'sabertooth_diff_drive = sabertooth_diff_drive.sabertooth_diff_drive_node:main',

            # NEW controller node
            'controller_node = sabertooth_diff_drive.controller_node:main',
        ],
    },
)