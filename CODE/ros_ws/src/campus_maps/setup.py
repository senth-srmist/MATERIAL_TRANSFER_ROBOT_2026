from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'campus_maps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        # Required for ament package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Install map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='lalithesh',
    maintainer_email='lk9092@srmist.edu.in',
    description='Campus map tiles and dynamic map switching node',
    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'tile_switcher = campus_maps.tile_switcher:main',
        ],
    },
)
