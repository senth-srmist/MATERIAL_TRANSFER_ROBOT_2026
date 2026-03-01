from setuptools import find_packages, setup

package_name = 'odom_base_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Tejas',
    maintainer_email='tejas@farmience.com',
    description='Transforms ZED camera odometry to base_link odometry',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_base_publisher = odom_base_publisher.odom_base_publisher:main',
        ],
    },
)
