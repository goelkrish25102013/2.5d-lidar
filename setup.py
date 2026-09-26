from setuptools import setup
import os
from glob import glob

package_name = 'lidar_mapper_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='you@example.com',
    description='3D lidar pan-scanner using RPLidar C1 and servo',
    license='MIT',
    entry_points={
    'console_scripts': [
        'lidar_driver_node_wired = lidar_mapper_ros2.lidar_driver_node_wired:main',
        'pan_controller_node_wired = lidar_mapper_ros2.pan_controller_node_wired:main',
        'lidar_driver_node_wireless = lidar_mapper_ros2.lidar_driver_node_wireless:main',
        'pan_controller_node_wireless = lidar_mapper_ros2.pan_controller_node_wireless:main',
        'scan_accumulator_node = lidar_mapper_ros2.scan_accumulator_node:main',
        ],
    },
)