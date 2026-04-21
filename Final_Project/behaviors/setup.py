from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emartinso',
    maintainer_email='emartinso@ltu.edu',
    description='Behavior based robotics tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller = behaviors.controller:main',
            'pilot = behaviors.pilot:main',
            'lidar_turn_controller = behaviors.lidar_turn_controller:main',
            'planner = behaviors.planner:main',
            'static_pose_publisher = behaviors.static_pose_publisher:main',
            'autonomous_mapper = behaviors.autonomous_mapper:main',
            'simple_explorer = behaviors.simple_explorer:main',
        ],
    },
)
