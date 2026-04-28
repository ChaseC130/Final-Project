from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'behaviors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Final project autonomous road navigation ROS2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_detector = behaviors.lane_detector:main',
            'road_navigator = behaviors.road_navigator:main',
            'lane_map_publisher = behaviors.lane_map_publisher:main',
            'cone_detector = behaviors.cone_detector:main',
            'sign_detector = behaviors.sign_detector:main',
            'autonomous_mapper = behaviors.autonomous_mapper:main',
        ],
    },
)
