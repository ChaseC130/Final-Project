from setuptools import setup, find_packages

package_name = 'edubot_autonomy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/edubot_autonomy_launch.py', 'launch/test_sensors_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edubot_team',
    maintainer_email='you@example.com',
    description='Edubot autonomy package for lane following, mapping, and sign recognition.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edubot_autonomy_node=edubot_autonomy.edubot_autonomy_node:main',
            'camera_driver_node=edubot_autonomy.camera_driver_node:main',
            'test_sensor_publisher=edubot_autonomy.test_sensor_publisher:main',
        ],
    },
)
