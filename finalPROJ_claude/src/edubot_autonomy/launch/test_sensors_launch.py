from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edubot_autonomy',
            executable='test_sensor_publisher',
            name='test_sensor_publisher',
            output='screen',
        )
    ])
