from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_test_sensors = LaunchConfiguration('use_test_sensors')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_test_sensors',
            default_value='false',
            description='Set to true to launch the simulated /scan and /odometry publisher.',
        ),
        Node(
            package='edubot_autonomy',
            executable='edubot_autonomy_node',
            name='edubot_autonomy_node',
            output='screen',
            parameters=[
                {'front_camera_topic': '/camera_1/image_raw'},
                {'downward_camera_topic': '/camera_2/image_raw'},
                {'scan_topic': '/scan'},
                {'odom_topic': '/wheel/odometry'},
                {'cmd_vel_topic': '/cmd_vel'},
                {'map_topic': '/map'},
            ],
        ),
        Node(
            package='edubot_autonomy',
            executable='test_sensor_publisher',
            name='test_sensor_publisher',
            output='screen',
            condition=IfCondition(use_test_sensors),
        )
    ])
