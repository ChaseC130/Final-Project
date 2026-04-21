from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Topic name for camera images'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for velocity commands'
    )
    
    # Lane follower node
    lane_follower_node = Node(
        package='your_package_name',  # CHANGE THIS to your package name
        executable='lane_follower',
        name='lane_follower',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'linear_speed': 0.2,
            'max_angular_speed': 0.5,
            'kp': 0.005,
            'ki': 0.0001,
            'kd': 0.001,
            'roi_height': 0.3,
            'min_line_length': 20,
            'max_line_gap': 10
        }]
    )
    
    return LaunchDescription([
        camera_topic_arg,
        cmd_vel_topic_arg,
        lane_follower_node
    ])
