from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera_1/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='behaviors',
            executable='lane_detector',
            name='lane_detector',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'image_topic': image_topic},
            ],
        ),

        Node(
            package='behaviors',
            executable='lane_map_publisher',
            name='lane_map_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='behaviors',
            executable='cone_detector',
            name='cone_detector',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'image_topic': image_topic},
                {'scan_topic': scan_topic},
            ],
        ),

        Node(
            package='behaviors',
            executable='sign_detector',
            name='sign_detector',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'image_topic': image_topic},
            ],
        ),

        Node(
            package='behaviors',
            executable='road_navigator',
            name='road_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'scan_topic': scan_topic},
            ],
        ),
    ])
