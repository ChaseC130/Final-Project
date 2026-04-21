from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.substitutions import PathJoinSubstitution 
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description(): 
    map_file = PathJoinSubstitution([ FindPackageShare('behaviors'), 
                                     'maps', 
                                     'SciHallway.yaml' ]) 

    return LaunchDescription(
        [   # Map Server
            Node(package='nav2_map_server', 
                executable='map_server', 
                name='map_server', 
                output='screen', 
                parameters=[ {'yaml_filename': map_file}, 
                            {'use_lifecycle_mgr': False} # disables lifecycle behavior 
                            ]),

            # Lifecycle manager - needed for jazzy
            Node(package='nav2_lifecycle_manager', 
                executable='lifecycle_manager', 
                name='lifecycle_manager_map', 
                output='screen', parameters=[ {'autostart': True}, 
                                            {'node_names': ['map_server']} 
                                            ] ),
            
            # Static Odom Publisher
            Node(package='behaviors', 
                executable='static_pose_publisher', 
                name='static_pose_publisher', 
                output='screen')
        ])