from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ftf_share = get_package_share_directory('ftf_description')
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(ftf_share, 'urdf', 'ftf_robot.urdf')).read(),
                'use_sim_time': False
            }]
        ),
        # SLAM Toolbox (Vorbereitet)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'mode': 'mapping'
            }],
            remappings=[('/scan', '/scan_filtered')]
        ),
    ])
