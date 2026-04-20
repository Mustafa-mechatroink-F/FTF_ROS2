from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ftf_share = get_package_share_directory('ftf_description')
    urdf_path = os.path.join(ftf_share, 'urdf', 'ftf_robot.urdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'mode': 'mapping',
            'minimum_travel_distance': 0.02,
            'minimum_travel_heading': 0.02,
            'map_update_interval': 1.0
        }],
        remappings=[('/scan', '/scan/filtered')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        slam_toolbox_node
    ])