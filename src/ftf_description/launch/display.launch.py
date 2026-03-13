from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('ftf_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'ftf_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Node 1: Robot State Publisher 
    # Er veröffentlicht automatisch alle festen (fixed) Joints aus der URDF 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # Node 2: RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz
    ])