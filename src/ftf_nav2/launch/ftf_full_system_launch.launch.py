import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Pfade zu den Paketen
    pkg_nav2 = get_package_share_directory('ftf_nav2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_description = get_package_share_directory('ftf_description')
    
    params_file = os.path.join(pkg_nav2, 'config', 'nav2_params.yaml')
    ekf_params = os.path.join(pkg_nav2, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'ftf_robot.urdf')
    rviz_config = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    
    # URDF laden
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Sensoren (SICK & IMU)
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sicks300_ros2'), 'launch', 'scan_with_filter.launch.py')])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('imu_3space'), 'launch', 'wt901c232.launch.py')])),

        # 2. State Publisher & EKF
        Node(package='robot_state_publisher', executable='robot_state_publisher', 
             parameters=[{'robot_description': robot_desc}]),
        Node(package='robot_localization', executable='ekf_node', parameters=[ekf_params]),

        # 3. FTF Drive & Navigation
        Node(package='ftf_control', executable='ftf_drive', parameters=[{'deadman_timeout_s': 1.0}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
            pkg_nav2, 'launch', 'navigation.launch.py')]), launch_arguments={'use_sim_time': 'False'}.items()),

        # 4. Docking & Unloading Nodes
        Node(package='ftf_control', executable='dock_laden', parameters=[params_file]),
        Node(package='ftf_control', executable='unload_return', parameters=[params_file]),

        # 5. RViz (Deine GUI)
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config]),
        
        # 6. Bonus: Das Mission Dashboard (siehe unten)
        ExecuteProcess(cmd=['python3', os.path.expanduser('~/fh_ws/ftf_dashboard.py')], output='screen')
    ])