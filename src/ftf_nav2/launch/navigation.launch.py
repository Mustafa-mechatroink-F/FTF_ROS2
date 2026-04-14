import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ftf_nav2 = get_package_share_directory('ftf_nav2')

    default_params_file = os.path.join(pkg_ftf_nav2, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(pkg_ftf_nav2, 'maps', 'CIMTT_komplett.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')

    # Liste aller Knoten für den Lifecycle Manager
    nav_nodes = [
        'map_server', 
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml file'
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

        # Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')]
        ),

       # BT Navigator
        Node(
         package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                # WIR ERZWINGEN DIE FUNKTIONIERENDE DATEI FÜR ALLES:
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                'default_nav_to_pose_bt_xml': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
                'default_nav_through_poses_bt_xml': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
            }]
        
        ),
        # Waypoint Follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}]
        ),

       # NEU: Docking Server (Damit der Prozess überhaupt startet)
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')] # <--- HIER UMBIEGEN
        ),
        # EINER für alles: Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_service',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': nav_nodes,
                'bond_timeout': 10.0
            }]
        ),
    ])