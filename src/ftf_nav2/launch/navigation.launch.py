import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Pfad zum Nav2-Paket
    pkg_ftf_nav2 = get_package_share_directory('ftf_nav2')

    # Konfigurationen
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    
    # Standardpfade für Parameter und Karte
    default_params_file = os.path.join(pkg_ftf_nav2, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(pkg_ftf_nav2, 'maps', 'CIMTT_komplett.yaml')

    params_file = LaunchConfiguration("params_file", default=default_params_file)
    map_yaml = LaunchConfiguration("map", default=default_map_path)

    return LaunchDescription([
        # Argumente deklarieren
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params_file),
        DeclareLaunchArgument("map", default_value=default_map_path),

        # --- NAVIGATION NODES (Software) ---

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        ),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")], 
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),

        # --- LIFECYCLE MANAGERS ---
        
        # Manager für Lokalisierung
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": ["map_server", "amcl"]}],
        ),
        
        # Manager für Navigation
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["controller_server", "planner_server", "bt_navigator", "behavior_server"],
            }],
        ),
    ])