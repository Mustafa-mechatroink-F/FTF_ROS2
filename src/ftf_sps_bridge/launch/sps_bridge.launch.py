from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ftf_sps_bridge',
            executable='sps_bridge',
            name='sps_bridge',
            output='screen',
            parameters=[
                {'ams_net_id': '192.168.100.1.1.1'},
                {'ip_address': '192.168.100.1'},
                {'ams_port': 851},

                # Geometrie deines FTF
                {'wheel_radius': 0.0905},
                {'wheel_base': 0.40},

                # Skalierung der SPS-Geschwindigkeit
                {'speed_scale': 0.001},

                # Frequenz der Odometrie
                {'update_rate': 50.0},
            ]
        )
    ])
