from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="imu_3space",
            executable="wt901c_rs232_node",
            name="wt901c_rs232",
            output="screen",
            parameters=[
                {"port": "/dev/ttyUSB0"},
                {"baudrate": 115200},
                {"frame_id": "imu_link"},
            ]
        )
    ])
