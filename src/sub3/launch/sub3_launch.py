from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub3',
            node_executable='iot_udp',
            node_name='client',
        ),
    ])

