from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='sub3',
        #     node_executable='iot_udp',
        #     node_name='client',
        # ),
        # Node(
        #     package='sub3',
        #     node_executable='data_center',
        #     node_name='data_center',
        # ),
        Node(
            package='sub3',
            node_executable='tf_detector',
            node_name='front',
        ),
        Node(
            package='sub3',
            node_executable='tf_detector_left',
            node_name='left',
        ),
        Node(
            package='sub3',
            node_executable='tf_detector_right',
            node_name='right',
        ),
    ])

