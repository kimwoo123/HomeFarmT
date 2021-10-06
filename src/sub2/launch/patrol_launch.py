from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='sub2',
            node_executable='odom',
            node_name='odom'
        ),





        
    ])