from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sub2',
            node_executable='odom',
            node_name='odom',
        ),
        Node(
            package='sub2',
            node_executable='load_map',
            node_name='load_map',
            output='screen',
        ),
        Node(
            package='sub2',
            node_executable='a_star_local_path',
            node_name='a_star_local_path',
            output='screen',
        ),
        Node(
            package='sub2',
            node_executable='path_tracking',
            node_name='path_tracking',
            output='screen',
        ),
        Node(
            package='sub2',
            node_executable='lidar_trans',
            node_name='lidar_trans',
        ),
        # Node(
        #     package='sub2',
        #     node_executable='path_pub',
        #     node_name='path_pub',
        # ),
        Node(
            package='sub2',
            node_executable='a_star',
            node_name='a_star',
            output='screen',
        ),
        Node(
            package='sub2',
            node_executable='local_map_pub',
            node_name='local_map_pub',
        ),
    ])



