import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_trainee22_1',
            namespace='demo',
            executable='odom',
            name='odom',
        ),
        Node(
            package='demo_trainee22_1',
            namespace='demo',
            executable='plot_position',
            name='plot_position'
        ),
        Node(
            package='robot_localization',
            namespace='demo',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("demo_trainee22_1"), 'params', 'ekf_node.yaml')]
        )
    ])