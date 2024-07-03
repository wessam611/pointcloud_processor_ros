# pointcloud_processor_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_processor',
            executable='pointcloud_processor_node',
            name='pointcloud_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[]  # Add parameters if needed
        ),
    ])
