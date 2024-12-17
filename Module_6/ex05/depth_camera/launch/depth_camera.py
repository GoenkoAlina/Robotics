from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_camera',
            executable='depth_camera',
            name='depth_camera',
        ),
    ])
