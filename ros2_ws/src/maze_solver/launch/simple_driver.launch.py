from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_solver',
            executable='simple_driver',
            name='simple_driver',
            output='screen',
        ),
    ])
