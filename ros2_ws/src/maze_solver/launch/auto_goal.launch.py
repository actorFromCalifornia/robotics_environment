#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('maze_solver')
    
    # Declare launch arguments
    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='Map topic name'
    )
    
    goal_frame_arg = DeclareLaunchArgument(
        'goal_frame',
        default_value='map',
        description='Goal frame ID'
    )
    
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Robot frame ID'
    )
    
    free_threshold_arg = DeclareLaunchArgument(
        'free_threshold',
        default_value='25',
        description='Free space threshold for occupancy grid'
    )
    
    visited_radius_arg = DeclareLaunchArgument(
        'visited_radius',
        default_value='0.4',
        description='Radius around robot position to mark as visited'
    )
    
    min_goal_distance_arg = DeclareLaunchArgument(
        'min_goal_distance',
        default_value='1.0',
        description='Minimum distance for goal selection'
    )
    
    goal_timeout_sec_arg = DeclareLaunchArgument(
        'goal_timeout_sec',
        default_value='30.0',
        description='Goal timeout in seconds'
    )

    # Auto goal setter node
    auto_goal_node = Node(
        package='maze_solver',
        executable='auto_goal_setter',
        name='auto_goal_setter',
        output='screen',
        parameters=[{
            'map_topic': LaunchConfiguration('map_topic'),
            'goal_frame': LaunchConfiguration('goal_frame'),
            'robot_frame': LaunchConfiguration('robot_frame'),
            'free_threshold': LaunchConfiguration('free_threshold'),
            'visited_radius': LaunchConfiguration('visited_radius'),
            'min_goal_distance': LaunchConfiguration('min_goal_distance'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
        }],
        remappings=[
            ('/map', LaunchConfiguration('map_topic')),
        ]
    )

    return LaunchDescription([
        map_topic_arg,
        goal_frame_arg,
        robot_frame_arg,
        free_threshold_arg,
        visited_radius_arg,
        min_goal_distance_arg,
        goal_timeout_sec_arg,
        auto_goal_node,
    ])
