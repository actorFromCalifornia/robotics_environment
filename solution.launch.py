from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Получаем пути к пакетам
    maze_solver_pkg = FindPackageShare('maze_solver')
    
    # Пути к launch-файлам
    map_and_nav_launch_path = PathJoinSubstitution([
        maze_solver_pkg, 'launch', 'map_and_nav.launch.py'
    ])

    simple_driver_launch_path = PathJoinSubstitution([
        maze_solver_pkg, 'launch', 'simple_driver.launch.py'
    ])

    auto_goal_launch_path = PathJoinSubstitution([
        maze_solver_pkg, 'launch', 'auto_goal.launch.py'
    ])
    
    return LaunchDescription([
        # Опционально: объявить аргументы, которые могут понадобиться
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        
        # Запуск map_and_nav.launch.py из webots_bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_and_nav_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        ),

        # Запуск ноды постановщика цели
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(auto_goal_launch_path),
        #     launch_arguments={
        #         'use_sim_time': LaunchConfiguration('use_sim_time'),
        #         'map_topic': '/rtabmap_map',
        #     }.items()
        # ),

        # Запуск Simple Driver'a
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(simple_driver_launch_path)
        # ),

        Node(
            package='maze_solver',
            executable='fixed_goal_setter',
            name='fixed_goal_setter',
            output='screen',
            parameters=[{
                'goal_x': 0.0,
                'goal_y': 2.5,
                'goal_yaw': 0.0,
                'max_attempts': 100,
                'retry_radius': 0.1,
            }],
        ),

        Node(
            package='maze_solver',
            executable='lidar_filter_node',
            name='lidar_filter',
            output='screen',
            parameters=[{
                'input_topic': '/scan_raw',
                'output_topic': '/scan',
                'angle_min1': 0.0,
                'angle_max1': 360.0,
                'angle_min2': 360.0,
                'angle_max2': 360.0,
                'angle_min3': 360.0,
                'angle_max3': 360.0
            }],
        ),
    ])
