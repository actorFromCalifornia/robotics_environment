import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Получаем путь к пакету
    maze_solver_dir = get_package_share_directory('maze_solver')

    rtabmap_params_file = os.path.join(
        maze_solver_dir,
        'config',
        'rtabmap_params.yaml'
    )

    nav2_params_file = os.path.join(
        maze_solver_dir, 
        'config', 
        'nav2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtabmap_params_file',
            default_value=rtabmap_params_file,
            description='Full path to the RTAB-Map parameters file'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=nav2_params_file,
            description='Full path to the Nav2 parameters file'),
            
        # RTAB-Map ICP Odometry Node
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[LaunchConfiguration('rtabmap_params_file')],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),  # Публикуем одометрию напрямую в /odom
            ]
        ),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[LaunchConfiguration('rtabmap_params_file')],
            remappings=[
                ('scan', '/scan'),
                ('map', '/rtabmap_map'),
                ('odom', '/odom'),
                ('grid_map', '/map'),
            ],
            arguments=['--delete_db_on_start']
        ),

        # Navigation
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/map', '/rtabmap_map')
            ]
        ),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')],
        ),
        
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')],
            remappings=[('/map', '/rtabmap_map')]
        ),
        
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')]),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[LaunchConfiguration('nav2_params_file')]),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }])
    ])
