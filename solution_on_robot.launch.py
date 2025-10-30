from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Запуск robot.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("robot.launch.py")
        ),

        # Запуск решения
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("solution.launch.py")
        ),
    ])
