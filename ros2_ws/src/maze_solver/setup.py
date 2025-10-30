from setuptools import setup

package_name = 'maze_solver'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(
    ('share/' + package_name + '/launch', [
        'launch/map_and_nav.launch.py',
        'launch/simple_driver.launch.py',
        'launch/auto_goal.launch.py',
    ])
)
data_files.append(('share/' + package_name + '/config', ['config/nav2_params.yaml', 'config/rtabmap_params.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pavel Skorynin',
    maintainer_email='pavel.skorynin@yandex.ru',
    description='',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            'simple_driver = maze_solver.simple_driver_node:main',
            'lidar_filter_node = maze_solver.lidar_filter_node:main',
            'auto_goal_setter = maze_solver.auto_goal_node:main',
            'fixed_goal_setter = maze_solver.fixed_goal_node:main',
        ],
    },
)
