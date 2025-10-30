from setuptools import setup

package_name = 'waveshare_motor_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Nikita Savinov',
    maintainer_email='nikita.savinov@example.com',
    description='Driver node that bridges cmd_vel to a Waveshare sub-controller over serial.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = waveshare_motor_driver.motor_driver_node:main',
        ],
    },
)
