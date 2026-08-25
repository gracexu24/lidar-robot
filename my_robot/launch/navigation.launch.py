"""Navigate autonomously using a previously saved map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the saved-map navigation launch description."""
    share = get_package_share_directory('my_robot')
    params = os.path.join(share, 'config', 'nav2.yaml')
    map_file = LaunchConfiguration('map')
    lidar_port = LaunchConfiguration('lidar_port')
    use_imu = LaunchConfiguration('use_imu')
    start_mpu6050 = LaunchConfiguration('start_mpu6050')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to the saved map YAML file'
        ),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('start_mpu6050', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'base.launch.py')
            ),
            launch_arguments={
                'lidar_port': lidar_port,
                'use_imu': use_imu,
                'start_mpu6050': start_mpu6050,
            }.items(),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[params, {'yaml_filename': map_file}],
            output='screen',
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'navigation_core.launch.py')
            )
        ),
    ])
