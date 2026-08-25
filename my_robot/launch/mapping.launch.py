"""Run the physical robot, online SLAM, and Nav2 for map creation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the online mapping launch description."""
    robot_share = get_package_share_directory('my_robot')
    slam_share = get_package_share_directory('slam_toolbox')
    lidar_port = LaunchConfiguration('lidar_port')
    use_imu = LaunchConfiguration('use_imu')
    start_mpu6050 = LaunchConfiguration('start_mpu6050')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('start_mpu6050', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_share, 'launch', 'base.launch.py')
            ),
            launch_arguments={
                'lidar_port': lidar_port,
                'use_imu': use_imu,
                'start_mpu6050': start_mpu6050,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': os.path.join(
                    robot_share, 'config', 'slam.yaml'
                ),
                'use_sim_time': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    robot_share, 'launch', 'navigation_core.launch.py'
                )
            )
        ),
    ])
