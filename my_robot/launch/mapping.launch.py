"""Run the physical robot and online SLAM for map creation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the online mapping launch description."""
    robot_share = get_package_share_directory('my_robot')
    slam_share = get_package_share_directory('slam_toolbox')
    lidar_port = LaunchConfiguration('lidar_port')
    start_base = LaunchConfiguration('start_base')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_imu = LaunchConfiguration('use_imu')
    start_mpu6050 = LaunchConfiguration('start_mpu6050')
    start_nav2 = LaunchConfiguration('start_nav2')
    debug_logging = LaunchConfiguration('debug_logging')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('start_mpu6050', default_value='true'),
        DeclareLaunchArgument(
            'start_nav2',
            default_value='false',
            description='Start Nav2 servers while mapping',
        ),
        DeclareLaunchArgument('debug_logging', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_share, 'launch', 'base.launch.py')
            ),
            condition=IfCondition(start_base),
            launch_arguments={
                'lidar_port': lidar_port,
                'use_imu': use_imu,
                'start_mpu6050': start_mpu6050,
                'debug_logging': debug_logging,
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
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    robot_share, 'launch', 'navigation_core.launch.py'
                )
            ),
            condition=IfCondition(start_nav2),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
