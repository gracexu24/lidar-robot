"""Run Gazebo, SLAM Toolbox, and Nav2 for simulated mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the simulated mapping launch description."""
    share = get_package_share_directory('my_robot')
    simulation_launch = os.path.join(
        share, 'launch', 'simulation.launch.py'
    )
    mapping_launch = os.path.join(share, 'launch', 'mapping.launch.py')
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation_launch),
            launch_arguments={
                'headless': headless,
                'start_rviz': start_rviz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'start_base': 'false',
                'use_sim_time': 'true',
                'use_imu': 'false',
            }.items(),
        ),
    ])
