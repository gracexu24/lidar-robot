"""Run Gazebo and Nav2 against a previously saved simulated map."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the simulated saved-map navigation launch description."""
    share = get_package_share_directory('my_robot')
    simulation_launch = os.path.join(
        share, 'launch', 'simulation.launch.py'
    )
    navigation_launch = os.path.join(
        share, 'launch', 'navigation.launch.py'
    )
    map_file = LaunchConfiguration('map')
    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', description='Absolute path to a saved simulation map'
        ),
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
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                'map': map_file,
                'start_base': 'false',
                'use_sim_time': 'true',
                'use_imu': 'false',
            }.items(),
        ),
    ])
