"""Launch the four-wheel robot in Gazebo Harmonic."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start Gazebo, topic bridges, robot transforms, and optional RViz."""
    package_share = get_package_share_directory('my_robot')
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(package_share, 'worlds', 'test_room.sdf')
    bridge_config = os.path.join(
        package_share, 'config', 'gazebo_bridge.yaml'
    )
    with open(os.path.join(package_share, 'urdf', 'robot.urdf')) as urdf_file:
        robot_description = urdf_file.read()

    headless = LaunchConfiguration('headless')
    start_rviz = LaunchConfiguration('start_rviz')
    gazebo_launch = os.path.join(
        ros_gz_share, 'launch', 'gz_sim.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the Gazebo server without its graphical client',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start RViz on this computer',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=UnlessCondition(headless),
            launch_arguments={
                'gz_args': ['-r -v 3 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            condition=IfCondition(headless),
            launch_arguments={
                'gz_args': ['-r -s -v 3 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gazebo_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),
        Node(
            package='my_robot',
            executable='cmd_vel_watchdog',
            name='cmd_vel_watchdog',
            parameters=[{'use_sim_time': True, 'timeout': 0.6}],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description, 'use_sim_time': True}
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            condition=IfCondition(start_rviz),
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ])
