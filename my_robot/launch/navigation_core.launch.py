"""Launch the Nav2 servers used by this small differential-drive robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the common Nav2 server launch description."""
    params = os.path.join(
        get_package_share_directory('my_robot'), 'config', 'nav2.yaml'
    )
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool
    )
    common = {
        'parameters': [params, {'use_sim_time': use_sim_time}],
        'output': 'screen',
    }
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            **common,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            **common,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            remappings=[('cmd_vel', 'cmd_vel_nav')],
            **common,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            **common,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('smoothed_cmd_vel', 'cmd_vel'),
            ],
            **common,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': lifecycle_nodes,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
