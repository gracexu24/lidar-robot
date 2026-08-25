"""Launch GPIO motor control, transforms, and optionally the D500 LiDAR."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the physical-base launch description."""
    package_share = get_package_share_directory('my_robot')
    with open(os.path.join(package_share, 'urdf', 'robot.urdf')) as urdf_file:
        robot_description = urdf_file.read()

    lidar_port = LaunchConfiguration('lidar_port')
    start_motors = LaunchConfiguration('start_motors')
    start_lidar = LaunchConfiguration('start_lidar')
    use_imu = LaunchConfiguration('use_imu')
    start_mpu6050 = LaunchConfiguration('start_mpu6050')
    debug_logging = LaunchConfiguration('debug_logging')
    publish_raw_odom = ParameterValue(
        PythonExpression(["'", use_imu, "'.lower() != 'true'"]),
        value_type=bool,
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ttyUSB0',
            description='D500/LD19 serial device',
        ),
        DeclareLaunchArgument('start_motors', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument(
            'debug_logging',
            default_value='false',
            description='Log motor, IMU, and scan diagnostics',
        ),
        DeclareLaunchArgument(
            'use_imu', default_value='false',
            description='Fuse /imu/data with wheel odometry',
        ),
        DeclareLaunchArgument(
            'start_mpu6050', default_value='true',
            description='Start the included I2C MPU6050 driver',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='my_robot',
            executable='base_controller',
            condition=IfCondition(start_motors),
            parameters=[
                os.path.join(package_share, 'config', 'motors.yaml'),
                {
                    'publish_odom': publish_raw_odom,
                    'publish_odom_tf': publish_raw_odom,
                    'debug_logging': ParameterValue(
                        debug_logging, value_type=bool
                    ),
                },
            ],
            output='screen',
        ),
        Node(
            package='my_robot',
            executable='mpu6050_driver',
            name='mpu6050_driver',
            condition=IfCondition(PythonExpression([
                "'", use_imu, "'.lower() == 'true' and '",
                start_mpu6050, "'.lower() == 'true'",
            ])),
            parameters=[
                os.path.join(package_share, 'config', 'mpu6050.yaml'),
                {
                    'debug_logging': ParameterValue(
                        debug_logging, value_type=bool
                    )
                },
            ],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            condition=IfCondition(use_imu),
            parameters=[os.path.join(package_share, 'config', 'ekf.yaml')],
            remappings=[('odometry/filtered', 'odom')],
            output='screen',
        ),
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='d500_lidar',
            condition=IfCondition(start_lidar),
            parameters=[{
                'product_name': 'LDLiDAR_LD19',
                'topic_name': 'scan',
                'frame_id': 'base_laser',
                'port_name': lidar_port,
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
            }],
            output='screen',
        ),
        Node(
            package='my_robot',
            executable='lidar_reader',
            name='lidar_reader',
            condition=IfCondition(PythonExpression([
                "'", start_lidar, "'.lower() == 'true' and '",
                debug_logging, "'.lower() == 'true'",
            ])),
            parameters=[{'debug_logging': True}],
            output='screen',
        ),
    ])
