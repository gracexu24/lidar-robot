"""Command-line front/back/turn/stop utility."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


COMMANDS = {
    'front': (1.0, 0.0),
    'back': (-1.0, 0.0),
    'left': (0.0, 1.0),
    'right': (0.0, -1.0),
    'stop': (0.0, 0.0),
}


def parse_arguments(args):
    """Parse command arguments after removing ROS-specific arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=COMMANDS)
    parser.add_argument('--speed', type=float, default=0.10, help='m/s')
    parser.add_argument(
        '--turn-speed', type=float, default=0.6, help='rad/s'
    )
    parser.add_argument('--duration', type=float, default=1.0, help='seconds')
    return parser.parse_args(remove_ros_args(args=args)[1:])


def main(args=None):
    """Publish the requested motion for a bounded duration."""
    rclpy.init(args=args)
    options = parse_arguments(args)
    node = Node('manual_drive')
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    linear_scale, angular_scale = COMMANDS[options.command]
    command = Twist()
    command.linear.x = linear_scale * abs(options.speed)
    command.angular.z = angular_scale * abs(options.turn_speed)

    end_time = time.monotonic() + max(0.0, options.duration)
    while rclpy.ok() and time.monotonic() < end_time:
        publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=0.05)

    publisher.publish(Twist())
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
