"""Forward velocity commands to Gazebo and stop when commands become stale."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):
    """Publish a zero velocity when the command source stops updating."""

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('timeout', 0.6)
        self.timeout = self.get_parameter('timeout').value
        self.last_command_time = None
        self.stop_sent = True

        self.publisher = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.command_callback, 10
        )
        self.timer = self.create_timer(0.05, self.check_timeout)

    def command_callback(self, message):
        """Forward a fresh command and reset the timeout."""
        self.last_command_time = self.get_clock().now()
        self.stop_sent = False
        self.publisher.publish(message)

    def check_timeout(self):
        """Stop Gazebo once when the latest command is stale."""
        if self.last_command_time is None or self.stop_sent:
            return

        age = (self.get_clock().now() - self.last_command_time).nanoseconds
        if age / 1e9 >= self.timeout:
            self.publisher.publish(Twist())
            self.stop_sent = True


def main(args=None):
    """Run the velocity watchdog."""
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
