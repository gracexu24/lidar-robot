"""ROS 2 cmd_vel controller and open-loop odometry for a four-wheel base."""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from my_robot.stepper_hardware import FourWheelHardware


def clamp(value, lower, upper):
    """Constrain a number to the inclusive bounds."""
    return max(lower, min(upper, value))


class BaseController(Node):
    """Convert differential-drive velocity commands into four step rates."""

    def __init__(self):
        super().__init__('base_controller')
        self.declare_parameter('step_pins', [17, 5, 23, 12])
        self.declare_parameter('dir_pins', [27, 6, 24, 16])
        self.declare_parameter('reset_pins', [22, 13, 25, 20])
        self.declare_parameter(
            'direction_inverted', [False, False, True, True]
        )
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('steps_per_revolution', 3200.0)
        self.declare_parameter('max_step_rate', 1200.0)
        self.declare_parameter('max_step_acceleration', 1600.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_odom_tf', True)

        self._radius = self.get_parameter('wheel_radius').value
        self._separation = self.get_parameter('wheel_separation').value
        self._steps_per_rev = self.get_parameter(
            'steps_per_revolution'
        ).value
        self._max_rate = self.get_parameter('max_step_rate').value
        self._max_acceleration = self.get_parameter(
            'max_step_acceleration'
        ).value
        self._timeout = self.get_parameter('command_timeout').value
        self._publish_odom = self.get_parameter('publish_odom').value
        self._publish_odom_tf = self.get_parameter('publish_odom_tf').value
        if min(self._radius, self._separation, self._steps_per_rev) <= 0.0:
            raise ValueError(
                'Robot dimensions and steps_per_revolution must be positive'
            )

        self._hardware = FourWheelHardware(
            self.get_parameter('step_pins').value,
            self.get_parameter('dir_pins').value,
            self.get_parameter('reset_pins').value,
            self.get_parameter('direction_inverted').value,
        )
        self._target_rates = [0.0] * 4
        self._current_rates = [0.0] * 4
        self._last_command = time.monotonic()
        self._last_update = time.monotonic()
        self._last_counts = self._hardware.step_counts()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.create_subscription(Twist, 'cmd_vel', self._command_callback, 10)
        self._wheel_odom_publisher = self.create_publisher(
            Odometry, 'wheel/odom', 10
        )
        self._odom_publisher = (
            self.create_publisher(Odometry, 'odom', 10)
            if self._publish_odom else None
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self._control_update)
        self.get_logger().info('Four-wheel GPIO base controller is ready')

    def _command_callback(self, message):
        linear = message.linear.x
        angular = message.angular.z
        left_velocity = linear - angular * self._separation / 2.0
        right_velocity = linear + angular * self._separation / 2.0
        steps_per_meter = self._steps_per_rev / (2.0 * math.pi * self._radius)
        left_rate = clamp(
            left_velocity * steps_per_meter, -self._max_rate, self._max_rate
        )
        right_rate = clamp(
            right_velocity * steps_per_meter, -self._max_rate, self._max_rate
        )
        self._target_rates = [left_rate, left_rate, right_rate, right_rate]
        self._last_command = time.monotonic()

    def _control_update(self):
        now_monotonic = time.monotonic()
        elapsed = max(now_monotonic - self._last_update, 1e-6)
        self._last_update = now_monotonic
        if now_monotonic - self._last_command > self._timeout:
            self._target_rates = [0.0] * 4

        max_change = self._max_acceleration * elapsed
        self._current_rates = [
            current + clamp(target - current, -max_change, max_change)
            for current, target in zip(self._current_rates, self._target_rates)
        ]
        self._hardware.set_rates(self._current_rates)
        self._publish_odometry(elapsed)

    def _publish_odometry(self, elapsed):
        counts = self._hardware.step_counts()
        deltas = [
            count - previous
            for count, previous in zip(counts, self._last_counts)
        ]
        self._last_counts = counts
        meters_per_step = 2.0 * math.pi * self._radius / self._steps_per_rev
        left_distance = 0.5 * (deltas[0] + deltas[1]) * meters_per_step
        right_distance = 0.5 * (deltas[2] + deltas[3]) * meters_per_step
        distance = 0.5 * (left_distance + right_distance)
        yaw_change = (right_distance - left_distance) / self._separation
        self._x += distance * math.cos(self._yaw + yaw_change / 2.0)
        self._y += distance * math.sin(self._yaw + yaw_change / 2.0)
        self._yaw += yaw_change
        linear_velocity = distance / elapsed
        angular_velocity = yaw_change / elapsed
        half_yaw = self._yaw / 2.0

        stamp = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_footprint'
        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.orientation.z = math.sin(half_yaw)
        odometry.pose.pose.orientation.w = math.cos(half_yaw)
        odometry.twist.twist.linear.x = linear_velocity
        odometry.twist.twist.angular.z = angular_velocity
        odometry.pose.covariance[0] = 0.05
        odometry.pose.covariance[7] = 0.05
        odometry.pose.covariance[35] = 0.10
        self._wheel_odom_publisher.publish(odometry)
        if self._odom_publisher is not None:
            self._odom_publisher.publish(odometry)

        if self._publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_footprint'
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation.z = math.sin(half_yaw)
            transform.transform.rotation.w = math.cos(half_yaw)
            self._tf_broadcaster.sendTransform(transform)

    def destroy_node(self):
        """Disable the motor drivers before destroying the ROS node."""
        self._hardware.close()
        return super().destroy_node()


def main(args=None):
    """Run the base controller node."""
    rclpy.init(args=args)
    node = None
    try:
        node = BaseController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
