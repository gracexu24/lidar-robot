"""Example node that reports the closest valid LiDAR return."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarReader(Node):
    """Subscribe to LaserScan data and report the nearest return."""

    def __init__(self):
        super().__init__('lidar_reader')
        self.declare_parameter('debug_logging', False)
        self._debug_logging = self.get_parameter('debug_logging').value

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self._last_scan_time = None
        self.create_timer(2.0, self._check_connection)
        self.get_logger().info('Waiting for LaserScan messages on /scan')

    def _check_connection(self):
        """Warn when the LiDAR scan stream is missing or stale."""
        if self._last_scan_time is None:
            detail = 'No /scan messages received'
        elif time.monotonic() - self._last_scan_time > 3.0:
            detail = 'LiDAR /scan stream stopped'
        else:
            return
        self.get_logger().warning(
            f'{detail}; check power, serial port, permissions, and driver',
            throttle_duration_sec=5.0,
        )

    def scan_callback(self, msg):
        """Process one laser scan."""
        first_scan = self._last_scan_time is None
        self._last_scan_time = time.monotonic()
        if first_scan:
            self.get_logger().info(
                'LiDAR connection active; receiving /scan messages'
            )

        valid_measurements = [
            (index, distance)
            for index, distance in enumerate(msg.ranges)
            if msg.range_min < distance < msg.range_max
        ]

        if not valid_measurements:
            self.get_logger().warning(
                'LiDAR scan contains no valid ranges',
                throttle_duration_sec=2.0,
            )
            return

        closest_index, closest_distance = min(
            valid_measurements, key=lambda measurement: measurement[1]
        )
        closest_angle = msg.angle_min + closest_index * msg.angle_increment
        if self._debug_logging:
            self.get_logger().info(
                '[lidar debug] '
                f'received {len(msg.ranges)} rays, '
                f'{len(valid_measurements)} valid, '
                f'closest={closest_distance:.2f} m '
                f'at {closest_angle:.2f} rad, '
                f'valid range={msg.range_min:.2f}..{msg.range_max:.2f} m',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f'Closest object: {closest_distance:.2f} meters',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    """Run the LiDAR reader example."""
    rclpy.init(args=args)
    node = None
    try:
        node = LidarReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
