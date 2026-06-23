import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max
        ]

        if not valid_ranges:
            self.get_logger().info("No valid LiDAR readings")
            return

        closest_distance = min(valid_ranges)

        self.get_logger().info(
            f"Closest object: {closest_distance:.2f} meters"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
