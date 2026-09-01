"""ROS 2 I2C driver for an MPU6050 accelerometer and gyroscope."""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


GRAVITY = 9.80665
DEGREES_TO_RADIANS = math.pi / 180.0


class Mpu6050Driver(Node):
    """Read an MPU6050 over I2C and publish standard ROS messages."""

    POWER_MANAGEMENT_1 = 0x6B
    SAMPLE_RATE_DIVIDER = 0x19
    CONFIGURATION = 0x1A
    GYROSCOPE_CONFIG = 0x1B
    ACCELEROMETER_CONFIG = 0x1C
    DATA_START = 0x3B
    WHO_AM_I = 0x75

    def __init__(self):
        super().__init__('mpu6050_driver')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('calibration_samples', 500)
        self.declare_parameter('gyro_stddev', 0.02)
        self.declare_parameter('accel_stddev', 0.20)
        self.declare_parameter('debug_logging', False)

        if SMBus is None:
            raise RuntimeError(
                'smbus2 is not installed; run: sudo apt install python3-smbus2'
            )

        self._address = self.get_parameter('i2c_address').value
        self._frame_id = self.get_parameter('frame_id').value
        self._debug_logging = self.get_parameter('debug_logging').value
        publish_rate = self.get_parameter('publish_rate').value
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        bus_number = self.get_parameter('i2c_bus').value
        self._bus = SMBus(bus_number)
        if self._debug_logging:
            self.get_logger().info(
                f'[imu debug] opened I2C bus {bus_number}; '
                f'probing address 0x{self._address:02x}'
            )
        try:
            self._initialize_sensor()
        except Exception as error:
            self.get_logger().error(
                f'MPU6050 connection failed on I2C bus {bus_number}, '
                f'address 0x{self._address:02x}: {error}'
            )
            self._bus.close()
            raise
        self._gyro_bias = self._calibrate_gyroscope(
            self.get_parameter('calibration_samples').value
        )
        if self._debug_logging:
            self.get_logger().info(
                '[imu debug] raw gyro bias '
                f'x={self._gyro_bias[0]:.1f}, y={self._gyro_bias[1]:.1f}, '
                f'z={self._gyro_bias[2]:.1f}'
            )

        self._imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        self._temperature_publisher = self.create_publisher(
            Temperature, 'imu/temperature', 10
        )
        self.create_timer(1.0 / publish_rate, self._publish_measurement)
        self.get_logger().info(
            f'MPU6050 ready on I2C bus {bus_number}, '
            f'address 0x{self._address:02x}'
        )

    def _initialize_sensor(self):
        self._bus.write_byte_data(
            self._address, self.POWER_MANAGEMENT_1, 0x01
        )
        time.sleep(0.1)
        identity = self._bus.read_byte_data(self._address, self.WHO_AM_I)
        if identity not in (0x68, 0x69):
            raise RuntimeError(
                f'Unexpected MPU6050 identity 0x{identity:02x}'
            )
        if self._debug_logging:
            self.get_logger().info(
                f'[imu debug] connection verified; '
                f'WHO_AM_I returned 0x{identity:02x}'
            )

        # 200 Hz internal sample rate, 44 Hz digital low-pass filter,
        # +/-250 deg/s gyroscope range, and +/-2 g accelerometer range.
        self._bus.write_byte_data(
            self._address, self.SAMPLE_RATE_DIVIDER, 0x04
        )
        self._bus.write_byte_data(
            self._address, self.CONFIGURATION, 0x03
        )
        self._bus.write_byte_data(
            self._address, self.GYROSCOPE_CONFIG, 0x00
        )
        self._bus.write_byte_data(
            self._address, self.ACCELEROMETER_CONFIG, 0x00
        )

    def _read_raw(self):
        block = self._bus.read_i2c_block_data(
            self._address, self.DATA_START, 14
        )
        return struct.unpack('>hhhhhhh', bytes(block))

    def _calibrate_gyroscope(self, sample_count):
        if sample_count <= 0:
            return (0.0, 0.0, 0.0)
        self.get_logger().info(
            f'Calibrating gyro with {sample_count} samples; keep robot still'
        )
        sums = [0.0, 0.0, 0.0]
        for _ in range(sample_count):
            _, _, _, _, gyro_x, gyro_y, gyro_z = self._read_raw()
            sums[0] += gyro_x
            sums[1] += gyro_y
            sums[2] += gyro_z
            time.sleep(0.005)
        return tuple(total / sample_count for total in sums)

    def _publish_measurement(self):
        try:
            accel_x, accel_y, accel_z, temperature, gyro_x, gyro_y, gyro_z = (
                self._read_raw()
            )
        except OSError as error:
            self.get_logger().error(
                f'MPU6050 I2C read failed: {error}',
                throttle_duration_sec=2.0,
            )
            return

        gyro_scale = DEGREES_TO_RADIANS / 131.0
        accel_scale = GRAVITY / 16384.0
        angular_velocity = [
            (raw - bias) * gyro_scale
            for raw, bias in zip(
                (gyro_x, gyro_y, gyro_z), self._gyro_bias
            )
        ]
        linear_acceleration = [
            raw * accel_scale for raw in (accel_x, accel_y, accel_z)
        ]
        stamp = self.get_clock().now().to_msg()

        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.orientation_covariance[0] = -1.0
        (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ) = angular_velocity
        (
            message.linear_acceleration.x,
            message.linear_acceleration.y,
            message.linear_acceleration.z,
        ) = linear_acceleration
        gyro_variance = self.get_parameter('gyro_stddev').value ** 2
        accel_variance = self.get_parameter('accel_stddev').value ** 2
        for index in (0, 4, 8):
            message.angular_velocity_covariance[index] = gyro_variance
            message.linear_acceleration_covariance[index] = accel_variance
        self._imu_publisher.publish(message)

        temperature_message = Temperature()
        temperature_message.header.stamp = stamp
        temperature_message.header.frame_id = self._frame_id
        temperature_message.temperature = temperature / 340.0 + 36.53
        self._temperature_publisher.publish(temperature_message)
        if self._debug_logging:
            self.get_logger().info(
                '[imu debug] '
                f'accel=({linear_acceleration[0]:.2f}, '
                f'{linear_acceleration[1]:.2f}, '
                f'{linear_acceleration[2]:.2f}) m/s^2, '
                f'gyro=({angular_velocity[0]:.3f}, '
                f'{angular_velocity[1]:.3f}, '
                f'{angular_velocity[2]:.3f}) rad/s, '
                f'temperature={temperature_message.temperature:.1f} C',
                throttle_duration_sec=1.0,
            )

    def destroy_node(self):
        """Close the I2C device when shutting down."""
        self._bus.close()
        return super().destroy_node()


def main(args=None):
    """Run the MPU6050 ROS driver."""
    rclpy.init(args=args)
    node = None
    try:
        node = Mpu6050Driver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
