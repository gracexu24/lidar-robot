# Four-wheel ROS 2 LiDAR robot

ROS 2 Jazzy software for a Raspberry Pi 5, four STEP/DIR stepper drivers, and
the Waveshare D500 kit (LDROBOT LD19 2D LiDAR).

## Important hardware requirements

The Raspberry Pi GPIO pins must connect to **stepper driver logic inputs**, not
directly to motors. Use one suitable driver per motor (for example, a correctly
sized A4988/TMC/DRV-style driver), an external motor power supply, current
limiting, and a common ground between that supply, the drivers, and the Pi.
Never power motors from a Pi 5 V or 3.3 V pin. Test with the wheels raised and
keep a physical emergency power disconnect within reach.

The default BCM pin order is left-front, left-rear, right-front, right-rear:

| Signal | LF | LR | RF | RR |
|---|---:|---:|---:|---:|
| STEP | 17 | 5 | 23 | 12 |
| DIR | 27 | 6 | 24 | 16 |
| RESET | 22 | 13 | 25 | 20 |

These are example assignments. Change `my_robot/config/motors.yaml` to match
your wiring. RESET is treated like the sample: high enables each driver and low
disables it. If the driver's input is actually named ENABLE and is active-low,
the electrical behavior is different and the code must be changed.

## Install on the Raspberry Pi

Use 64-bit Ubuntu 24.04 and ROS 2 Jazzy. Raspberry Pi OS is not a binary
supported Jazzy platform.

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  python3-gpiozero python3-lgpio python3-smbus2 python3-rosdep i2c-tools

cd ~/lidar-robot
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Connect the D500 USB adapter. Prefer its stable name from
`ls -l /dev/serial/by-id/`; otherwise use `/dev/ttyUSB0`. Add your user to the
serial group and then log out and in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not use `chmod 666` as a permanent serial-port solution.

## Configure and test the wheels

Edit `my_robot/config/motors.yaml`:

- `direction_inverted`: change each value until a positive command rotates all
  wheels toward the robot's front.
- `wheel_radius`: loaded wheel radius in metres.
- `wheel_separation`: left-to-right wheel contact-line distance in metres.
- `steps_per_revolution`: motor full steps × microstep setting × gear ratio.
- `max_step_rate`: lower this if motors skip or stall.

Start only the base controller for the first wheel test:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lidar-robot/install/setup.bash
ros2 launch my_robot base.launch.py start_lidar:=false
```

In another sourced terminal, run a command. It publishes repeatedly for the
requested duration, then publishes stop:

```bash
ros2 run my_robot drive front --duration 1 --speed 0.05
ros2 run my_robot drive back --duration 1 --speed 0.05
ros2 run my_robot drive left --duration 1 --turn-speed 0.4
ros2 run my_robot drive right --duration 1 --turn-speed 0.4
ros2 run my_robot drive stop
```

The controller also accepts normal `geometry_msgs/msg/Twist` messages on
`/cmd_vel`. A 0.5-second watchdog stops the motors when commands disappear.

## Check the LiDAR

```bash
ros2 launch my_robot base.launch.py \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
ros2 topic echo /scan --once
```

The D500 uses the LD19 protocol at 230400 baud. `base.launch.py` starts the
official `ldlidar_stl_ros2_node` and publishes scans in `base_laser`.
Update the `laser_joint` position and yaw in `my_robot/urdf/robot.urdf` to the
measured LiDAR pose. A wrong pose or forward direction distorts maps.

## Optional MPU6050 IMU fusion

The included `mpu6050_driver` reads an MPU6050 directly from Raspberry Pi I2C.
With the Pi powered off, connect a typical 3.3 V MPU6050 breakout as follows:

- MPU6050 VCC to Pi 3.3 V, physical pin 1.
- MPU6050 GND to Pi ground, physical pin 6.
- MPU6050 SDA to GPIO2/SDA, physical pin 3.
- MPU6050 SCL to GPIO3/SCL, physical pin 5.
- Leave INT, XDA, and XCL disconnected.
- Tie AD0 low for address `0x68`; tie it high for `0x69`.

Using 3.3 V is safest. Do not use 5 V unless the exact breakout explicitly
includes both a regulator and safe I2C level shifting.

Enable I2C in `/boot/firmware/config.txt` by ensuring it contains
`dtparam=i2c_arm=on`, then reboot. Grant your user I2C access and check that the
sensor appears:

```bash
sudo usermod -aG i2c "$USER"
sudo reboot

ls /dev/i2c-1
sudo i2cdetect -y 1
```

The scan should show `68`, or `69` when AD0 is high. Set that address in
`my_robot/config/mpu6050.yaml`. A blank scan indicates a wiring, power, I2C
configuration, or defective-module problem.

Mount the module rigidly with its marked x axis forward, y axis left, and z axis
up. If it is mounted differently, set the measured rotation in `imu_joint` in
`my_robot/urdf/robot.urdf`.

Rebuild after adding the driver, then launch it by enabling IMU fusion:

```bash
cd ~/lidar-robot
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

ros2 launch my_robot base.launch.py use_imu:=true
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once

ros2 launch my_robot mapping.launch.py use_imu:=true \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE

ros2 launch my_robot navigation.launch.py use_imu:=true \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Keep the robot completely still for about 2.5 seconds whenever the driver
starts; it measures gyro bias during this period. The MPU6050 has no
magnetometer and cannot measure absolute yaw, so `config/ekf.yaml` fuses its
measured yaw rate, not an invented orientation. This improves short-term turns
but does not eliminate long-term heading drift.

When fusion is enabled, `robot_localization` alone publishes `/odom` and
`odom -> base_footprint`, preventing duplicate TF publishers. To use another
IMU driver instead, launch with `use_imu:=true start_mpu6050:=false` and make
that driver publish `sensor_msgs/msg/Imu` on `/imu/data`.

## Create a map with SLAM and Nav2

Raise-test manual motion first, then place the robot in a clear area:

```bash
ros2 launch my_robot mapping.launch.py \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Open RViz (`rviz2`), set the fixed frame to `map`, add Map and LaserScan
displays, then drive manually or select a Nav2 goal. Save a completed map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/home
```

## Navigate on a saved map

```bash
ros2 launch my_robot navigation.launch.py \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

In RViz, use **2D Pose Estimate** once to initialize localization, then use
**Nav2 Goal**. Start with low speeds in `config/nav2.yaml`.

## Code structure

- `base_controller.py`: converts `/cmd_vel` into left/right wheel speeds,
  ramps step rates, enforces the watchdog, and publishes wheel odometry.
- `stepper_hardware.py`: four GPIO pulse threads and driver reset handling.
- `mpu6050_driver.py`: I2C accelerometer, gyro, and temperature publisher.
- `manual_drive.py`: the `front`, `back`, `left`, `right`, and `stop` CLI.
- `base.launch.py`: motor node, robot transforms, D500 driver, and optional EKF.
- `mapping.launch.py`: base + SLAM Toolbox + Nav2.
- `navigation.launch.py`: base + map server + AMCL + Nav2.
- `motors.yaml`, `mpu6050.yaml`, `ekf.yaml`, `slam.yaml`, `nav2.yaml`: hardware
  and navigation tuning.

## Accuracy limitation that must be addressed

The supplied code estimates odometry by counting pulses sent to the drivers.
It cannot detect skipped motor steps or wheel slip. SLAM may correct some map
drift, but dependable autonomous navigation needs wheel encoders (and ideally
an IMU) fused with `robot_localization`. After adding encoders, publish measured
wheel odometry instead of this open-loop estimate. Python on Linux also does
not produce hard real-time GPIO timing; for higher speed/reliability, move pulse
generation and encoder counting to a microcontroller and exchange velocity and
odometry with the Pi over micro-ROS or serial.