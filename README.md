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
| STEP | 14 | 23 | 17 | 5 |
| DIR | 15 | 24 | 27 | 6 |
| RESET | 4 | 25 | 22 | 26 |
| SLEEP | 12 | 7 | 16 | 1 |

The matching defaults in
`~/ros2_ws/src/lidar-robot/my_robot/config/motors.yaml` are:

```yaml
base_controller:
  ros__parameters:
    step_pins: [14, 23, 17, 5]
    dir_pins: [15, 24, 27, 6]
    reset_pins: [4, 25, 22, 26]
    sleep_pins: [12, 7, 16, 1]
    direction_inverted: [true, true, false, false]
```

These are BCM GPIO numbers, not physical header pin numbers. GPIO14 and GPIO15
are also UART pins, so the serial console/UART must not be using them. GPIO1 is
normally reserved for HAT identification; use it only when nothing attached to
the Pi needs that interface.

The code assumes active-low RESET and SLEEP inputs, as used by A4988-style
drivers. It pulses RESET once during initialization and otherwise leaves it
high. It stops STEP pulses and pulls SLEEP low whenever a wheel is stopped,
including watchdog and shutdown stops. Before movement it raises SLEEP and
waits for the driver to wake. Confirm these input levels against the exact
driver datasheet before applying power.

## Install on the Raspberry Pi

need ros2 and rosdep 

Use 64-bit Ubuntu 24.04 and ROS 2 Jazzy. Raspberry Pi OS is not a binary
supported Jazzy platform. For a new machine, initialize rosdep once with
`sudo rosdep init`; if it reports that rosdep is already initialized, continue.

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  python3-gpiozero python3-lgpio python3-smbus2 python3-rosdep i2c-tools

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gracexu24/lidar-robot.git
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Connect the D500 USB adapter. Prefer its stable name from
`ls -l /dev/serial/by-id/`; otherwise use `/dev/ttyUSB0`. Add your user to the
serial group and then log out and in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not use `chmod 666` as a permanent serial-port solution.

## Connection and debug checks

Rebuild and source the workspace after changing this repository:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 doctor --report
```

Set `debug_logging:=true` on a launch command to print motor GPIO
initialization, wake/sleep, watchdog, commanded and actual pulse rates,
MPU6050 connection and live measurements, and LiDAR connection and scan
statistics. Debug messages are throttled so they do not print at sensor or
control-loop rate.

### Motor test

Raise all wheels off the floor and keep the emergency power disconnect within
reach. Start the motor controller without the LiDAR:

```bash
ros2 launch my_robot base.launch.py \
  start_lidar:=false debug_logging:=true
```

Expected startup output lists all configured GPIOs and reports that RESET
completed while each driver is sleeping. In another sourced terminal, run one
low-speed command at a time:

```bash
ros2 run my_robot drive front --duration 1 --speed 0.05
ros2 run my_robot drive back --duration 1 --speed 0.05
ros2 run my_robot drive left --duration 1 --turn-speed 0.4
ros2 run my_robot drive right --duration 1 --turn-speed 0.4
```

For each command, debug output should show the target rates, actual pulse rates
and counts, `SLEEP high`, then `SLEEP low` after stopping. All four wheels must
physically move in the named direction without skipping. The software can prove
that pulses were generated, but there are no wheel encoders, so it cannot prove
that a disconnected or stalled motor actually turned. Use a multimeter or logic
analyzer if needed: SLEEP should be low while stopped and high while moving;
RESET should remain high after its short startup pulse.

### IMU test

First verify the kernel can see the device. The output must contain `68` (AD0
low) or `69` (AD0 high):

```bash
ls -l /dev/i2c-1
i2cdetect -y 1
```

Keep the robot still and run only the MPU6050 node:

```bash
ros2 run my_robot mpu6050_driver --ros-args \
  --params-file "$(ros2 pkg prefix my_robot)/share/my_robot/config/mpu6050.yaml" \
  -p debug_logging:=true
```

Expected output includes a valid `WHO_AM_I`, calibration completion, gyro bias,
and live measurements. In another sourced terminal:

```bash
timeout 5s ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/temperature --once
```

The rate should be near 100 Hz. At rest, angular velocity should be near zero
and total acceleration should be near 9.8 m/s². Rotate the sensor by hand and
confirm the corresponding angular-velocity axis changes. I2C read errors,
an unexpected identity, or no topic messages indicate a connection,
permissions, address, or configuration problem.

### LiDAR test

Find the stable serial device and verify that the current user can access it:

```bash
ls -l /dev/serial/by-id/
test -r /dev/serial/by-id/YOUR_D500_DEVICE && \
  test -w /dev/serial/by-id/YOUR_D500_DEVICE && echo "serial access OK"
```

Start only the LiDAR side of the base launch:

```bash
ros2 launch my_robot base.launch.py start_motors:=false \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE debug_logging:=true
```

In another sourced terminal:

```bash
ros2 topic info /scan --verbose
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once
```

The scan topic must have a publisher, a stable message rate, finite ranges, and
changing closest-distance and angle output when an object moves in front of the
sensor. The debug reader starts automatically when debug logging is enabled and
warns if `/scan` never arrives or stops. If the serial device exists but no
scans arrive, check the `dialout` group, 230400 baud support, the selected
device path, USB power, and the launch log.

## Configure and test the wheels

Edit `~/ros2_ws/src/lidar-robot/my_robot/config/motors.yaml`:

- `direction_inverted`: change each value until a positive command rotates all
  wheels toward the robot's front.
- `step_pins`, `dir_pins`, `reset_pins`, and `sleep_pins`: four BCM GPIO
  numbers in LF, LR, RF, RR order.
- `wheel_radius`: loaded wheel radius in metres.
- `wheel_separation`: left-to-right wheel contact-line distance in metres.
- `steps_per_revolution`: motor full steps × microstep setting × gear ratio.
- `max_step_rate`: lower this if motors skip or stall.

Start only the base controller for the first wheel test:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot base.launch.py start_lidar:=false
```

In another sourced terminal, run a command. It publishes repeatedly for the
requested duration, then publishes stop:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
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
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot base.launch.py \
  start_motors:=false \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

While that launch command is running, use another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /scan --once
```

The D500 uses the LD19 protocol at 230400 baud. `base.launch.py` starts the
official `ldlidar_stl_ros2_node`, publishes `/scan`, and sets its frame ID to
`base_laser`. Update the `laser_joint` position and yaw in
`my_robot/urdf/robot.urdf` to the measured LiDAR pose. A wrong pose or forward
direction distorts maps.

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
```

After the reboot:

```bash
ls /dev/i2c-1
i2cdetect -y 1
```

The scan should show `68`, or `69` when AD0 is high. Set that address in
`my_robot/config/mpu6050.yaml`. A blank scan indicates a wiring, power, I2C
configuration, or defective-module problem.

Mount the module rigidly with its marked x axis forward, y axis left, and z axis
up. If it is mounted differently, set the measured rotation in `imu_joint` in
`my_robot/urdf/robot.urdf`.

Rebuild after code or dependency changes, then launch with IMU fusion enabled:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash

ros2 launch my_robot base.launch.py use_imu:=true
```

While the launch command is running, check its topics in another sourced
terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
timeout 5s ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once
```

Run either mapping:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot mapping.launch.py use_imu:=true \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Or saved-map navigation:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot navigation.launch.py use_imu:=true \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Keep the robot completely still for about 2.5 seconds whenever the driver
starts; it measures gyro bias during this period. The MPU6050 has no
magnetometer and cannot measure absolute yaw, so `my_robot/config/ekf.yaml`
fuses its
measured yaw rate, not an invented orientation. This improves short-term turns
but does not eliminate long-term heading drift.

When fusion is enabled, `robot_localization` alone publishes `/odom` and
`odom -> base_footprint`, preventing duplicate TF publishers. To use another
IMU driver instead, launch with `use_imu:=true start_mpu6050:=false` and make
that driver publish `sensor_msgs/msg/Imu` on `/imu/data`.

## Create a map with SLAM and Nav2

Raise-test manual motion first, then place the robot in a clear area:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot mapping.launch.py \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Open RViz (`rviz2`), set the fixed frame to `map`, add Map and LaserScan
displays, then drive manually or select a Nav2 goal. Keep the mapping launch
running while saving a completed map from another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/home
```

## Navigate on a saved map

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot navigation.launch.py \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

In RViz, use **2D Pose Estimate** once to initialize localization, then use
**Nav2 Goal**. Start with low speeds in `my_robot/config/nav2.yaml`.

## Run package tests

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select my_robot
colcon test-result --verbose
```

The tests include lint checks and mocked verification that routine motor stops
use SLEEP without reasserting RESET.

## Code structure

- `base_controller.py`: converts `/cmd_vel` into left/right wheel speeds,
  ramps step rates, enforces the watchdog, and publishes wheel odometry.
- `stepper_hardware.py`: four GPIO pulse threads and driver sleep/reset
  handling.
- `mpu6050_driver.py`: I2C accelerometer, gyro, and temperature publisher.
- `lidar_reader.py`: diagnostic subscriber that reports LiDAR scan statistics.
- `manual_drive.py`: the `front`, `back`, `left`, `right`, and `stop` CLI.
- `base.launch.py`: motor node, robot transforms, D500 driver, and optional EKF.
- `mapping.launch.py`: base + SLAM Toolbox + Nav2.
- `navigation.launch.py`: base + map server + AMCL + Nav2.
- `navigation_core.launch.py`: Nav2 servers shared by mapping and navigation.
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