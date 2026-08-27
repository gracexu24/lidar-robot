# Four-wheel ROS 2 LiDAR robot

ROS 2 Jazzy software for a Raspberry Pi 5 robot with:

- Four independently driven STEP/DIR stepper motors
- Four-wheel skid-steer/differential-drive motion
- Waveshare D500 / LDROBOT LD19 2D LiDAR
- Optional MPU6050 IMU over I²C
- SLAM Toolbox mapping
- Nav2 saved-map navigation

## How the system works

The main data flow is:

```text
Manual command or Nav2
          |
       /cmd_vel
          |
   base_controller
          |
  four STEP/DIR rates
          |
     motor drivers
```

The localization and navigation flow is:

```text
Motor pulse counts -> /wheel/odom --+
                                    +-> robot_localization EKF -> /odom
MPU6050 gyro ------> /imu/data -----+

D500 LiDAR -> /scan -> SLAM Toolbox or AMCL -> Nav2 -> /cmd_vel
```

Without IMU fusion, the base controller publishes `/odom` and the
`odom -> base_footprint` transform directly. With `use_imu:=true`,
`robot_localization` combines wheel odometry and MPU6050 yaw rate and becomes
the only publisher of that filtered odometry and transform.

The TF frame chain is:

```text
map -> odom -> base_footprint -> base_link -> base_laser
                                         \-> imu_link
```

## Safety and motor-driver assumptions

GPIO pins connect only to stepper-driver logic inputs. Never connect a motor
directly to Raspberry Pi GPIO or power a motor from the Pi's 3.3 V or 5 V
pins. Use:

- One correctly sized driver per motor
- A separate motor power supply
- Proper driver current limiting and cooling
- A common ground between the Pi, drivers, and motor supply
- A physical emergency power disconnect

Always perform initial tests with the wheels raised.

The code assumes A4988-style active-low RESET and SLEEP inputs. RESET is used
during startup; SLEEP is held low while a wheel is stopped and raised before
movement. Verify these signal levels against your exact driver datasheet.

## Default motor wiring

All values below are BCM GPIO numbers in left-front, left-rear, right-front,
right-rear order:

```text
STEP:  14, 23, 17, 5
DIR:   15, 24, 27, 6
RESET:  4, 25, 22, 26
SLEEP: 12,  7, 16, 1
```

The configuration is in `my_robot/config/motors.yaml`.

GPIO14 and GPIO15 are UART pins, so disable the serial console before using
them as motor GPIO. GPIO1 is normally reserved for HAT identification; change
that assignment if a HAT or EEPROM interface needs it.

## MPU6050 wiring

Power off the Pi before wiring:

- VCC -> Pi 3.3 V, physical pin 1
- GND -> Pi ground, physical pin 6
- SDA -> GPIO2/SDA, physical pin 3
- SCL -> GPIO3/SCL, physical pin 5
- AD0 low/GND -> I²C address `0x68`
- AD0 high/3.3 V -> I²C address `0x69`
- Leave INT, XDA, and XCL disconnected

Using 3.3 V is safest. Do not use 5 V unless the exact breakout includes safe
regulation and I²C level shifting.

Ensure `/boot/firmware/config.txt` contains:

```text
dtparam=i2c_arm=on
```

Then reboot and verify the device:

```bash
ls /dev/i2c-1
sudo i2cdetect -y 1
```

Set the detected address in `my_robot/config/mpu6050.yaml`.

## Install

Use 64-bit Ubuntu 24.04 and ROS 2 Jazzy. Configure the official ROS 2 apt
repository first, then install the required packages:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-desktop \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  python3-gpiozero \
  python3-lgpio \
  python3-smbus2 \
  python3-rosdep \
  i2c-tools
```

Initialize rosdep once:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` says it is already initialized, continue.

Create and build the workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gracexu24/lidar-robot.git
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Grant hardware access, then log out and back in:

```bash
sudo usermod -aG dialout,i2c "$USER"
```

Use `/dev/serial/by-id/...` for the LiDAR when possible because it is more
stable than `/dev/ttyUSB0`.

## Configure the robot

Edit `my_robot/config/motors.yaml`:

- `direction_inverted`: reverse individual logical wheel directions.
- `wheel_radius`: loaded wheel radius in metres.
- `wheel_separation`: distance between the left and right wheel contact lines.
- `steps_per_revolution`: full motor steps × microsteps × gearbox ratio.
- `max_step_rate`: maximum STEP pulses per second.
- `max_step_acceleration`: pulse-rate change per second.
- `command_timeout`: stop after this many seconds without `/cmd_vel`.

Edit `my_robot/urdf/robot.urdf`:

- Set `base_joint` to the measured chassis height above the floor.
- Set `laser_joint` to the measured LiDAR position and orientation.
- Set `imu_joint` to the measured MPU6050 position and orientation.

ROS robot axes are x forward, y left, and z up.

Edit `my_robot/config/nav2.yaml`:

- Set `robot_radius` to fully contain the robot.
- Keep initial velocity and acceleration limits low.
- Tune inflation and obstacle ranges only after `/scan` and odometry work.

## Terminal preparation

Every new terminal used below must source ROS and this workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

After changing Python, launch, YAML, URDF, or package files, rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Test 1: package tests

This test does not require connected hardware:

```bash
cd ~/ros2_ws
colcon test --packages-select my_robot
colcon test-result --verbose
```

It runs Python lint checks and a mocked GPIO test for RESET/SLEEP behavior.

## Test 2: motors

Raise all wheels and start only the motor controller:

```bash
ros2 launch my_robot base.launch.py \
  start_lidar:=false \
  debug_logging:=true
```

In another sourced terminal, test one command at a time:

```bash
ros2 run my_robot drive front --duration 1 --speed 0.05
ros2 run my_robot drive back --duration 1 --speed 0.05
ros2 run my_robot drive left --duration 1 --turn-speed 0.4
ros2 run my_robot drive right --duration 1 --turn-speed 0.4
ros2 run my_robot drive stop
```

All wheels should move in the named direction without skipping. Adjust
`direction_inverted` if an individual wheel turns backward.

You can also publish a standard ROS velocity command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Press Ctrl+C to stop publishing. The command watchdog then sets the target
speed to zero.

## Test 3: MPU6050

The I²C scan must show `68` or `69`:

```bash
sudo i2cdetect -y 1
```

Keep the robot completely still while starting the driver. Its 500-sample
gyro calibration takes approximately 2.5 seconds:

```bash
ros2 run my_robot mpu6050_driver --ros-args \
  --params-file "$(ros2 pkg prefix my_robot)/share/my_robot/config/mpu6050.yaml" \
  -p debug_logging:=true
```

Check the topics from another sourced terminal:

```bash
timeout 5s ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/temperature --once
```

The publication rate should be close to 100 Hz. At rest, angular velocity
should be near zero and acceleration magnitude should be near 9.8 m/s².

The MPU6050 has no magnetometer, so it cannot measure absolute yaw. The EKF
uses only its measured z-axis angular velocity.

## Test 4: D500 LiDAR and RViz

Locate the serial device:

```bash
ls -l /dev/serial/by-id/
```

Start the LiDAR without activating the motors:

```bash
ros2 launch my_robot base.launch.py \
  start_motors:=false \
  use_imu:=false \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE \
  debug_logging:=true
```

Check scan delivery:

```bash
ros2 topic info /scan --verbose
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once
```

Open the viewer:

```bash
rviz2
```

In RViz:

1. Set **Fixed Frame** to `base_link`.
2. Select **Add**, then **By topic**.
3. Add the `/scan` LaserScan display.
4. Optionally add RobotModel and TF displays.

Verify the static LiDAR pose:

```bash
ros2 run tf2_ros tf2_echo base_link base_laser
```

This prints the translation and rotation of `base_laser` relative to
`base_link`. It does not move or configure the robot.

## Test 5: IMU and wheel-odometry fusion

Start motors and the MPU6050 without the LiDAR:

```bash
ros2 launch my_robot base.launch.py \
  start_lidar:=false \
  use_imu:=true \
  debug_logging:=true
```

Check the raw and filtered inputs:

```bash
ros2 topic echo /wheel/odom --once
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

`robot_localization` reads `/wheel/odom` and `/imu/data`, then publishes
filtered `/odom` and `odom -> base_footprint`.

## Test 6: create and save a map

Start with low speeds in a clear indoor area:

```bash
ros2 launch my_robot mapping.launch.py \
  use_imu:=true \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

Open RViz, set **Fixed Frame** to `map`, and add Map, LaserScan, RobotModel,
and TF. Drive manually or send Nav2 goals while the map is built.

Save the completed map from another sourced terminal:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/home
```

This creates `home.yaml` and `home.pgm`.

## Test 7: navigate on a saved map

```bash
ros2 launch my_robot navigation.launch.py \
  use_imu:=true \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/serial/by-id/YOUR_D500_DEVICE
```

In RViz:

1. Set the fixed frame to `map`.
2. Use **2D Pose Estimate** to initialize AMCL.
3. Use **Nav2 Goal** to select a destination.

The current Nav2 configuration provides basic reactive LiDAR obstacle
avoidance through the local costmap and DWB controller. It does not predict
moving-object trajectories and is not safety-rated around people.

## Debugging

Add `debug_logging:=true` to base, mapping, or navigation launch commands to
print throttled motor, IMU, and LiDAR diagnostics.

Useful commands:

```bash
ros2 doctor --report
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

If LiDAR data is missing, check serial permissions, the selected serial path,
USB power, and whether the LD19 driver started successfully.

If IMU data is missing, check `/dev/i2c-1`, address `0x68`/`0x69`, wiring,
permissions, and whether the robot remained still during startup.

If motors do not move, verify the external motor supply, common ground,
driver current limit, SLEEP level, GPIO assignments, and STEP pulses with a
logic analyzer.

## Code overview

- `my_robot/my_robot/base_controller.py`: receives `/cmd_vel`, calculates
  differential-drive wheel rates, ramps acceleration, commands GPIO hardware,
  and estimates odometry from generated pulses.
- `my_robot/my_robot/stepper_hardware.py`: controls each driver's STEP, DIR,
  RESET, and SLEEP pins in a background pulse thread.
- `my_robot/my_robot/mpu6050_driver.py`: reads the MPU6050 over I²C, calibrates
  gyro bias, and publishes acceleration, angular velocity, and temperature.
- `my_robot/my_robot/manual_drive.py`: publishes bounded front, back, left,
  right, and stop commands.
- `my_robot/my_robot/lidar_reader.py`: optional diagnostic subscriber for
  `/scan`.
- `my_robot/launch/base.launch.py`: starts hardware nodes, robot transforms,
  optional EKF fusion, and diagnostics.
- `my_robot/launch/mapping.launch.py`: starts the physical robot, online SLAM,
  and Nav2.
- `my_robot/launch/navigation.launch.py`: starts the physical robot, saved-map
  server, AMCL, and Nav2.
- `my_robot/launch/navigation_core.launch.py`: starts shared Nav2 planning,
  control, behavior, smoothing, and lifecycle nodes.
- `my_robot/config/motors.yaml`: GPIO and drivetrain calibration.
- `my_robot/config/mpu6050.yaml`: I²C and IMU publication settings.
- `my_robot/config/ekf.yaml`: wheel/IMU fusion selection.
- `my_robot/config/slam.yaml`: SLAM frames, scan matching, resolution, and loop
  closure.
- `my_robot/config/nav2.yaml`: AMCL, planner, DWB controller, costmaps,
  recovery behaviors, and velocity limits.
- `my_robot/urdf/robot.urdf`: chassis and sensor frames used by TF.
- `my_robot/setup.py` and `package.xml`: Python executables, installed data,
  ROS dependencies, and package metadata.

## Known limitations

- Wheel odometry counts generated pulses, not measured wheel rotation.
- Missed steps and wheel slip are not detected because there are no encoders.
- MPU6050 yaw rate improves short-term turns but still drifts over time.
- Python/Linux GPIO timing is not hard real-time.
- The 2D LiDAR only detects obstacles intersecting its scan plane.
- Navigation and obstacle avoidance are not safety-rated.

For more reliable autonomous operation, add wheel encoders and fuse measured
encoder odometry with the IMU. For higher pulse rates, move motor pulse
generation and encoder counting to a microcontroller.
