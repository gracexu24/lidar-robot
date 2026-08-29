# Four-wheel ROS 2 LiDAR robot

ROS 2 Jazzy software for a Raspberry Pi 5, four STEP/DIR stepper drivers, an
MPU6050 IMU, and the Waveshare D500 kit (LDROBOT LD19 2D LiDAR). The package
supports physical hardware, keyboard driving, SLAM mapping, saved-map Nav2
navigation, and Gazebo Harmonic simulation.

## Hardware safety and wiring

The Raspberry Pi GPIO pins must connect to stepper-driver logic inputs, not
directly to motors. Use one correctly sized driver per motor, an external motor
power supply, current limiting, and a common ground between the supply, drivers,
and Pi. Never power motors from a Pi 5 V or 3.3 V pin.

Raise the wheels for initial tests and keep a physical emergency motor-power
disconnect within reach.

The default GPIO order is left-front, left-rear, right-front, right-rear:

- STEP: BCM `[14, 23, 17, 5]`
- DIR: BCM `[15, 24, 27, 6]`
- RESET: BCM `[4, 25, 22, 26]`
- SLEEP: BCM `[12, 7, 16, 1]`

These are BCM numbers, not physical header-pin numbers. Change them in
`my_robot/config/motors.yaml` to match the actual wiring.

GPIO7 is SPI CE1 and is unavailable while SPI is enabled. Either choose another
GPIO and move that SLEEP wire, or disable SPI in `/boot/firmware/config.txt` if
no other device uses it. GPIO14 and GPIO15 are UART pins, so the serial console
must not use them. GPIO1 is commonly reserved for HAT identification.

The code assumes active-low RESET and SLEEP, as used by A4988-style drivers. It
pulses RESET during initialization, leaves RESET high, and pulls SLEEP low when
a wheel is stopped. Verify these levels against the exact driver datasheet.

## Install

Use 64-bit Ubuntu 24.04 and ROS 2 Jazzy. Raspberry Pi OS is not a supported
binary platform for Jazzy.

Install the common ROS packages:

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard \
  python3-rosdep
```

On the Raspberry Pi, install the hardware packages:

```bash
sudo apt install -y python3-gpiozero python3-lgpio python3-smbus2 i2c-tools
```

On a desktop used for Gazebo, install the Gazebo integration:

```bash
sudo apt install -y ros-jazzy-ros-gz
```

Initialize rosdep once. If it says rosdep is already initialized, continue:

```bash
sudo rosdep init
rosdep update
```

Create the workspace and clone both the robot and vendor LiDAR driver:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/gracexu24/lidar-robot.git
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
```

Install dependencies and build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Rebuild after changing source, launch, configuration, URDF, or simulation files.

## Terminal setup

Run these commands in every new terminal before using this workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Confirm that ROS can find the package:

```bash
ros2 pkg prefix my_robot
```

## Shared operating commands

### Keyboard control

Keyboard control requires a robot or simulation launch to be running. In a
separate sourced terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

Useful keys:

- `i`: forward
- `,`: reverse
- `j`: turn left
- `l`: turn right
- `k`: stop
- `q` and `z`: increase and decrease all speeds
- `w` and `x`: increase and decrease linear speed
- `e` and `c`: increase and decrease angular speed
- `Ctrl+C`: exit

The physical motor controller has a 0.5-second watchdog, so hold or repeat a
movement key. Press `k` before leaving the keyboard terminal.

### RViz setup

Start RViz when a launch does not start it:

```bash
rviz2
```

For basic simulation, use `odom` as the Fixed Frame. For mapping and navigation,
use `map`. Add these displays as needed:

- RobotModel
- TF
- LaserScan using `/scan`
- Map using `/map`
- Navigation 2 panel for autonomous goals

When using SSH, run RViz on another ROS 2 Jazzy computer on the same ROS domain,
use X11 forwarding, or use Foxglove.

### Save the active map

Keep the mapping launch running. In another sourced terminal, choose a filename
without an extension and run:

```bash
MAP_NAME=${MAP_NAME:-$HOME/maps/home}
mkdir -p "$(dirname "$MAP_NAME")"
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
ls -l "$MAP_NAME.yaml" "$MAP_NAME.pgm"
```

Both the YAML and image file must exist before stopping mapping. Change
`MAP_NAME` to `$HOME/maps/sim_home` when saving a simulation map.

## Physical hardware setup and tests

### Configure LiDAR serial access

Connect the D500 USB adapter and find its stable path:

```bash
ls -l /dev/serial/by-id/
```

Add the current user to the serial-access group, then log out and back in or
reboot:

```bash
sudo usermod -aG dialout "$USER"
groups
```

Use the `/dev/serial/by-id/...` path in launch commands. `/dev/ttyUSB0` is used
below for brevity. Do not use broad `chmod 666` or `chmod 777` permissions as a
permanent solution.

### Motor connection test

Raise all wheels and start only the physical motor controller:

```bash
ros2 launch my_robot base.launch.py \
  start_motors:=true start_lidar:=false use_imu:=false \
  debug_logging:=true
```

In another sourced terminal, issue one bounded command at a time:

```bash
ros2 run my_robot drive front --duration 1 --speed 0.05
ros2 run my_robot drive back --duration 1 --speed 0.05
ros2 run my_robot drive left --duration 1 --turn-speed 0.4
ros2 run my_robot drive right --duration 1 --turn-speed 0.4
```

The debug output reports GPIO initialization, target and actual pulse rates,
pulse counts, wake/sleep changes, and watchdog stops. Pulse counts prove that
software generated pulses; without encoders they do not prove that a motor
physically moved.

### Keyboard-only motor movement

Raise-test all wheels first. Terminal 1 starts the motor controller without the
LiDAR or IMU:

```bash
ros2 launch my_robot base.launch.py \
  start_motors:=true start_lidar:=false use_imu:=false \
  debug_logging:=true
```

In Terminal 2, run the command under **Keyboard control**.

### IMU connection test

The I2C scan must show address `68`, or `69` when AD0 is tied high:

```bash
ls -l /dev/i2c-1
i2cdetect -y 1
```

Keep the robot still and start only the MPU6050 node:

```bash
ros2 run my_robot mpu6050_driver --ros-args \
  --params-file "$(ros2 pkg prefix my_robot)/share/my_robot/config/mpu6050.yaml" \
  -p debug_logging:=true
```

In another sourced terminal:

```bash
timeout 5s ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/temperature --once
```

The data rate should be near 100 Hz. At rest, angular velocity should be near
zero and total acceleration near 9.8 m/s². Keep the robot still for the first
approximately 2.5 seconds while gyro bias is measured.

### LiDAR connection test

Start only the LiDAR:

```bash
ros2 launch my_robot base.launch.py \
  start_motors:=false start_lidar:=true use_imu:=false \
  lidar_port:=/dev/ttyUSB0 debug_logging:=true
```

In another sourced terminal:

```bash
ros2 topic info /scan --verbose
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once
```

The expected rate is about 10 Hz. A few `nan` values are normal when no valid
reflection is returned. Debug output reports the closest valid distance and
warns when the scan stream is missing or stale.

### Values to measure and tune

Use metres, radians, seconds, kilograms, and SI-derived units throughout the
configuration. Define `base_link` at the robot's rotation center: halfway
between the front and rear axles, on the left-to-right centerline, at axle
height. `base_footprint` is the point directly below it on the floor. ROS axes
are +X forward, +Y left, and +Z upward.

Record these physical measurements before calibration:

- loaded wheel radius: axle center to floor with the robot at operating weight
- left-to-right wheel separation: center of left tread to center of right tread
- wheelbase: front axle center to rear axle center
- overall body length, width, and height, including parts that can collide
- front, rear, left, and right extents measured from `base_link`
- floor-to-axle height
- robot total mass and approximate center of mass, if Gazebo should be realistic
- LiDAR sensing-center position and orientation relative to `base_link`
- IMU chip-center position and board orientation relative to `base_link`

Enter or tune the following values.

#### `my_robot/config/motors.yaml`

- `step_pins`, `dir_pins`, `reset_pins`, `sleep_pins`: actual BCM GPIO wiring
  in LF, LR, RF, RR order
- `direction_inverted`: one Boolean per wheel; tune with the wheels raised
- `wheel_radius`: loaded wheel radius in metres
- `wheel_separation`: begin with the measured tread-center separation, then
  tune it using a measured rotation because four-wheel skid steering scrubs
- `steps_per_revolution`: motor full steps × driver microsteps × gearbox ratio
- `max_step_rate`: highest reliable pulse rate in steps/s without missed steps
- `max_step_acceleration`: highest reliable ramp in steps/s² without stalling
- `command_timeout`: safety stop delay in seconds; normally leave at 0.5

The current controller sends one rate to both left wheels and another to both
right wheels. Wheelbase is not a separate kinematic parameter, but it affects
scrubbing and therefore the calibrated effective `wheel_separation`.

#### `my_robot/urdf/robot.urdf`

- body box `size`: measured length, width, and height in metres
- body visual `origin xyz`: body-box center relative to `base_link`
- `base_joint` Z: floor-to-`base_link` height, normally the loaded wheel radius
- `laser_joint xyz`: `base_link` to the LiDAR rotating/sensing center in metres
- `laser_joint rpy`: LiDAR roll, pitch, and yaw in radians
- `imu_joint xyz`: `base_link` to the center of the MPU6050 chip in metres
- `imu_joint rpy`: rotation needed to align IMU axes with the ROS robot axes

Common angles are 90° = 1.5708 rad and 180° = 3.1416 rad. Sensor positions must
be measured to the sensing center, not to an enclosure edge.

#### `my_robot/config/mpu6050.yaml`

- `i2c_bus`: bus reported by the system, normally `1`
- `i2c_address`: `0x68` or `0x69`, as reported by `i2cdetect`
- `gyro_stddev`: stationary angular-velocity standard deviation in rad/s
- `accel_stddev`: stationary linear-acceleration standard deviation in m/s²
- `publish_rate`: measured sustainable data rate; normally leave at 100 Hz
- `calibration_samples`: increase for a steadier bias estimate if startup time
  is acceptable; keep the robot still for all samples

The `frame_id` must remain the same as the URDF IMU link unless both are renamed
together.

#### `my_robot/config/nav2.yaml`

- robot collision shape in both local and global costmaps: use either
  `robot_radius` or a `footprint`, but not both
- `robot_radius`: center to the furthest collidable point plus a small margin
- rectangular `footprint`: ordered `[x, y]` corners measured from
  `base_footprint`, including wheels, bumpers, and sensor mounts
- `footprint_padding`: additional collision margin, typically 0.01–0.03 m
- `inflation_radius`: desired navigation clearance around obstacles
- `obstacle_max_range`: maximum range at which LiDAR returns are reliably marked
- `raytrace_max_range`: reliable clearing range, slightly greater than the
  obstacle range but not beyond useful LiDAR data
- `max_vel_x` and `max_vel_theta`: safe measured linear and angular speeds
- `acc_lim_x`, `decel_lim_x`, `acc_lim_theta`, `decel_lim_theta`: acceleration
  limits that do not cause skipped steps or tipping
- velocity-smoother limits: keep them consistent with the controller limits
- goal tolerances and progress timeout: tune only after odometry is calibrated

For a centered rectangular robot, the minimum enclosing radius is:

```text
robot_radius = sqrt((length / 2)² + (width / 2)²) + safety margin
```

A more accurate asymmetric footprint uses the measured extents:

```yaml
footprint: "[[front, left], [front, -right], [-rear, -right], [-rear, left]]"
footprint_padding: 0.02
```

Replace `front`, `rear`, `left`, and `right` with distances in metres, and use
the same footprint in both costmaps.

#### `my_robot/config/slam.yaml`

- `max_laser_range`: maximum distance that produces dependable measurements,
  not merely the advertised maximum
- `resolution`: map cell size in metres; 0.05 m is a normal starting point
- minimum travel distance and heading: tune only if maps update too often or
  fail to update during deliberate motion

#### `my_robot/worlds/test_room.sdf`

These values affect only Gazebo. Copy calibrated real dimensions when the
simulation should match the physical robot:

- body collision and visual length, width, height, and center
- wheel X/Y positions, radius, width, mass, and inertia
- `wheel_separation` and `wheel_radius` in the DiffDrive plugin
- robot mass, center of mass, and inertia estimates
- simulated LiDAR and IMU poses
- LiDAR minimum/maximum range, sample count, and update rate

The URDF and SDF are separate models; changing one does not update the other.

### Calibration procedure

1. Verify all GPIO assignments against the physical wiring.
2. Raise the wheels and tune `direction_inverted`.
3. Enter loaded wheel radius and calculated motor steps per revolution.
4. Find conservative step-rate and acceleration limits.
5. Calibrate straight distance, then effective wheel separation.
6. Enter body and sensor measurements in the URDF and Nav2 costmaps.
7. Validate IMU axes and noise values.
8. Copy the final geometry into the Gazebo SDF if simulation should match.

Mark a starting position and request a 1.0 m straight movement:

```bash
ros2 run my_robot drive front --duration 10 --speed 0.10
```

Correct the step calibration:

```text
new steps_per_revolution =
    old steps_per_revolution × expected distance ÷ measured distance
```

Next, command a slow measured rotation and correct turning:

```text
new wheel_separation =
    old wheel_separation × expected turn angle ÷ measured turn angle
```

### Map with the physical robot

Terminal 1 starts the robot, SLAM Toolbox, and Nav2:

```bash
ros2 launch my_robot mapping.launch.py \
  lidar_port:=/dev/ttyUSB0 use_imu:=false debug_logging:=true
```

Terminal 2 runs RViz using the shared RViz setup with Fixed Frame `map`.
Terminal 3 runs the shared keyboard-control command.

Drive slowly along boundaries, avoid rapid turns, and revisit the starting area
to create loop closures. Verify that walls remain sharp and aligned. In Terminal
4, follow **Save the active map** while mapping remains running.

Add `use_imu:=true` only after the IMU test succeeds. The MPU6050 improves
short-term turn estimates but has no magnetometer and cannot eliminate long-term
yaw drift.

### Navigate on a physical saved map

Stop mapping, then start saved-map navigation in Terminal 1:

```bash
ros2 launch my_robot navigation.launch.py \
  map:=$HOME/maps/home.yaml \
  lidar_port:=/dev/ttyUSB0 use_imu:=false debug_logging:=true
```

Terminal 2 runs RViz with Fixed Frame `map`. Use **2D Pose Estimate** to set the
robot's measured starting position and heading. Send a short unobstructed
**Nav2 Goal**, then increase the distance after verifying localization, obstacle
detection, stopping, and recovery behavior.

## Gazebo Harmonic simulation

Simulation replaces physical GPIO motors and serial sensors with:

- a four-wheel differential-drive model
- simulated odometry and TF
- a GPU LiDAR publishing `/scan`
- an IMU publishing `/imu/data`
- a bridge between Gazebo Transport and ROS 2 topics

It uses the same `/cmd_vel`, `/odom`, `/scan`, `/imu/data`, and TF interfaces as
the physical robot. It does not test GPIO timing, wiring, current limits, motor
stalling, wheel slip, or real sensor noise.

The robot is embedded in `worlds/test_room.sdf`; it is not spawned from the
URDF. Keep SDF wheel, body, and sensor dimensions synchronized with calibrated
physical dimensions if the simulation is intended to match the real robot.

### Basic simulation and keyboard movement

Terminal 1 starts Gazebo, the robot, bridge, transforms, and RViz:

```bash
ros2 launch my_robot simulation.launch.py start_rviz:=true
```

Terminal 2 runs the shared keyboard-control command. In RViz, use Fixed Frame
`odom`.

Terminal 3 can validate every simulated interface:

```bash
ros2 topic echo /clock --once
timeout 5s ros2 topic hz /scan
timeout 5s ros2 topic hz /odom
timeout 5s ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo odom base_footprint
```

For a machine without a graphical display:

```bash
ros2 launch my_robot simulation.launch.py headless:=true
```

### Create a map in simulation

Stop any existing Gazebo launch. Terminal 1 starts Gazebo, SLAM, Nav2, and RViz:

```bash
ros2 launch my_robot simulation_mapping.launch.py start_rviz:=true
```

Terminal 2 runs the shared keyboard-control command. Drive around the complete
room and revisit the starting area. In RViz, use Fixed Frame `map` and inspect
`/map` and `/scan`.

In Terminal 3, follow **Save the active map** with:

```bash
export MAP_NAME=$HOME/maps/sim_home
```

Stop the mapping launch only after `sim_home.yaml` and `sim_home.pgm` exist.

### Navigate on a simulated saved map

Stop simulated mapping so only one Gazebo instance runs. Terminal 1 starts the
same room with the saved map:

```bash
ros2 launch my_robot simulation_navigation.launch.py \
  map:=$HOME/maps/sim_home.yaml start_rviz:=true
```

In RViz, use Fixed Frame `map`, set the initial pose near the world origin with
**2D Pose Estimate**, and send a short **Nav2 Goal**. Verify that obstacles
appear in the costmaps, `/cmd_vel` is generated, the robot stops before
collisions, and recovery behaviors work. Keyboard control is optional during
navigation and publishes directly to `/cmd_vel`.

## Gazebo implementation

- `worlds/test_room.sdf` defines the room, obstacles, four-wheel robot,
  collisions, differential-drive plugin, GPU LiDAR, and IMU.
- `config/gazebo_bridge.yaml` bridges clock, commands, odometry, TF, scans, and
  IMU messages between Gazebo and ROS.
- `launch/simulation.launch.py` starts Gazebo, the bridge, transforms, and
  optional RViz.
- `launch/simulation_mapping.launch.py` composes simulation with SLAM and Nav2.
- `launch/simulation_navigation.launch.py` composes simulation with AMCL and
  saved-map Nav2.
- `urdf/robot.urdf` supplies the static ROS transform tree used by RViz and
  Nav2; the simulated physics model itself comes from the SDF.

`setup.py` installs all launch, configuration, URDF, and world files. If ROS
cannot find a simulation launch after pulling changes, rebuild and source the
workspace using the commands under **Install** and **Terminal setup**.

## Run tests

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select my_robot
colcon test-result --verbose
```

The tests include lint checks and mocked verification of motor sleep/reset
behavior.

## Code structure

- `base_controller.py`: converts `/cmd_vel` to wheel step rates, ramps commands,
  enforces the watchdog, and publishes open-loop wheel odometry.
- `stepper_hardware.py`: owns four GPIO pulse threads and driver state.
- `mpu6050_driver.py`: publishes I2C accelerometer, gyro, and temperature data.
- `lidar_reader.py`: reports LiDAR connection and scan diagnostics.
- `manual_drive.py`: provides bounded front, back, left, right, and stop tests.
- `base.launch.py`: starts physical motors, transforms, LiDAR, and optional EKF.
- `mapping.launch.py`: starts the physical base, SLAM Toolbox, and Nav2.
- `navigation.launch.py`: starts the physical base, map server, AMCL, and Nav2.
- `navigation_core.launch.py`: contains Nav2 servers shared by both modes.
- `simulation*.launch.py`, `gazebo_bridge.yaml`, and `test_room.sdf`: implement
  Gazebo movement, sensors, mapping, and navigation.

## Accuracy limitations

Physical odometry counts pulses sent to the stepper drivers. It cannot detect
skipped steps or wheel slip. SLAM can correct some drift, but dependable
navigation needs wheel encoders and preferably an IMU fused with
`robot_localization`.

Python on Linux does not provide hard real-time GPIO timing. For greater speed
and reliability, move pulse generation and encoder counting to a
microcontroller and exchange velocity and odometry with the Pi over micro-ROS
or serial.
