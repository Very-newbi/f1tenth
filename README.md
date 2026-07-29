[README.md](https://github.com/user-attachments/files/30489320/README.md)
# ROS2 Foxy Disparity Extender + AEB

This package is for:

```text
Ubuntu 20.04
ROS2 Foxy
Jetson
Livox Mid-360
VESC direct command topics
```

Package name:

```text
my_algo
```

Main idea:

```text
LiDAR scan
  -> disparity extender planner
  -> planner motor/servo command
  -> AEB safety mux
  -> final VESC motor/servo command
```

## Folder Structure

Put the package here on the Jetson:

```text
~/f1tenth/src/my_algo
```

or, if your workspace is named `ros2_ws`:

```text
~/ros2_ws/src/my_algo
```

Package files:

```text
src/my_algo/
  package.xml
  setup.py
  setup.cfg
  resource/my_algo

  my_algo/
    __init__.py
    vesc_utils.py
    disparity_extender.py
    aeb_mux.py

  config/
    disparity_extender.yaml
    aeb.yaml

  launch/
    real_launch.py
    scan_launch.py
```

## Build

Example for workspace `~/f1tenth`:

```bash
cd ~/f1tenth
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If your workspace is `~/ros2_ws`, use:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Required ROS package for `real_launch.py`:

```bash
sudo apt update
sudo apt install ros-foxy-pointcloud-to-laserscan
```

## Which Launch File To Use

Use `real_launch.py` when the Livox driver publishes 3D point cloud only:

```text
/livox/lidar  sensor_msgs/msg/PointCloud2
```

Command:

```bash
ros2 launch my_algo real_launch.py points_topic:=/livox/lidar
```

Use `scan_launch.py` when `/scan` already exists:

```text
/scan  sensor_msgs/msg/LaserScan
```

Command:

```bash
ros2 launch my_algo scan_launch.py scan_topic:=/scan
```

Quick decision:

```bash
ros2 topic list
```

```text
/livox/lidar exists, /scan does not exist -> real_launch.py
/scan already exists                     -> scan_launch.py
```

## Recommended Terminal Layout

Terminal 1: Livox Mid-360 driver

```bash
cd ~/ws_livox
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 YOUR_LIVOX_LAUNCH.py
```

Check:

```bash
ros2 topic hz /livox/lidar
```

Terminal 2: VESC driver

```bash
cd ~/vesc_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch vesc_driver vesc_driver_node.launch.py
```

Check:

```bash
ros2 topic list | grep vesc
ros2 topic hz /vesc/odom
```

Terminal 3: autonomous driving stack

If using Livox PointCloud2:

```bash
cd ~/f1tenth
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch my_algo real_launch.py points_topic:=/livox/lidar
```

If `/scan` already exists:

```bash
cd ~/f1tenth
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch my_algo scan_launch.py scan_topic:=/scan
```

Terminal 4: enable autonomous mode

```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub /autonomous_mode std_msgs/msg/Bool "{data: true}" -r 5
```

The planner will not drive unless:

```text
/autonomous_mode == true
/joy_active == false
```

For quick bench testing without the autonomous-mode gate:

```bash
ros2 launch my_algo real_launch.py \
  points_topic:=/livox/lidar \
  require_autonomous_mode:=false
```

## real_launch.py Nodes

`real_launch.py` starts four things:

```text
1. livox_static_tf
2. pointcloud_to_laserscan
3. disparity_extender
4. aeb_mux
```

### 1. livox_static_tf

Package:

```text
tf2_ros
```

Executable:

```text
static_transform_publisher
```

Purpose:

```text
Publishes the fixed transform between base_link and livox_frame.
This is used so PointCloud2 can be projected into base_link.
```

Default transform:

```text
base_frame:  base_link
lidar_frame: livox_frame
lidar_x:     0.0
lidar_y:     0.0
lidar_z:     0.20
lidar_yaw:  -1.5708
lidar_pitch: 0.2618
lidar_roll:  0.0
```

If RViz shows the scan tilted the wrong way, try:

```bash
ros2 launch my_algo real_launch.py \
  points_topic:=/livox/lidar \
  lidar_pitch:=-0.2618
```

If left/right or front/back is wrong, try:

```bash
ros2 launch my_algo real_launch.py \
  points_topic:=/livox/lidar \
  lidar_yaw:=1.5708
```

### 2. pointcloud_to_laserscan

Package:

```text
pointcloud_to_laserscan
```

Executable:

```text
pointcloud_to_laserscan_node
```

Subscribe:

```text
cloud_in -> /livox/lidar  sensor_msgs/msg/PointCloud2
```

Publish:

```text
scan -> /scan  sensor_msgs/msg/LaserScan
```

Important parameters in `real_launch.py`:

```yaml
target_frame: base_link
min_height: -0.05
max_height: 0.40
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.00436
range_min: 0.05
range_max: 20.0
```

If the car sees the floor as an obstacle, raise `min_height`:

```yaml
min_height: 0.00
```

If it misses low obstacles, lower `min_height` slightly:

```yaml
min_height: -0.08
```

### 3. disparity_extender

Package:

```text
my_algo
```

Executable:

```text
disparity_extender
```

Subscribe:

```text
/scan             sensor_msgs/msg/LaserScan
/joy_active       std_msgs/msg/Bool
/autonomous_mode  std_msgs/msg/Bool
```

Publish:

```text
/planner/motor/speed      std_msgs/msg/Float64  # ERPM
/planner/servo/position   std_msgs/msg/Float64  # servo position, 0.0 to 1.0
```

Purpose:

```text
Reads /scan.
Finds large distance jumps between neighboring LaserScan beams.
Inflates obstacle edges by car width + safety margin.
Chooses a far safe target direction.
Publishes planner motor ERPM and servo position.
```

Config file:

```text
config/disparity_extender.yaml
```

Important tuning parameters:

```yaml
disparity_threshold_m: 0.45  # distance jump that counts as an obstacle edge
car_width_m: 0.31
safety_margin_m: 0.12
max_speed_mps: 2.5
base_speed_mps: 1.6
min_speed_mps: 0.6
servo_center: 0.5
servo_gain: 0.28
erpm_gain: 4614.0
```

### 4. aeb_mux

Package:

```text
my_algo
```

Executable:

```text
aeb_mux
```

Subscribe:

```text
/scan                    sensor_msgs/msg/LaserScan
/vesc/odom               nav_msgs/msg/Odometry
/planner/motor/speed     std_msgs/msg/Float64
/planner/servo/position  std_msgs/msg/Float64
```

Publish:

```text
/commands/motor/speed      std_msgs/msg/Float64
/commands/servo/position   std_msgs/msg/Float64
```

Purpose:

```text
Receives the planner command.
Checks front clearance and TTC from /scan and /vesc/odom.
If safe, forwards planner command to VESC.
If unsafe, publishes zero motor speed.
```

Config file:

```text
config/aeb.yaml
```

Important tuning parameters:

```yaml
front_fov_deg: 50.0
stop_clearance_m: 0.15
slow_clearance_m: 0.55
ttc_threshold_s: 0.35
max_erpm: 12000.0
```

## Full Topic Graph

With `real_launch.py`:

```text
Livox driver
  publishes /livox/lidar
  publishes /livox/imu

pointcloud_to_laserscan
  subscribes /livox/lidar
  publishes  /scan

disparity_extender
  subscribes /scan
  subscribes /joy_active
  subscribes /autonomous_mode
  publishes  /planner/motor/speed
  publishes  /planner/servo/position

aeb_mux
  subscribes /scan
  subscribes /vesc/odom
  subscribes /planner/motor/speed
  subscribes /planner/servo/position
  publishes  /commands/motor/speed
  publishes  /commands/servo/position

VESC driver
  subscribes /commands/motor/speed
  subscribes /commands/servo/position
  publishes  /vesc/odom
```

With `scan_launch.py`:

```text
Existing scan source
  publishes /scan

disparity_extender
  subscribes /scan
  subscribes /joy_active
  subscribes /autonomous_mode
  publishes  /planner/motor/speed
  publishes  /planner/servo/position

aeb_mux
  subscribes /scan
  subscribes /vesc/odom
  subscribes /planner/motor/speed
  subscribes /planner/servo/position
  publishes  /commands/motor/speed
  publishes  /commands/servo/position
```

## What To Check Before Driving

Check that Livox data exists:

```bash
ros2 topic hz /livox/lidar
```

Check that `/scan` exists:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

Check planner output:

```bash
ros2 topic echo /planner/motor/speed
ros2 topic echo /planner/servo/position
```

Check final VESC output:

```bash
ros2 topic echo /commands/motor/speed
ros2 topic echo /commands/servo/position
```

Check odometry:

```bash
ros2 topic hz /vesc/odom
```

## Debugging

Case 1:

```text
/scan is missing
```

Likely causes:

```text
Livox topic name is not /livox/lidar
pointcloud_to_laserscan is not installed
TF between base_link and livox_frame is wrong or missing
```

Check:

```bash
ros2 topic list
ros2 topic info /livox/lidar
ros2 run tf2_ros tf2_echo base_link livox_frame
```

Case 2:

```text
/planner/motor/speed is zero
```

Likely causes:

```text
/autonomous_mode is false
/joy_active is true
/scan is seeing the floor or a very close object
```

Enable autonomous mode:

```bash
ros2 topic pub /autonomous_mode std_msgs/msg/Bool "{data: true}" -r 5
```

Case 3:

```text
/planner/motor/speed is positive, but /commands/motor/speed is zero
```

Likely cause:

```text
AEB is stopping the car.
```

Check RViz `/scan`. If the floor appears as a close obstacle, tune:

```text
real_launch.py:
  min_height
  max_height
  lidar_pitch

config/aeb.yaml:
  front_fov_deg
  stop_clearance_m
  slow_clearance_m
```

Case 4:

```text
/commands/motor/speed is positive, but the car does not move
```

Likely causes:

```text
VESC driver is not subscribing to /commands/motor/speed
ERPM is too low
deadman switch or hardware safety is blocking motor output
```

Check:

```bash
ros2 topic info /commands/motor/speed
ros2 topic echo /commands/motor/speed
```

## RViz Notes

The original Livox topic is 3D:

```text
/livox/lidar  PointCloud2
```

The driving algorithm uses 2D:

```text
/scan  LaserScan
```

If the floor is red in the PointCloud2 view, that can be normal. The important
question is whether `/scan` contains very close floor points in front of the
car. If `/scan` shows a close arc in front of the car while the path is empty,
fix TF and height filtering before increasing speed.
