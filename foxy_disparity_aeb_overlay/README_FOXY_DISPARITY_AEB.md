# ROS2 Foxy Disparity Extender + AEB

Target:

```text
Ubuntu 20.04
ROS2 Foxy
Jetson
Livox /scan or PointCloud2-to-LaserScan pipeline
VESC direct command topics
```

This is a single ROS2 Python package named `my_algo`.

## Topic Graph

```text
/scan
  -> my_algo/disparity_extender
  -> /planner/motor/speed
  -> /planner/servo/position
  -> my_algo/aeb_mux
  -> /commands/motor/speed
  -> /commands/servo/position
```

AEB also reads odometry:

```text
/vesc/odom
```

## Install

Copy this folder into the Jetson workspace:

```text
~/ros2_ws/src/my_algo
```

Then build:

```bash
cd ~/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

If a `/scan` topic already exists:

```bash
ros2 launch my_algo scan_launch.py
```

If Livox publishes `/livox/lidar` as `sensor_msgs/msg/PointCloud2`:

```bash
ros2 launch my_algo real_launch.py points_topic:=/livox/lidar
```

Autonomous commands are gated by `/autonomous_mode` by default:

```bash
ros2 topic pub /autonomous_mode std_msgs/msg/Bool "{data: true}" --once
```

If you do not use joystick/autonomous-mode topics, set this in
`config/disparity_extender.yaml`:

```yaml
require_autonomous_mode: false
```

## VESC Topics

Output to VESC:

```text
/commands/motor/speed      std_msgs/msg/Float64  # ERPM
/commands/servo/position   std_msgs/msg/Float64  # 0.0 to 1.0
```

Tune these first:

```yaml
erpm_gain
servo_center
servo_gain
max_speed_mps
```

Start with the car lifted or motor output disabled.
