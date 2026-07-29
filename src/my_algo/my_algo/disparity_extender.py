"""Disparity extender planner for a real F1TENTH car."""

import math
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64

from my_algo.vesc_utils import clamp, speed_to_erpm


class DisparityExtenderNode(Node):
    """Pick a safe LiDAR direction using disparity extension."""

    def __init__(self):
        super().__init__("disparity_extender")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("joy_active_topic", "/joy_active")
        self.declare_parameter("autonomous_mode_topic", "/autonomous_mode")
        self.declare_parameter("motor_speed_topic", "/planner/motor/speed")
        self.declare_parameter("servo_position_topic", "/planner/servo/position")
        self.declare_parameter("require_autonomous_mode", True)

        self.declare_parameter("lidar_yaw_offset_deg", 90.0)
        self.declare_parameter("front_fov_deg", 180.0)
        self.declare_parameter("max_range_m", 12.0)
        self.declare_parameter("min_range_m", 0.08)
        self.declare_parameter("smoothing_window", 3)
        self.declare_parameter("disparity_threshold_m", 0.45)
        self.declare_parameter("car_width_m", 0.31)
        self.declare_parameter("safety_margin_m", 0.16)
        self.declare_parameter("target_range_ratio", 0.82)
        self.declare_parameter("center_angle_weight", 0.18)

        self.declare_parameter("steering_gain", 1.0)
        self.declare_parameter("max_steer_rad", 0.34)
        self.declare_parameter("steering_deadband_rad", 0.04)
        self.declare_parameter("steering_filter_alpha", 0.22)

        self.declare_parameter("min_speed_mps", 0.45)
        self.declare_parameter("base_speed_mps", 1.1)
        self.declare_parameter("max_speed_mps", 1.8)
        self.declare_parameter("straight_steer_rad", 0.06)
        self.declare_parameter("corner_steer_rad", 0.22)
        self.declare_parameter("slow_clearance_m", 1.4)
        self.declare_parameter("stop_clearance_m", 0.28)
        self.declare_parameter("speed_ramp_rate_mps2", 1.2)

        self.declare_parameter("erpm_gain", 4614.0)
        self.declare_parameter("min_drive_erpm", 1850.0)
        self.declare_parameter("erpm_ramp_rate_per_s", 4500.0)
        self.declare_parameter("servo_center", 0.5)
        self.declare_parameter("servo_gain", 0.28)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.joy_active_topic = self.get_parameter("joy_active_topic").value
        self.autonomous_mode_topic = self.get_parameter("autonomous_mode_topic").value
        self.motor_speed_topic = self.get_parameter("motor_speed_topic").value
        self.servo_position_topic = self.get_parameter("servo_position_topic").value
        self.require_autonomous_mode = self.as_bool(
            self.get_parameter("require_autonomous_mode").value
        )

        self.lidar_yaw_offset = math.radians(
            float(self.get_parameter("lidar_yaw_offset_deg").value)
        )
        self.front_fov = math.radians(float(self.get_parameter("front_fov_deg").value))
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.min_range_m = float(self.get_parameter("min_range_m").value)
        self.smoothing_window = int(self.get_parameter("smoothing_window").value)
        self.disparity_threshold_m = float(
            self.get_parameter("disparity_threshold_m").value
        )
        self.car_width_m = float(self.get_parameter("car_width_m").value)
        self.safety_margin_m = float(self.get_parameter("safety_margin_m").value)
        self.target_range_ratio = float(self.get_parameter("target_range_ratio").value)
        self.center_angle_weight = float(
            self.get_parameter("center_angle_weight").value
        )

        self.steering_gain = float(self.get_parameter("steering_gain").value)
        self.max_steer_rad = float(self.get_parameter("max_steer_rad").value)
        self.steering_deadband_rad = float(
            self.get_parameter("steering_deadband_rad").value
        )
        self.steering_filter_alpha = float(
            self.get_parameter("steering_filter_alpha").value
        )

        self.min_speed_mps = float(self.get_parameter("min_speed_mps").value)
        self.base_speed_mps = float(self.get_parameter("base_speed_mps").value)
        self.max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self.straight_steer_rad = float(
            self.get_parameter("straight_steer_rad").value
        )
        self.corner_steer_rad = float(self.get_parameter("corner_steer_rad").value)
        self.slow_clearance_m = float(self.get_parameter("slow_clearance_m").value)
        self.stop_clearance_m = float(self.get_parameter("stop_clearance_m").value)
        self.speed_ramp_rate_mps2 = float(
            self.get_parameter("speed_ramp_rate_mps2").value
        )

        self.erpm_gain = float(self.get_parameter("erpm_gain").value)
        self.min_drive_erpm = float(self.get_parameter("min_drive_erpm").value)
        self.erpm_ramp_rate_per_s = float(
            self.get_parameter("erpm_ramp_rate_per_s").value
        )
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_gain = float(self.get_parameter("servo_gain").value)

        self.extension_radius_m = self.car_width_m / 2.0 + self.safety_margin_m
        self.joy_active = False
        self.auto_mode = False
        self.prev_steering = 0.0
        self.current_speed_cmd = 0.0
        self.current_erpm_cmd = 0.0
        self.prev_time = self.get_clock().now()

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )
        self.joy_active_sub = self.create_subscription(
            Bool,
            self.joy_active_topic,
            self.joy_active_callback,
            10,
        )
        self.auto_mode_sub = self.create_subscription(
            Bool,
            self.autonomous_mode_topic,
            self.auto_mode_callback,
            10,
        )
        self.speed_pub = self.create_publisher(Float64, self.motor_speed_topic, 10)
        self.servo_pub = self.create_publisher(
            Float64,
            self.servo_position_topic,
            10,
        )

        self.get_logger().info(
            f"Disparity extender ready: {self.scan_topic} -> "
            f"{self.motor_speed_topic}, {self.servo_position_topic}"
        )

    def joy_active_callback(self, msg):
        """Track whether joystick control is active."""
        self.joy_active = msg.data

    def auto_mode_callback(self, msg):
        """Track autonomous mode state."""
        previous = self.auto_mode
        self.auto_mode = msg.data
        if previous != self.auto_mode:
            mode = "ON" if self.auto_mode else "OFF"
            self.get_logger().info(f"Autonomous mode {mode}")

    def scan_callback(self, scan):
        """Compute a planner command from one LaserScan."""
        if self.joy_active:
            self.publish_stop()
            return
        if self.require_autonomous_mode and not self.auto_mode:
            self.publish_stop()
            return
        if scan.angle_increment == 0.0 or not scan.ranges:
            self.publish_stop()
            return

        samples = self.front_samples(scan)
        if len(samples) < 3:
            self.publish_stop()
            return

        angles = [angle for angle, _ in samples]
        ranges = [distance for _, distance in samples]
        self.smooth_ranges(ranges, self.smoothing_window)

        angle_step = self.median_angle_step(angles)
        extended_ranges = self.extend_disparities(ranges, angle_step)
        target_idx = self.select_target_index(angles, extended_ranges)
        if target_idx is None:
            self.publish_stop()
            return

        steering = self.steering_for_angle(angles[target_idx])
        target_speed = self.speed_for_target(
            abs(steering),
            extended_ranges[target_idx],
        )

        now = self.get_clock().now()
        dt = max((now - self.prev_time).nanoseconds / 1e9, 1e-3)
        speed = self.ramp_speed(target_speed, dt)
        self.prev_time = now
        self.publish_command(steering, speed, dt)

    def front_samples(self, scan) -> List[Tuple[float, float]]:
        """Collect scan readings in the vehicle-front sector."""
        half_fov = self.front_fov / 2.0
        samples = []

        for index, raw_range in enumerate(scan.ranges):
            lidar_angle = scan.angle_min + index * scan.angle_increment
            vehicle_angle = self.lidar_to_vehicle_angle(lidar_angle)
            if abs(vehicle_angle) > half_fov:
                continue
            samples.append((vehicle_angle, self.sanitize_range(scan, raw_range)))

        samples.sort(key=lambda item: item[0])
        return samples

    def lidar_to_vehicle_angle(self, lidar_angle):
        """Convert raw LiDAR scan angle to vehicle angle where front is zero."""
        angle = lidar_angle - self.lidar_yaw_offset
        return math.atan2(math.sin(angle), math.cos(angle))

    def sanitize_range(self, scan, value):
        """Clamp invalid or out-of-range LiDAR readings."""
        upper = self.max_range_m
        if scan.range_max > 0.0:
            upper = min(upper, scan.range_max)
        lower = max(self.min_range_m, scan.range_min)

        if math.isnan(value) or math.isinf(value):
            return upper
        return clamp(value, lower, upper)

    @staticmethod
    def smooth_ranges(ranges: List[float], window: int):
        """Apply a small moving average in-place."""
        if window <= 1 or len(ranges) < window:
            return
        radius = window // 2
        original = list(ranges)
        for index in range(len(ranges)):
            start = max(0, index - radius)
            end = min(len(ranges), index + radius + 1)
            ranges[index] = sum(original[start:end]) / float(end - start)

    @staticmethod
    def median_angle_step(angles: Sequence[float]):
        """Estimate angular resolution for a sorted angle list."""
        diffs = [
            abs(angles[index + 1] - angles[index])
            for index in range(len(angles) - 1)
            if angles[index + 1] > angles[index]
        ]
        if not diffs:
            return math.radians(0.25)
        diffs.sort()
        return max(diffs[len(diffs) // 2], math.radians(0.05))

    def extend_disparities(
        self,
        ranges: Sequence[float],
        angle_step: float,
    ) -> List[float]:
        """Inflate obstacle edges across the open side of each disparity."""
        extended = list(ranges)
        original = list(ranges)

        for index in range(len(original) - 1):
            left = original[index]
            right = original[index + 1]
            if abs(left - right) < self.disparity_threshold_m:
                continue

            if left < right:
                count = self.extension_count(left, angle_step)
                self.fill_range(extended, index + 1, index + 1 + count, left)
            else:
                count = self.extension_count(right, angle_step)
                self.fill_range(extended, index + 1 - count, index + 1, right)

        return extended

    def extension_count(self, distance, angle_step):
        """Return how many scan bins the car width occupies at this distance."""
        angle = math.atan2(
            self.extension_radius_m,
            max(distance, self.min_range_m),
        )
        return max(1, int(math.ceil(angle / angle_step)))

    @staticmethod
    def as_bool(value):
        """Parse launch/YAML boolean values robustly on Foxy."""
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def fill_range(ranges: List[float], start: int, end: int, distance: float):
        """Overwrite an index range with a nearer obstacle distance."""
        start = max(0, start)
        end = min(len(ranges), end)
        for index in range(start, end):
            ranges[index] = min(ranges[index], distance)

    def select_target_index(
        self,
        angles: Sequence[float],
        ranges: Sequence[float],
    ) -> Optional[int]:
        """Choose a far safe bin while mildly preferring straight ahead."""
        best_range = max(ranges)
        if best_range <= self.min_range_m:
            return None

        ratio = clamp(self.target_range_ratio, 0.0, 1.0)
        threshold = best_range * ratio
        candidates = [
            index for index, value in enumerate(ranges)
            if value >= threshold
        ]
        if not candidates:
            return None

        return max(
            candidates,
            key=lambda index: (
                ranges[index] - self.center_angle_weight * abs(angles[index])
            ),
        )

    def steering_for_angle(self, target_angle):
        """Convert a target angle into filtered steering radians."""
        steering = clamp(
            target_angle * self.steering_gain,
            -self.max_steer_rad,
            self.max_steer_rad,
        )
        if abs(steering) < self.steering_deadband_rad:
            steering = 0.0

        filtered = self.prev_steering + self.steering_filter_alpha * (
            steering - self.prev_steering
        )
        self.prev_steering = clamp(filtered, -self.max_steer_rad, self.max_steer_rad)
        return self.prev_steering

    def speed_for_target(self, abs_steer, clearance):
        """Pick a speed from steering angle and available clearance."""
        if clearance <= self.stop_clearance_m:
            return 0.0

        if abs_steer <= self.straight_steer_rad:
            steer_speed = self.max_speed_mps
        elif abs_steer >= self.corner_steer_rad:
            steer_speed = self.min_speed_mps
        else:
            ratio = (abs_steer - self.straight_steer_rad) / (
                self.corner_steer_rad - self.straight_steer_rad
            )
            steer_speed = self.max_speed_mps - ratio * (
                self.max_speed_mps - self.base_speed_mps
            )

        clearance_ratio = (clearance - self.stop_clearance_m) / max(
            self.slow_clearance_m - self.stop_clearance_m,
            0.01,
        )
        speed = steer_speed * clamp(clearance_ratio, 0.0, 1.0)
        if speed <= 0.0:
            return 0.0
        return clamp(speed, self.min_speed_mps, self.max_speed_mps)

    def ramp_speed(self, target_speed, dt):
        """Limit acceleration into the target speed."""
        if target_speed <= self.current_speed_cmd:
            self.current_speed_cmd = target_speed
            return self.current_speed_cmd

        max_step = self.speed_ramp_rate_mps2 * dt
        self.current_speed_cmd = min(self.current_speed_cmd + max_step, target_speed)
        return self.current_speed_cmd

    def publish_command(self, steering_rad, speed_mps, dt):
        """Publish planner ERPM and servo commands."""
        speed_msg = Float64()
        speed_msg.data = self.ramp_erpm(self.target_erpm(speed_mps), dt)
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = clamp(
            self.servo_center - steering_rad * self.servo_gain,
            0.0,
            1.0,
        )
        self.servo_pub.publish(servo_msg)

    def target_erpm(self, speed_mps):
        """Convert speed to ERPM while respecting the minimum rolling command."""
        if speed_mps <= 0.0:
            return 0.0
        return max(speed_to_erpm(speed_mps, self.erpm_gain), self.min_drive_erpm)

    def ramp_erpm(self, target_erpm, dt):
        """Limit ERPM changes so starts do not jump to min_drive_erpm at once."""
        if target_erpm <= 0.0:
            self.current_erpm_cmd = 0.0
            return self.current_erpm_cmd

        max_step = max(self.erpm_ramp_rate_per_s, 0.0) * max(dt, 1e-3)
        if self.current_erpm_cmd <= 0.0:
            self.current_erpm_cmd = min(max_step, target_erpm)
        else:
            delta = clamp(target_erpm - self.current_erpm_cmd, -max_step, max_step)
            self.current_erpm_cmd += delta
        return self.current_erpm_cmd

    def publish_stop(self):
        """Publish zero planner speed and centered steering."""
        speed_msg = Float64()
        speed_msg.data = 0.0
        self.speed_pub.publish(speed_msg)

        servo_msg = Float64()
        servo_msg.data = self.servo_center
        self.servo_pub.publish(servo_msg)
        self.current_speed_cmd = 0.0
        self.current_erpm_cmd = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
