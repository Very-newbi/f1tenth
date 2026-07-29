"""AEB mux that filters planner commands before sending them to VESC."""

import math
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from my_algo.vesc_utils import clamp


class AebMuxNode(Node):
    """Forward planner commands unless LiDAR time-to-collision is unsafe."""

    def __init__(self):
        super().__init__("aeb_mux")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/vesc/odom")
        self.declare_parameter("input_motor_speed_topic", "/planner/motor/speed")
        self.declare_parameter("input_servo_position_topic", "/planner/servo/position")
        self.declare_parameter("output_motor_speed_topic", "/commands/motor/speed")
        self.declare_parameter(
            "output_servo_position_topic",
            "/commands/servo/position",
        )

        self.declare_parameter("lidar_yaw_offset_deg", 90.0)
        self.declare_parameter("front_fov_deg", 70.0)
        self.declare_parameter("lidar_to_bumper_m", 0.10)
        self.declare_parameter("stop_clearance_m", 0.20)
        self.declare_parameter("slow_clearance_m", 0.75)
        self.declare_parameter("ttc_threshold_s", 0.45)
        self.declare_parameter("command_timeout_s", 0.35)
        self.declare_parameter("max_erpm", 8500.0)
        self.declare_parameter("servo_center", 0.5)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.input_motor_speed_topic = self.get_parameter(
            "input_motor_speed_topic"
        ).value
        self.input_servo_position_topic = self.get_parameter(
            "input_servo_position_topic"
        ).value
        self.output_motor_speed_topic = self.get_parameter(
            "output_motor_speed_topic"
        ).value
        self.output_servo_position_topic = self.get_parameter(
            "output_servo_position_topic"
        ).value

        self.lidar_yaw_offset = math.radians(
            float(self.get_parameter("lidar_yaw_offset_deg").value)
        )
        self.front_fov = math.radians(float(self.get_parameter("front_fov_deg").value))
        self.lidar_to_bumper_m = float(self.get_parameter("lidar_to_bumper_m").value)
        self.stop_clearance_m = float(self.get_parameter("stop_clearance_m").value)
        self.slow_clearance_m = float(self.get_parameter("slow_clearance_m").value)
        self.ttc_threshold_s = float(self.get_parameter("ttc_threshold_s").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.max_erpm = float(self.get_parameter("max_erpm").value)
        self.servo_center = float(self.get_parameter("servo_center").value)

        self.current_speed_mps = 0.0
        self.front_min_clearance_m = math.inf
        self.front_min_ttc_s = math.inf
        self.last_motor_erpm = 0.0
        self.last_servo_position = self.servo_center
        self.last_command_time = self.get_clock().now()

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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.motor_sub = self.create_subscription(
            Float64,
            self.input_motor_speed_topic,
            self.motor_callback,
            10,
        )
        self.servo_sub = self.create_subscription(
            Float64,
            self.input_servo_position_topic,
            self.servo_callback,
            10,
        )
        self.motor_pub = self.create_publisher(
            Float64,
            self.output_motor_speed_topic,
            10,
        )
        self.servo_pub = self.create_publisher(
            Float64,
            self.output_servo_position_topic,
            10,
        )
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)

        self.get_logger().info(
            f"AEB mux ready: {self.input_motor_speed_topic}, "
            f"{self.input_servo_position_topic} -> "
            f"{self.output_motor_speed_topic}, {self.output_servo_position_topic}"
        )

    def odom_callback(self, msg):
        """Track current longitudinal speed from VESC odometry."""
        self.current_speed_mps = msg.twist.twist.linear.x

    def scan_callback(self, scan):
        """Update AEB clearance and TTC from LiDAR."""
        self.front_min_clearance_m = self.front_min_clearance(scan)
        self.front_min_ttc_s = self.front_min_ttc(scan)
        self.publish_filtered()

    def motor_callback(self, msg):
        """Store latest planner motor ERPM command."""
        self.last_motor_erpm = clamp(msg.data, -self.max_erpm, self.max_erpm)
        self.last_command_time = self.get_clock().now()
        self.publish_filtered()

    def servo_callback(self, msg):
        """Store latest planner servo command."""
        self.last_servo_position = clamp(msg.data, 0.0, 1.0)
        self.last_command_time = self.get_clock().now()
        self.publish_filtered()

    def watchdog_callback(self):
        """Stop if planner commands stop arriving."""
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        if elapsed > self.command_timeout_s:
            self.publish_stop()

    def publish_filtered(self):
        """Publish the safe version of the latest planner command."""
        motor_erpm = self.last_motor_erpm
        if self.must_stop():
            motor_erpm = 0.0
        elif self.front_min_clearance_m < self.slow_clearance_m:
            motor_erpm *= self.slowdown_scale(self.front_min_clearance_m)

        motor_msg = Float64()
        motor_msg.data = clamp(motor_erpm, -self.max_erpm, self.max_erpm)
        self.motor_pub.publish(motor_msg)

        servo_msg = Float64()
        servo_msg.data = self.last_servo_position
        self.servo_pub.publish(servo_msg)

    def must_stop(self):
        """Return true when clearance or TTC requires braking."""
        return (
            self.front_min_clearance_m <= self.stop_clearance_m
            or self.front_min_ttc_s <= self.ttc_threshold_s
        )

    def slowdown_scale(self, clearance):
        """Scale speed down as the obstacle approaches stop clearance."""
        span = max(self.slow_clearance_m - self.stop_clearance_m, 0.01)
        return clamp((clearance - self.stop_clearance_m) / span, 0.0, 1.0)

    def front_min_clearance(self, scan):
        """Return nearest bumper-frame clearance in the front sector."""
        values = self.front_values(scan)
        if not values:
            return math.inf
        return max(0.0, min(values) - self.lidar_to_bumper_m)

    def front_min_ttc(self, scan):
        """Return minimum front time-to-collision."""
        if self.current_speed_mps <= 0.05:
            return math.inf

        min_ttc = math.inf
        half_fov = self.front_fov / 2.0
        for vehicle_angle, distance in self.indexed_front_values(scan):
            if abs(vehicle_angle) > half_fov:
                continue
            closing_speed = self.current_speed_mps * math.cos(vehicle_angle)
            if closing_speed <= 0.05:
                continue
            clearance = max(0.0, distance - self.lidar_to_bumper_m)
            min_ttc = min(min_ttc, clearance / closing_speed)
        return min_ttc

    def front_values(self, scan):
        """Return finite range values in the front sector."""
        return [distance for _, distance in self.indexed_front_values(scan)]

    def indexed_front_values(self, scan):
        """Yield vehicle-angle and distance pairs in the front sector."""
        half_fov = self.front_fov / 2.0
        for index, raw_range in enumerate(scan.ranges):
            lidar_angle = scan.angle_min + index * scan.angle_increment
            vehicle_angle = self.lidar_to_vehicle_angle(lidar_angle)
            if abs(vehicle_angle) > half_fov:
                continue
            distance = self.sanitize_range(scan, raw_range)
            if distance is not None:
                yield vehicle_angle, distance

    def lidar_to_vehicle_angle(self, lidar_angle):
        """Convert raw LiDAR angle to vehicle angle where front is zero."""
        angle = lidar_angle - self.lidar_yaw_offset
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def sanitize_range(scan, value) -> Optional[float]:
        """Return finite valid range or None."""
        if math.isnan(value) or math.isinf(value):
            return None
        if value <= 0.0:
            return 0.0
        return min(value, scan.range_max)

    def publish_stop(self):
        """Publish a zero-speed stop command."""
        motor_msg = Float64()
        motor_msg.data = 0.0
        self.motor_pub.publish(motor_msg)

        servo_msg = Float64()
        servo_msg.data = self.servo_center
        self.servo_pub.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AebMuxNode()
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
