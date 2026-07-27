from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory("my_algo")
    planner_config = os.path.join(
        package_share,
        "config",
        "disparity_extender.yaml",
    )
    aeb_config = os.path.join(package_share, "config", "aeb.yaml")

    points_topic = LaunchConfiguration("points_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    target_frame = LaunchConfiguration("target_frame")
    require_autonomous_mode = LaunchConfiguration("require_autonomous_mode")
    publish_lidar_tf = LaunchConfiguration("publish_lidar_tf")
    base_frame = LaunchConfiguration("base_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")
    lidar_pitch = LaunchConfiguration("lidar_pitch")
    lidar_roll = LaunchConfiguration("lidar_roll")

    return LaunchDescription(
        [
            DeclareLaunchArgument("points_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/vesc/odom"),
            DeclareLaunchArgument("target_frame", default_value="livox_frame"),
            DeclareLaunchArgument("require_autonomous_mode", default_value="true"),
            DeclareLaunchArgument("publish_lidar_tf", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
            DeclareLaunchArgument("lidar_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.20"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="livox_static_tf",
                output="screen",
                condition=IfCondition(publish_lidar_tf),
                arguments=[
                    lidar_x,
                    lidar_y,
                    lidar_z,
                    lidar_yaw,
                    lidar_pitch,
                    lidar_roll,
                    base_frame,
                    lidar_frame,
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "transform_tolerance": 0.02,
                        "min_height": -0.10,
                        "max_height": 0.50,
                        "angle_min": -3.14159,
                        "angle_max": 3.14159,
                        "angle_increment": 0.00436,
                        "scan_time": 0.10,
                        "range_min": 0.05,
                        "range_max": 20.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }
                ],
                remappings=[
                    ("cloud_in", points_topic),
                    ("scan", scan_topic),
                ],
            ),
            Node(
                package="my_algo",
                executable="disparity_extender",
                name="disparity_extender",
                output="screen",
                parameters=[
                    planner_config,
                    {
                        "scan_topic": scan_topic,
                        "require_autonomous_mode": require_autonomous_mode,
                    },
                ],
            ),
            Node(
                package="my_algo",
                executable="aeb_mux",
                name="aeb_mux",
                output="screen",
                parameters=[
                    aeb_config,
                    {
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                    },
                ],
            ),
        ]
    )
