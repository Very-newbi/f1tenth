from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    require_autonomous_mode = LaunchConfiguration("require_autonomous_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/vesc/odom"),
            DeclareLaunchArgument("require_autonomous_mode", default_value="true"),
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
