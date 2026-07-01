from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    cmd_port = LaunchConfiguration("cmd_port")
    obs_port = LaunchConfiguration("obs_port")
    linear_x_scale = LaunchConfiguration("linear_x_scale")
    linear_y_scale = LaunchConfiguration("linear_y_scale")
    angular_z_scale = LaunchConfiguration("angular_z_scale")
    swap_xy = LaunchConfiguration("swap_xy")
    require_observation_for_motion = LaunchConfiguration("require_observation_for_motion")

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("cmd_port", default_value="5555"),
            DeclareLaunchArgument("obs_port", default_value="5556"),
            DeclareLaunchArgument("linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("swap_xy", default_value="false"),
            DeclareLaunchArgument("require_observation_for_motion", default_value="true"),
            Node(
                package="alohamini_nav_bridge",
                executable="bridge_node",
                name="alohamini_nav_bridge",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "cmd_port": cmd_port,
                        "obs_port": obs_port,
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "publish_tf": True,
                        "rate_hz": 30.0,
                        "cmd_timeout_sec": 0.5,
                        "obs_timeout_sec": 0.5,
                        "max_linear_speed": 0.20,
                        "max_lateral_speed": 0.20,
                        "max_angular_speed": 0.8,
                        "linear_x_scale": linear_x_scale,
                        "linear_y_scale": linear_y_scale,
                        "angular_z_scale": angular_z_scale,
                        "swap_xy": swap_xy,
                        "require_observation_for_motion": require_observation_for_motion,
                        "pose_covariance_xy": 0.10,
                        "pose_covariance_yaw": 0.25,
                        "twist_covariance_xy": 0.20,
                        "twist_covariance_yaw": 0.35,
                    }
                ],
            ),
        ]
    )

