from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    cmd_port = LaunchConfiguration("cmd_port")
    obs_port = LaunchConfiguration("obs_port")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    linear_x_scale = LaunchConfiguration("linear_x_scale")
    linear_y_scale = LaunchConfiguration("linear_y_scale")
    angular_z_scale = LaunchConfiguration("angular_z_scale")
    swap_xy = LaunchConfiguration("swap_xy")
    require_observation_for_motion = LaunchConfiguration("require_observation_for_motion")
    scan_input_topic = LaunchConfiguration("scan_input_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_min_angle = LaunchConfiguration("scan_min_angle")
    scan_max_angle = LaunchConfiguration("scan_max_angle")

    description_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_description"), "launch", "description.launch.py"]
    )
    bridge_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_nav_bridge"), "launch", "bridge.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="127.0.0.1",
                description="AlohaMini host ZMQ address. Use the Pi IP if loopback does not connect.",
            ),
            DeclareLaunchArgument("cmd_port", default_value="5555"),
            DeclareLaunchArgument("obs_port", default_value="5556"),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description="Start joint_state_publisher for the imported URDF joints.",
            ),
            DeclareLaunchArgument("linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("swap_xy", default_value="false"),
            DeclareLaunchArgument(
                "require_observation_for_motion",
                default_value="true",
                description="Send zero velocity when AlohaMini host observations are stale.",
            ),
            DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("scan_min_angle", default_value="-1.57079632679"),
            DeclareLaunchArgument("scan_max_angle", default_value="1.57079632679"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "use_joint_state_publisher": use_joint_state_publisher,
                }.items(),
            ),
            Node(
                package="alohamini_bringup",
                executable="scan_sector_filter",
                name="scan_sector_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": scan_input_topic,
                        "output_topic": scan_topic,
                        "min_angle": scan_min_angle,
                        "max_angle": scan_max_angle,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "host": host,
                    "cmd_port": cmd_port,
                    "obs_port": obs_port,
                    "linear_x_scale": linear_x_scale,
                    "linear_y_scale": linear_y_scale,
                    "angular_z_scale": angular_z_scale,
                    "swap_xy": swap_xy,
                    "require_observation_for_motion": require_observation_for_motion,
                }.items(),
            ),
        ]
    )
