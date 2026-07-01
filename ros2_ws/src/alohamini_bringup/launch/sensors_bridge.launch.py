from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    cmd_port = LaunchConfiguration("cmd_port")
    obs_port = LaunchConfiguration("obs_port")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    bridge_cmd_vel_topic = LaunchConfiguration("bridge_cmd_vel_topic")
    bridge_odom_topic = LaunchConfiguration("bridge_odom_topic")
    bridge_publish_tf = LaunchConfiguration("bridge_publish_tf")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    collision_monitor_params_file = LaunchConfiguration("collision_monitor_params_file")
    linear_x_scale = LaunchConfiguration("linear_x_scale")
    linear_y_scale = LaunchConfiguration("linear_y_scale")
    angular_z_scale = LaunchConfiguration("angular_z_scale")
    odom_linear_x_scale = LaunchConfiguration("odom_linear_x_scale")
    odom_linear_y_scale = LaunchConfiguration("odom_linear_y_scale")
    odom_angular_z_scale = LaunchConfiguration("odom_angular_z_scale")
    swap_xy = LaunchConfiguration("swap_xy")
    allow_reverse = LaunchConfiguration("allow_reverse")
    allow_lateral_motion = LaunchConfiguration("allow_lateral_motion")
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
    default_ekf_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "ekf.yaml"]
    )
    default_collision_monitor_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "collision_monitor.yaml"]
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
                "bridge_cmd_vel_topic",
                default_value="/cmd_vel_safe",
                description="Topic consumed by the ZMQ bridge after collision monitor filtering.",
            ),
            DeclareLaunchArgument(
                "bridge_odom_topic",
                default_value="/wheel/odom",
                description="Raw host odometry topic published by the bridge before EKF fusion.",
            ),
            DeclareLaunchArgument(
                "bridge_publish_tf",
                default_value="false",
                description="Disable bridge TF when EKF publishes odom->base_link.",
            ),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            DeclareLaunchArgument("collision_monitor_params_file", default_value=default_collision_monitor_params),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description="Start joint_state_publisher for the imported URDF joints.",
            ),
            DeclareLaunchArgument("linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("odom_linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("odom_linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("odom_angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("swap_xy", default_value="false"),
            DeclareLaunchArgument("allow_reverse", default_value="false"),
            DeclareLaunchArgument("allow_lateral_motion", default_value="false"),
            DeclareLaunchArgument(
                "require_observation_for_motion",
                default_value="true",
                description="Send zero velocity when AlohaMini host observations are stale.",
            ),
            DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument(
                "scan_min_angle",
                default_value="-1.57079632679",
                description="Minimum scan angle to keep in radians; default keeps the forward lidar sector.",
            ),
            DeclareLaunchArgument(
                "scan_max_angle",
                default_value="1.57079632679",
                description="Maximum scan angle to keep in radians; default keeps the forward lidar sector.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch),
                launch_arguments={
                    "use_joint_state_publisher": use_joint_state_publisher,
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params_file],
                remappings=[("odometry/filtered", "/odom")],
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
            LifecycleNode(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                namespace="",
                output="screen",
                parameters=[collision_monitor_params_file],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_collision_monitor",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": ["collision_monitor"],
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bridge_launch),
                launch_arguments={
                    "host": host,
                    "cmd_port": cmd_port,
                    "obs_port": obs_port,
                    "cmd_vel_topic": bridge_cmd_vel_topic,
                    "odom_topic": bridge_odom_topic,
                    "publish_tf": bridge_publish_tf,
                    "linear_x_scale": linear_x_scale,
                    "linear_y_scale": linear_y_scale,
                    "angular_z_scale": angular_z_scale,
                    "odom_linear_x_scale": odom_linear_x_scale,
                    "odom_linear_y_scale": odom_linear_y_scale,
                    "odom_angular_z_scale": odom_angular_z_scale,
                    "swap_xy": swap_xy,
                    "allow_reverse": allow_reverse,
                    "allow_lateral_motion": allow_lateral_motion,
                    "require_observation_for_motion": require_observation_for_motion,
                }.items(),
            ),
        ]
    )
