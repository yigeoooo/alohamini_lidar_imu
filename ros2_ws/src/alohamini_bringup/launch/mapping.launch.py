from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    cmd_port = LaunchConfiguration("cmd_port")
    obs_port = LaunchConfiguration("obs_port")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
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
    use_commanded_yaw_for_odom = LaunchConfiguration("use_commanded_yaw_for_odom")
    commanded_yaw_odom_scale = LaunchConfiguration("commanded_yaw_odom_scale")
    enable_head_camera = LaunchConfiguration("enable_head_camera")
    head_camera_device = LaunchConfiguration("head_camera_device")
    head_camera_fps = LaunchConfiguration("head_camera_fps")
    head_camera_width = LaunchConfiguration("head_camera_width")
    head_camera_height = LaunchConfiguration("head_camera_height")
    head_camera_jpeg_quality = LaunchConfiguration("head_camera_jpeg_quality")

    sensors_bridge_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "launch", "sensors_bridge.launch.py"]
    )
    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "slam_toolbox.yaml"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("cmd_port", default_value="5555"),
            DeclareLaunchArgument("obs_port", default_value="5556"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument("linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("swap_xy", default_value="false"),
            DeclareLaunchArgument("odom_linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("odom_linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("odom_angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("allow_reverse", default_value="true"),
            DeclareLaunchArgument("allow_lateral_motion", default_value="true"),
            DeclareLaunchArgument("require_observation_for_motion", default_value="false"),
            DeclareLaunchArgument("use_commanded_yaw_for_odom", default_value="true"),
            DeclareLaunchArgument("commanded_yaw_odom_scale", default_value="0.95"),
            DeclareLaunchArgument("enable_head_camera", default_value="false"),
            DeclareLaunchArgument("head_camera_device", default_value="/dev/am_camera_forward"),
            DeclareLaunchArgument("head_camera_fps", default_value="10.0"),
            DeclareLaunchArgument("head_camera_width", default_value="640"),
            DeclareLaunchArgument("head_camera_height", default_value="480"),
            DeclareLaunchArgument("head_camera_jpeg_quality", default_value="70"),
            DeclareLaunchArgument("slam_params_file", default_value=default_slam_params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_bridge_launch),
                launch_arguments={
                    "host": host,
                    "cmd_port": cmd_port,
                    "obs_port": obs_port,
                    "use_joint_state_publisher": use_joint_state_publisher,
                    "bridge_cmd_vel_topic": "/cmd_vel",
                    "enable_scan_filter": "true",
                    "enable_collision_monitor": "false",
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
                    "use_commanded_yaw_for_odom": use_commanded_yaw_for_odom,
                    "commanded_yaw_odom_scale": commanded_yaw_odom_scale,
                    "enable_head_camera": enable_head_camera,
                    "head_camera_device": head_camera_device,
                    "head_camera_fps": head_camera_fps,
                    "head_camera_width": head_camera_width,
                    "head_camera_height": head_camera_height,
                    "head_camera_jpeg_quality": head_camera_jpeg_quality,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": "false",
                    "slam_params_file": slam_params_file,
                }.items(),
            ),
        ]
    )
