from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    cmd_port = LaunchConfiguration("cmd_port")
    obs_port = LaunchConfiguration("obs_port")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    linear_x_scale = LaunchConfiguration("linear_x_scale")
    linear_y_scale = LaunchConfiguration("linear_y_scale")
    angular_z_scale = LaunchConfiguration("angular_z_scale")
    swap_xy = LaunchConfiguration("swap_xy")
    require_observation_for_motion = LaunchConfiguration("require_observation_for_motion")

    sensors_bridge_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "launch", "sensors_bridge.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "nav2_params.yaml"]
    )
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("cmd_port", default_value="5555"),
            DeclareLaunchArgument("obs_port", default_value="5556"),
            DeclareLaunchArgument("map", description="Path to a saved map yaml file."),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument("linear_x_scale", default_value="1.0"),
            DeclareLaunchArgument("linear_y_scale", default_value="1.0"),
            DeclareLaunchArgument("angular_z_scale", default_value="1.0"),
            DeclareLaunchArgument("swap_xy", default_value="false"),
            DeclareLaunchArgument("require_observation_for_motion", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_bridge_launch),
                launch_arguments={
                    "host": host,
                    "cmd_port": cmd_port,
                    "obs_port": obs_port,
                    "use_joint_state_publisher": use_joint_state_publisher,
                    "linear_x_scale": linear_x_scale,
                    "linear_y_scale": linear_y_scale,
                    "angular_z_scale": angular_z_scale,
                    "swap_xy": swap_xy,
                    "require_observation_for_motion": require_observation_for_motion,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "slam": "False",
                    "map": map_file,
                    "use_sim_time": "false",
                    "params_file": params_file,
                    "autostart": "true",
                }.items(),
            ),
        ]
    )
