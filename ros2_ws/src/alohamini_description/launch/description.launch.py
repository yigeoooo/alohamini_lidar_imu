from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = Path(get_package_share_directory("alohamini_description"))
    urdf_path = package_dir / "urdf" / "alohamini_nav.urdf"
    robot_description = urdf_path.read_text()

    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description="Start joint_state_publisher with zero/default joints for visualization.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(use_joint_state_publisher),
            ),
        ]
    )

