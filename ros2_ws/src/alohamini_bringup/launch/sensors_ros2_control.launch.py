# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

# Sensors + base layer using the native C++ ros2_control driver
# (alohamini_base_control) instead of the ZMQ bridge (alohamini_nav_bridge).
#
# Provides the same downstream interface as sensors_bridge.launch.py:
#   - robot_description + full /joint_states -> TF tree (odom->base_link->sensors)
#   - /scan_filtered (sector-limited laser)
#   - /cmd_vel subscriber + /odom (+ odom->base_link TF) via OmniBaseController
#
# Joint-state handling: base_control's joint_state_broadcaster publishes only the
# three wheel joints (remapped here to /wheel_joint_states). A joint_state_publisher
# then merges those (via source_list) with zero defaults for the arm/lift joints and
# republishes the complete /joint_states, so robot_state_publisher can emit TF for
# every link (laser_frame etc.). Without this the arm/sensor TFs would be missing.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    scan_input_topic = LaunchConfiguration("scan_input_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_min_angle = LaunchConfiguration("scan_min_angle")
    scan_max_angle = LaunchConfiguration("scan_max_angle")

    base_control_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_base_control"), "launch", "base_control.launch.py"]
    )

    # URDF for the arm/lift joints that joint_state_publisher fills with zeros.
    desc_urdf = os.path.join(
        get_package_share_directory("alohamini_description"), "urdf", "alohamini_nav.urdf"
    )
    with open(desc_urdf, "r") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial bus with the base wheels (motor IDs 8/9/10). "
                "Do not run the lerobot host concurrently.",
            ),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use ros2_control mock_components instead of the real serial driver.",
            ),
            DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("scan_min_angle", default_value="-1.57079632679"),
            DeclareLaunchArgument("scan_max_angle", default_value="1.57079632679"),
            # Base: robot_state_publisher + controller_manager + wheel controllers.
            # Remap the broadcaster's /joint_states to /wheel_joint_states so it does
            # not clash with the merged /joint_states below.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_control_launch),
                launch_arguments={
                    "serial_port": serial_port,
                    "baud_rate": baud_rate,
                    "use_mock_hardware": use_mock_hardware,
                    "joint_states_topic": "/wheel_joint_states",
                }.items(),
            ),
            # Merge wheel states (real, from broadcaster) with zero defaults for the
            # arm/lift joints, publishing the complete /joint_states.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "source_list": ["/wheel_joint_states"],
                        "use_sim_time": False,
                        "rate": 30,
                    }
                ],
            ),
            # Sector-limit the laser scan (unchanged from the ZMQ-bridge stack).
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
        ]
    )
