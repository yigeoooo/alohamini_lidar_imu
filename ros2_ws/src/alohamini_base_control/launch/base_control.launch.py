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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    pkg_share = FindPackageShare("alohamini_base_control")

    # Expand the ros2_control xacro (which itself includes the description URDF).
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "alohamini_base.ros2_control.xacro"]),
            " serial_port:=",
            serial_port,
            " baud_rate:=",
            baud_rate,
            " use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Start the base controller only after the joint_state_broadcaster is up.
    delay_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_base_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ACM0",
                description="Serial port of the Feetech bus driving the base wheels. "
                "NOTE: shared with the left arm/lift; do not run the lerobot host concurrently.",
            ),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use ros2_control mock_components instead of the real serial driver.",
            ),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_base_controller,
        ]
    )
