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
#   - robot_description + full /joint_states -> TF tree (odom->base_footprint->base_link->sensors)
#   - /scan_filtered (sector-limited laser)
#   - /cmd_vel subscriber + /odom (+ odom->base_footprint TF) via OmniBaseController
#   - head camera publisher for the PC-side RViz view
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    base_yaw_deg = LaunchConfiguration("base_yaw_deg")
    scan_input_topic = LaunchConfiguration("scan_input_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    scan_marker_topic = LaunchConfiguration("scan_marker_topic")
    scan_marker_range = LaunchConfiguration("scan_marker_range")
    scan_min_angle = LaunchConfiguration("scan_min_angle")
    scan_max_angle = LaunchConfiguration("scan_max_angle")
    enable_head_camera = LaunchConfiguration("enable_head_camera")
    head_camera_device = LaunchConfiguration("head_camera_device")
    head_camera_image_topic = LaunchConfiguration("head_camera_image_topic")
    head_camera_frame_id = LaunchConfiguration("head_camera_frame_id")
    head_camera_fps = LaunchConfiguration("head_camera_fps")
    head_camera_width = LaunchConfiguration("head_camera_width")
    head_camera_height = LaunchConfiguration("head_camera_height")
    head_camera_jpeg_quality = LaunchConfiguration("head_camera_jpeg_quality")

    base_control_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_base_control"), "launch", "base_control.launch.py"]
    )
    head_camera_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "launch", "head_camera.launch.py"]
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
            DeclareLaunchArgument(
                "base_yaw_deg",
                default_value="90.0",
                description="Static yaw (deg) of base_footprint->base_link; tune if RViz forward/left look wrong.",
            ),
            DeclareLaunchArgument("scan_input_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("scan_marker_topic", default_value="/scan_sector_marker"),
            DeclareLaunchArgument("scan_marker_range", default_value="1.0"),
            DeclareLaunchArgument("scan_min_angle", default_value="-3.14159265359"),
            DeclareLaunchArgument("scan_max_angle", default_value="0.0"),
            DeclareLaunchArgument("enable_head_camera", default_value="true"),
            DeclareLaunchArgument("head_camera_device", default_value="/dev/am_camera_forward"),
            DeclareLaunchArgument("head_camera_image_topic", default_value="/head_camera/image_raw"),
            DeclareLaunchArgument("head_camera_frame_id", default_value="head_camera"),
            DeclareLaunchArgument("head_camera_fps", default_value="20.0"),
            DeclareLaunchArgument("head_camera_width", default_value="640"),
            DeclareLaunchArgument("head_camera_height", default_value="480"),
            DeclareLaunchArgument("head_camera_jpeg_quality", default_value="80"),
            # Base: robot_state_publisher + controller_manager + wheel controllers.
            # Remap the broadcaster's /joint_states to /wheel_joint_states so it does
            # not clash with the merged /joint_states below.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_control_launch),
                launch_arguments={
                    "serial_port": serial_port,
                    "baud_rate": baud_rate,
                    "use_mock_hardware": use_mock_hardware,
                    "base_yaw_deg": base_yaw_deg,
                    "joint_states_topic": "/wheel_joint_states",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(head_camera_launch),
                condition=IfCondition(enable_head_camera),
                launch_arguments={
                    "device": head_camera_device,
                    "image_topic": head_camera_image_topic,
                    "frame_id": head_camera_frame_id,
                    "fps": head_camera_fps,
                    "width": head_camera_width,
                    "height": head_camera_height,
                    "jpeg_quality": head_camera_jpeg_quality,
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
                        "marker_topic": scan_marker_topic,
                        "marker_range": scan_marker_range,
                    }
                ],
            ),
        ]
    )
