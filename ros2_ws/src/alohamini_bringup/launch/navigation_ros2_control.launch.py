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

# Nav2 navigation using the native C++ ros2_control base driver
# (alohamini_base_control) instead of the ZMQ bridge. Mirrors navigation.launch.py
# but swaps the sensors layer for sensors_ros2_control.launch.py.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    enable_head_camera = LaunchConfiguration("enable_head_camera")
    head_camera_device = LaunchConfiguration("head_camera_device")
    head_camera_image_topic = LaunchConfiguration("head_camera_image_topic")
    head_camera_frame_id = LaunchConfiguration("head_camera_frame_id")
    head_camera_fps = LaunchConfiguration("head_camera_fps")
    head_camera_width = LaunchConfiguration("head_camera_width")
    head_camera_height = LaunchConfiguration("head_camera_height")
    head_camera_jpeg_quality = LaunchConfiguration("head_camera_jpeg_quality")
    wait_for_time_sync = LaunchConfiguration("wait_for_time_sync")

    sensors_launch = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "launch", "sensors_ros2_control.launch.py"]
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare("alohamini_bringup"), "config", "nav2_params.yaml"]
    )
    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    time_gate = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [FindPackagePrefix("alohamini_bringup"), "lib", "alohamini_bringup", "wait_for_clock_sync"]
            ),
            "--enabled",
            wait_for_time_sync,
        ],
        output="screen",
    )
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "use_mock_hardware": use_mock_hardware,
            "enable_head_camera": enable_head_camera,
            "head_camera_device": head_camera_device,
            "head_camera_image_topic": head_camera_image_topic,
            "head_camera_frame_id": head_camera_frame_id,
            "head_camera_fps": head_camera_fps,
            "head_camera_width": head_camera_width,
            "head_camera_height": head_camera_height,
            "head_camera_jpeg_quality": head_camera_jpeg_quality,
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={
            "slam": "False",
            "map": map_file,
            "use_sim_time": "false",
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    def start_after_clock_gate(event, _context):
        if event.returncode == 0:
            return [sensors, nav2]
        reason = f"Clock synchronization gate failed with exit code {event.returncode}"
        return [EmitEvent(event=Shutdown(reason=reason))]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Serial bus with the base wheels (motor IDs 8/9/10).",
            ),
            DeclareLaunchArgument("baud_rate", default_value="1000000"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("map", description="Path to a saved map yaml file."),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "enable_head_camera",
                default_value="false",
                description="Publish the head camera stream (disabled by default to reduce DDS/Wi-Fi load).",
            ),
            DeclareLaunchArgument("head_camera_device", default_value="/dev/am_camera_forward"),
            DeclareLaunchArgument("head_camera_image_topic", default_value="/head_camera/image_raw"),
            DeclareLaunchArgument("head_camera_frame_id", default_value="head_camera"),
            DeclareLaunchArgument("head_camera_fps", default_value="10.0"),
            DeclareLaunchArgument("head_camera_width", default_value="640"),
            DeclareLaunchArgument("head_camera_height", default_value="480"),
            DeclareLaunchArgument("head_camera_jpeg_quality", default_value="70"),
            DeclareLaunchArgument(
                "wait_for_time_sync",
                default_value="true",
                description="Wait for the Pi kernel clock to report NTP synchronization before ROS nodes start.",
            ),
            time_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=time_gate, on_exit=start_after_clock_gate)
            ),
        ]
    )
