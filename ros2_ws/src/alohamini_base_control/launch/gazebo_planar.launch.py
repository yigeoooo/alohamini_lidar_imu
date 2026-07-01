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

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description() -> str:
    """Expand the planar-move gazebo xacro.

    The SolidWorks-exported URDF names each wheel joint identically to its child link
    (wheel1/2/3), which Gazebo Classic rejects as a name collision. There is no
    ros2_control block here, but the collision still breaks model loading, so we
    rename the physical <joint ... type="continuous"> declarations to wheelN_joint
    (links stay wheelN). XML comments are stripped for consistency with the
    ros2_control launch (harmless here, avoids any parser edge cases).
    """
    pkg_share = get_package_share_directory("alohamini_base_control")
    xacro_path = os.path.join(pkg_share, "urdf", "alohamini_base.gazebo_planar.xacro")
    urdf_xml = xacro.process_file(xacro_path).toxml()
    urdf_xml = re.sub(
        r'(<joint\s+name=")wheel([123])("\s+type="continuous")',
        r"\1wheel\2_joint\3",
        urdf_xml,
    )
    urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    return urdf_xml


def _launch_setup(context, *args, **kwargs):
    robot_description = {"robot_description": _build_robot_description(), "use_sim_time": True}

    # Resolve model:// mesh URIs locally and keep the built-in ground_plane/sun models
    # (see gazebo.launch.py for the rationale).
    desc_share = get_package_share_directory("alohamini_description")
    model_path_parts = [os.path.dirname(desc_share)]
    for builtin in ("/usr/share/gazebo-11/models", "/usr/share/gazebo/models"):
        if os.path.isdir(builtin):
            model_path_parts.append(builtin)
    prev_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
    if prev_model_path:
        model_path_parts.append(prev_model_path)
    set_model_path = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH", os.pathsep.join(model_path_parts)
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "world": LaunchConfiguration("world"),
            "verbose": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "alohamini", "-z", "0.1"],
        output="screen",
    )

    return [set_model_path, gazebo, robot_state_publisher, spawn_entity]


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            DeclareLaunchArgument(
                "gui", default_value="true",
                description="Launch the Gazebo client GUI (set false for headless).",
            ),
            DeclareLaunchArgument(
                "world", default_value="",
                description="Optional path to a Gazebo .world file.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
