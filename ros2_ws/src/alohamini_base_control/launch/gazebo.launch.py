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
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description() -> str:
    """Expand the gazebo xacro and rename the wheel joints wheelN -> wheelN_joint.

    The SolidWorks-exported URDF names each wheel joint identically to its child
    link (wheel1/2/3). Gazebo Classic rejects that as a joint/link name collision,
    so we rename only the physical <joint ... type="continuous"> declarations. Link
    names stay wheelN; the ros2_control block and controllers_sim.yaml already use
    the wheelN_joint names, so everything lines up.
    """
    pkg_share = get_package_share_directory("alohamini_base_control")
    xacro_path = os.path.join(pkg_share, "urdf", "alohamini_base.gazebo.xacro")
    doc = xacro.process_file(xacro_path)
    urdf_xml = doc.toxml()
    # Rename only the continuous wheel joints, not the links or ros2_control joints.
    urdf_xml = re.sub(
        r'(<joint\s+name=")wheel([123])("\s+type="continuous")',
        r"\1wheel\2_joint\3",
        urdf_xml,
    )
    # Strip XML comments. gazebo_ros2_control (Humble) forwards robot_description as a
    # --param override to the controller_manager; comments containing '--' break its
    # argument parser ("Couldn't parse parameter override rule"). Upstream fix:
    # ros-controls/gz_ros2_control PR #505, not in the installed version.
    urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
    return urdf_xml


def _launch_setup(context, *args, **kwargs):
    robot_description = {"robot_description": _build_robot_description(), "use_sim_time": True}

    # Let Gazebo resolve model://alohamini_description/... URIs (the URDF's
    # package:// mesh paths become model:// on spawn) to the local install space.
    # GAZEBO_MODEL_PATH must contain the directory whose child is the package dir.
    # We also keep the built-in gazebo models dir so the empty world's ground_plane
    # and sun still resolve after GAZEBO_MODEL_DATABASE_URI is disabled (otherwise
    # there is no ground for the wheels to push against and the base cannot move).
    desc_share = get_package_share_directory("alohamini_description")
    models_root = os.path.dirname(desc_share)  # .../install/alohamini_description/share
    model_path_parts = [models_root]
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

    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_base_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_base_controller_spawner],
        )
    )

    return [
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_jsb_after_spawn,
        delay_base_after_jsb,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # Disable the online model database fetch. Without this, gzserver blocks
            # on http://models.gazebosim.org at startup, which can delay/prevent the
            # gazebo_ros2_control plugin from initialising the controller_manager.
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
