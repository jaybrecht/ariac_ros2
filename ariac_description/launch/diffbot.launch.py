# Copyright 2020 Open Source Robotics Foundation, Inc.
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
# See the License for the specific language governing permissions and
# limitations under the License.

from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("name")

    # Generate Robot Description parameter from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/diffbot", "diffbot.urdf.xacro"])
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        namespace="diffbot",
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {"use_sim_time": True}]
    )

    # Gazebo Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/diffbot/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    position_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "-c", "/diffbot/controller_manager"],
        parameters=[{"use_sim_time": True}]
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_diffbot",
        arguments=["-entity", "diffbot", "-topic", "/diffbot/robot_description", "-x", "-4.0", "-y", "3.5", "-z", "0.1", "-Y", "3.14"],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        gazebo_spawn_robot,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("name", default_value="diffbot", description="robot_name")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])