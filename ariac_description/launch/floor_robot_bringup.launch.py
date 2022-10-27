import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    # Arguments 
    start_moveit = LaunchConfiguration("start_moveit")
    start_rviz = LaunchConfiguration("start_rviz")

    # Generate Robot Description parameter from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/floor_robot", "floor_robot.urdf.xacro"]), 
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    ## Moveit Parameters
    robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/floor_robot.srdf")}

    robot_description_kinematics = {"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    # Nodes
    robot_state_publisher_node = Node(
        namespace="floor_robot",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Gazebo Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/floor_robot/controller_manager"],
    )

    joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/floor_robot/controller_manager"],
    )
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,     
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("start_moveit", default_value="false", description="Start moveit nodes for the robots?")
    )

    declared_arguments.append(
        DeclareLaunchArgument("start_rviz", default_value="false", description="Start rviz?")
    )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])