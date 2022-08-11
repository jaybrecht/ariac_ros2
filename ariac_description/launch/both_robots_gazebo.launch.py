import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    assembly_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf", "assembly_robot.urdf.xacro"]), 
            " "
        ]
    )
    
    assembly_robot_description = {"robot_description": assembly_robot_description_content}

    assembly_robot_state_publisher_node = Node(
        namespace="assembly",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, assembly_robot_description],
    )

    kitting_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf", "kitting_robot.urdf.xacro"]), 
            " "
        ]
    )
    
    kitting_robot_description = {"robot_description": kitting_robot_description_content}

    kitting_robot_state_publisher_node = Node(
        namespace="kitting",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, kitting_robot_description],
    )

    # Assembly robot controllers
    assembly_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/assembly/controller_manager"],
    )

    assembly_arm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_trajectory_controller", "-c", "/assembly/controller_manager"],
    )

    assembly_gantry_joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_joint_trajectory_controller", "-c", "/assembly/controller_manager"],
    )

    # Kitting robot controllers
    kitting_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/kitting/controller_manager"],
    )

    kitting_arm_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_joint_trajectory_controller", "-c", "/kitting/controller_manager"],
    )

    kitting_linear_joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["linear_actuator_joint_trajectory_controller", "-c", "/kitting/controller_manager"],
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn assembly robot
    gazebo_spawn_assembly_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_assembly_robot",
        arguments=["-entity", "assembly_robot", "-topic", "assembly/robot_description"],
        output="screen",
    )

    # Spawn kitting robot
    gazebo_spawn_kitting_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_kitting_robot",
        arguments=["-entity", "kitting_robot", "-topic", "kitting/robot_description"],
        output="screen",
    )

    nodes_to_start = [
        assembly_robot_state_publisher_node,
        kitting_robot_state_publisher_node,
        assembly_joint_state_broadcaster_spawner,
        assembly_arm_joint_controller_spawner,
        assembly_gantry_joint_controller_spawner,
        kitting_joint_state_broadcaster_spawner,
        kitting_arm_joint_controller_spawner,
        kitting_linear_joint_controller_spawner,
        gazebo,
        gazebo_spawn_assembly_robot,
        gazebo_spawn_kitting_robot
    ]

    return LaunchDescription(nodes_to_start)