from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Arguments 
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}

    # Nodes
    robot_state_publisher_node = Node(
        namespace="ceiling_robot",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
            {"use_sim_time": True},
        ],
    )

    # Gazebo Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ceiling_robot_joint_state_broadcaster", "-c", "ceiling_robot_controller_manager"],
    )

    joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ceiling_robot_joint_trajectory_controller", "-c", "ceiling_robot_controller_manager"],
    )
    
    nodes_to_start = [
        # robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,     
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("robot_description", default_value="", description="Robot description content")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])