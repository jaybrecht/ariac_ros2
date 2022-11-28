from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        joint_state_broadcaster_spawner,
        joint_controller_spawner,     
    ]

    return LaunchDescription(nodes_to_start)