import os
import yaml

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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


def generate_launch_description():
    floor_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/floor_robot", "floor_robot.urdf.xacro"]), 
            " "
        ]
    )

    floor_robot_description = {"floor_robot_description": floor_robot_description_content}

    ## Moveit Parameters
    floor_robot_description_semantic = {"floor_robot_description_semantic": load_file("ariac_moveit_config", "srdf/floor_robot.srdf")}

    floor_robot_kinematics = {"floor_robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    gripper_change = Node(
        package="test_competitor",
        namespace="floor_robot",
        executable="gripper_change",
        output="screen",
        parameters=[
            floor_robot_description,
            floor_robot_description_semantic,
            floor_robot_kinematics,
            {"use_sim_time": True},
        ],
        # remappings=[("floor_robot/joint_states", "joint_states")]
        
    )

    nodes_to_start = [gripper_change]

    return LaunchDescription(nodes_to_start)
