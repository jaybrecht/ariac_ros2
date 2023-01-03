import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    ceiling_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ceiling_robot", "ceiling_robot.urdf.xacro"]), 
            " "
        ]
    )

    floor_robot_description = {"floor_robot_description": floor_robot_description_content}

    ceiling_robot_description = {"ceiling_robot_description": ceiling_robot_description_content}

    
    ## Moveit Parameters
    floor_robot_description_semantic = {"floor_robot_description_semantic": load_file("ariac_moveit_config", "srdf/floor_robot.srdf")}

    floor_robot_kinematics = {"floor_robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    ceiling_robot_description_semantic = {"ceiling_robot_description_semantic": load_file("ariac_moveit_config", "srdf/ceiling_robot.srdf")}

    ceiling_robot_kinematics = {"ceiling_robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    moveit_test = Node(
        package="test_competitor",
        executable="moveit_test",
        output="screen",
        parameters=[
            floor_robot_description,
            floor_robot_description_semantic,
            floor_robot_kinematics,
            ceiling_robot_description,
            ceiling_robot_description_semantic,
            ceiling_robot_kinematics,
            {"use_sim_time": True},
        ],
        
    )

    # RVIZ 
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("test_competitor"), "rviz", "moveit_test.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            floor_robot_description,
            floor_robot_description_semantic,
            floor_robot_kinematics,
            ceiling_robot_description,
            ceiling_robot_description_semantic,
            ceiling_robot_kinematics,
            {"use_sim_time": True},
        ],
        arguments=['-d', rviz_config_file]
    )

    nodes_to_start = [moveit_test, rviz]

    return LaunchDescription(nodes_to_start)
