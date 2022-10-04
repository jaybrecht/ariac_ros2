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
            " "
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    ## Moveit Parameters
    robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/floor_robot.srdf")}

    robot_description_kinematics = {"robot_description_kinematics": load_yaml("ariac_moveit_config", "config/kinematics.yaml")}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "ariac_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml("ariac_moveit_config", "config/floor_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Nodes
    robot_state_publisher_node = Node(
        namespace="floor",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Move group node
    move_group_node = Node(
        namespace="floor",
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_moveit),
    )

    # Gazebo Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/floor/controller_manager"],
    )

    joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/floor/controller_manager"],
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_gantry",
        arguments=["-entity", "floor_robot", "-topic", "/floor/robot_description"],
        output="screen",
    )

    # rviz with moveit configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ariac_moveit_config"), "config", "view_floor_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
        condition=IfCondition(start_rviz),
    )
    
    
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        gazebo_spawn_robot,
        move_group_node,
        rviz_node,      
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