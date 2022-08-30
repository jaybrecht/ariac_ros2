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
    # Generate Robot Description parameter from xacro
    assembly_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf", "assembly_robot.urdf.xacro"]), 
            " "
        ]
    )
    assembly_robot_description = {"robot_description": assembly_robot_description_content}

    kitting_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf", "kitting_robot.urdf.xacro"]), 
            " "
        ]
    )
    kitting_robot_description = {"robot_description": kitting_robot_description_content}

    ## Moveit Parameters
    assembly_robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/assembly_robot.srdf")}
    
    kitting_robot_description_semantic = {"robot_description_semantic": load_file("ariac_moveit_config", "srdf/kitting_robot.srdf")}

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
    
    assembly_moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml("ariac_moveit_config", "config/assembly_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    kitting_moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml("ariac_moveit_config", "config/kitting_controllers.yaml"),
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
    assembly_robot_state_publisher_node = Node(
        namespace="assembly",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[assembly_robot_description,
            assembly_robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    kitting_robot_state_publisher_node = Node(
        namespace="kitting",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[kitting_robot_description,
            kitting_robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # Move group node
    assembly_move_group_node = Node(
        namespace="assembly",
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            assembly_robot_description,
            assembly_robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            assembly_moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    kitting_move_group_node = Node(
        namespace="kitting",
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            kitting_robot_description,
            kitting_robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            kitting_moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    # Gazebo Controllers
    assembly_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/assembly/controller_manager"],
    )

    assembly_joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/assembly/controller_manager"],
    )

    kitting_joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/kitting/controller_manager"],
    )

    kitting_joint_controller_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/kitting/controller_manager"],
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
    )

    # Spawn robot
    gazebo_spawn_assembly_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_gantry",
        arguments=["-entity", "assembly_robot", "-topic", "/assembly/robot_description"],
        output="screen",
    )

    gazebo_spawn_kitting_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_gantry",
        arguments=["-entity", "kitting_robot", "-topic", "/kitting/robot_description"],
        output="screen",
    )

    # rviz with moveit configuration
    assembly_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ariac_moveit_config"), "config", "view_assembly_robot.rviz"]
    )

    assembly_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="assembly_rviz2",
        output="log",
        arguments=["-d", assembly_rviz_config_file],
        parameters=[
            assembly_robot_description,
            assembly_robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    kitting_rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ariac_moveit_config"), "config", "view_kitting_robot.rviz"]
    )

    kitting_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="kitting_rviz2",
        output="log",
        arguments=["-d", kitting_rviz_config_file],
        parameters=[
            kitting_robot_description,
            kitting_robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )
    
    nodes_to_start = [
        assembly_robot_state_publisher_node,
        kitting_robot_state_publisher_node,
        assembly_joint_state_broadcaster_spawner,
        kitting_joint_state_broadcaster_spawner,
        assembly_joint_controller_spawner,
        kitting_joint_controller_spawner,
        gazebo,
        gazebo_spawn_assembly_robot,
        gazebo_spawn_kitting_robot,
        assembly_move_group_node,
        kitting_move_group_node,
        assembly_rviz_node,
        kitting_rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
