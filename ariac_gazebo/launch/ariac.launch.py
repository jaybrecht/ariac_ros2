import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='ariac_gazebo').find('ariac_gazebo')
    
    # Set the path to the world file
    world_file_name = 'ariac.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    gazebo_plugin_path = os.path.join(pkg_share)
    os.environ["GAZEBO_PLUGIN_PATH"] = gazebo_plugin_path

    # General arguments
    start_rviz = LaunchConfiguration("start_rviz")
    start_moveit = LaunchConfiguration("start_moveit")
    
    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Floor Robot Bringup
    floor_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/floor_robot_bringup.launch.py"]
        ),
        launch_arguments={'start_moveit': start_moveit, 'start_rviz': start_rviz}.items()
    )

    # Ceiling Robot Bringup
    ceiling_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/ceiling_robot_bringup.launch.py"]
        ),
        launch_arguments={'start_moveit': start_moveit, 'start_rviz': start_rviz}.items()
    )

    # AGV Bringup
    agv_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
    )

    # Mobile Robot Bringup
    mobile_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_mobile_robot"), "/launch", "/mobile_robot_bringup.launch.py"]
        ),
    )
    
    # ROSbridge
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [FindPackageShare("rosbridge_server"), "/launch", "/rosbridge_websocket_launch.xml"]
        ),
    )

    # Robot Spawner Node
    robot_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_robots.py',
        output='screen',
    )

    nodes_to_start = [
        gazebo,
        agv_bringup,
        floor_robot_bringup,
        ceiling_robot_bringup,
        mobile_robot_bringup,
        robot_spawner,
        rosbridge
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument("start_rviz", default_value="false", description="Launch RViz?")
    )

    declared_arguments.append(
        DeclareLaunchArgument("start_moveit", default_value="false", description="Start moveit nodes for the robots?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
