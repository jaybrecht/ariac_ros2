import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):    
    # Set the path to this package.
    pkg_share = FindPackageShare(package='ariac_gazebo').find('ariac_gazebo')
    
    # Set the path to the world file
    world_file_name = 'ariac.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    
    # General arguments
    start_rviz = LaunchConfiguration("start_rviz")
    start_moveit = LaunchConfiguration("start_moveit")
    
    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            'world': world_path,
            }.items()
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
    agv1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv1", 'y_position': "4.707484"}.items()
    )

    agv2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv2", 'y_position': "1.302086"}.items()
    )

    agv3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv3", 'y_position': "-1.295472"}.items()
    )

    agv4_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={'agv_number': "agv4", 'y_position': "-4.696062"}.items()
    )

    # Mobile Robot Bringup
    mobile_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_mobile_robot"), "/launch", "/mobile_robot_bringup.launch.py"]
        ),
    )

    sensor_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_sensors.py',
        output='screen',
        arguments=[]
    )

    sensor_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='sensor_tf_broadcaster.py',
        output='screen',
        arguments=[]
    )

    bins_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='bins_tf_broadcaster.py',
        output='screen',
        arguments=[]
    )

    # Robot Spawner Node
    robot_spawner = Node(
        package='ariac_gazebo',
        executable='spawn_robots.py',
        output='screen',
        arguments=[]
    )

    spawn_robots_after_sensors = RegisterEventHandler(
        OnProcessExit(
            target_action=sensor_spawner,
            on_exit=[
                robot_spawner,
                agv1_bringup,
                agv2_bringup,
                agv3_bringup,
                agv4_bringup,
                # floor_robot_bringup,
                ceiling_robot_bringup,
                ]
        )
    )

    nodes_to_start = [
        gazebo,
        # sensor_spawner,
        # sensor_tf_broadcaster,
        # bins_tf_broadcaster,
        # spawn_robots_after_sensors
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