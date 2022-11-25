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

    robot_descriptions = generate_robot_descriptions()

    config_name = LaunchConfiguration("config")
    # trial_config_path = os.path.join(pkg_share, 'config', config_name)

    # Gazebo node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            'world': world_path,
            }.items()
    )

    # Sensor TF
    sensor_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='sensor_tf_broadcaster.py',
        output='screen',
    )

    # Objects TF
    object_tf_broadcaster = Node(
        package='ariac_gazebo',
        executable='object_tf_broadcaster.py',
        output='screen',
        arguments=[],
    )

    # Environment Startup
    environment_startup = Node(
        package='ariac_gazebo',
        executable='environment_startup_node.py',
        output='screen',
        parameters=[
            {'floor_robot_description': robot_descriptions['floor_robot']},
            {'ceiling_robot_description': robot_descriptions['ceiling_robot']},
            {'agv1_description': robot_descriptions['agv1']},
            {'agv2_description': robot_descriptions['agv2']},
            {'agv3_description': robot_descriptions['agv3']},
            {'agv4_description': robot_descriptions['agv4']},
            # {"use_sim_time": True},
        ],
    )

    floor_robot_state_publisher = Node(
        namespace="floor_robot",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['floor_robot']},
            {"use_sim_time": True},
        ],
    )

    ceiling_robot_state_publisher = Node(
        namespace="ceiling_robot",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['ceiling_robot']},
            {"use_sim_time": True},
        ],
    )

    agv1_robot_state_publisher = Node(
        namespace="agv1",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['agv1']},
            {"use_sim_time": True},
        ],
    )

    agv2_robot_state_publisher = Node(
        namespace="agv2",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['agv2']},
            {"use_sim_time": True},
        ],
    )

    agv3_robot_state_publisher = Node(
        namespace="agv3",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['agv3']},
            {"use_sim_time": True},
        ],
    )

    agv4_robot_state_publisher = Node(
        namespace="agv4",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'robot_description': robot_descriptions['agv4']},
            {"use_sim_time": True},
        ],
    )
    

    floor_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/floor_robot_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['floor_robot'],
            }.items()
    )

    ceiling_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/ceiling_robot_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['ceiling_robot'],
            }.items()
    )

    agv1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['agv1'],
            'agv_number': 'agv1',
            }.items()
    )

    agv2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['agv2'],
            'agv_number': 'agv2',
            }.items()
    )
    
    agv3_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['agv3'],
            'agv_number': 'agv3',
            }.items()
    )
    
    agv4_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/agv_bringup.launch.py"]
        ),
        launch_arguments={
            'robot_description': robot_descriptions['agv4'],
            'agv_number': 'agv4',
            }.items()
    )
    

    # Robot Bringups
    robot_bringups = RegisterEventHandler(
        OnProcessExit(
            target_action=environment_startup,
            on_exit=[
                ceiling_robot_bringup,
                floor_robot_bringup,
                # agv1_bringup,
                # agv2_bringup,
                # agv3_bringup,
                # agv4_bringup
            ]
        )
    )

    nodes_to_start = [
        gazebo,
        # sensor_tf_broadcaster,
        # object_tf_broadcaster,
        environment_startup,
        floor_robot_state_publisher,
        ceiling_robot_state_publisher,
        agv1_robot_state_publisher,
        agv2_robot_state_publisher,
        agv3_robot_state_publisher,
        agv4_robot_state_publisher,
        robot_bringups
    ]

    return nodes_to_start

def generate_robot_descriptions():
    descriptions = {}
    
    descriptions['floor_robot'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/floor_robot", "floor_robot.urdf.xacro"]), 
            " "
        ]
    )

    descriptions['ceiling_robot'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/ceiling_robot", "ceiling_robot.urdf.xacro"]), 
            " "
        ]
    )

    descriptions['agv1'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"]), 
            " ",
            "agv_number:=agv1",
            " ",
            "y_position:=4.8",
            " ",
        ]
    )

    descriptions['agv2'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"]), 
            " ",
            "agv_number:=agv2",
            " ",
            "y_position:=1.2",
            " ",
        ]
    )

    descriptions['agv3'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"]), 
            " ",
            "agv_number:=agv3",
            " ",
            "y_position:=-1.2",
            " ",
        ]
    )

    descriptions['agv4'] = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"]), 
            " ",
            "agv_number:=agv4",
            " ",
            "y_position:=-4.8",
            " ",
        ]
    )

    return descriptions

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("config", default_value="sample.yaml", description="trial_configuration")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])