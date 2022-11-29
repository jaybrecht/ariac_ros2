import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
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

    trial_config_name = LaunchConfiguration("trial_config").perform(context)
    trial_config_path = os.path.join(pkg_share, 'config', 'trial_configuration', trial_config_name)

    user_config_name = LaunchConfiguration("user_config").perform(context)
    user_config_path = os.path.join(pkg_share, 'config', 'user_configuration', user_config_name)

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
        arguments=[user_config_path]
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
        ],
        arguments=[trial_config_path, user_config_path]
    )

    state_publishers = []
    for name in robot_descriptions.keys():
        state_pub = Node(
            namespace=name,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            # name=name + "_robot_state_publisher",
            output="screen",
            parameters=[
                {'robot_description': robot_descriptions[name]},
                {"use_sim_time": True},
            ],
            remappings=[
                ('joint_states', '/joint_states'),
            ]
        )

        state_publishers.append(state_pub)
    
    robot_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ariac_description"), "/launch", "/robot_controllers.launch.py"]
        )
    )

    nodes_to_start = [
        gazebo,
        sensor_tf_broadcaster,
        object_tf_broadcaster,
        environment_startup,
        *state_publishers,
        robot_controllers,
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
        DeclareLaunchArgument("trial_config", default_value="sample.yaml", description="trial_configuration")
    )

    declared_arguments.append(
        DeclareLaunchArgument("user_config", default_value="sample.yaml", description="user_configuration")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])