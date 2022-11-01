from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Launch arguments
    agv_number = LaunchConfiguration("agv_number")
    y_position = LaunchConfiguration("y_position")

    num = agv_number.perform(context)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"]), 
            " ",
            "agv_number:=",
            agv_number,
            " ",
            "y_position:=",
            y_position,
            " ",
        ]
    )

    robot_state_publisher = Node(
        name="robot_state_publisher",
        namespace=agv_number,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': robot_description_content},
            {"use_sim_time": True},
        ],
        arguments=[PathJoinSubstitution([FindPackageShare("ariac_description"), "urdf/AGV", "agv.urdf.xacro"])]
    )
    
    joint_state_spawner = Node(
        name=num + "_joint_state_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/" + num + "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        name=num + "_velocity_controller_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/" + num + "/controller_manager"],
    )

    gazebo_spawn_robot = Node(
        name="spawn_" + num,
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", num, "-topic", "/" + num + "/robot_description"],
        output="screen",
    )

    return [
        robot_state_publisher,
        joint_state_spawner,
        velocity_controller_spawner,
        gazebo_spawn_robot,
    ]

def generate_launch_description(): 
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("agv_number", default_value="agv1", description="AGV number"))
    declared_arguments.append(DeclareLaunchArgument("y_position", default_value="4.707484", description="y position for agv"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
