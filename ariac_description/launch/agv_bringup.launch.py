from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    # Launch arguments
    robot_description = {"robot_description": LaunchConfiguration("robot_description")}
    agv_number = LaunchConfiguration("agv_number")

    num = agv_number.perform(context)

    robot_state_publisher = Node(
        name="robot_state_publisher",
        namespace=agv_number,
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
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

    return [
        # robot_state_publisher,
        joint_state_spawner,
        velocity_controller_spawner,
    ]

def generate_launch_description(): 
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("agv_number", default_value="agv1", description="AGV number"))
    
    declared_arguments.append(
        DeclareLaunchArgument("robot_description", default_value="", description="Robot description content")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
