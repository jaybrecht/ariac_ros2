from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):  
    # Define y_positions for agvs
    y_positions = ["4.707484", "1.302086", "-1.295472", "-4.696062"]
    
    nodes_to_start = []

    for i, pos in enumerate(y_positions):
        agv_number = "agv" + str(i+1)
    
        # Generate Robot Description parameter from xacro
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
                pos,
                " ",
            ]
        )
        robot_description = {"robot_description": robot_description_content}

        robot_state_publisher_node = Node(
            namespace=agv_number,
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description,
                {"use_sim_time": True},
            ],
        )

        # Gazebo Controllers
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/" + agv_number + "/controller_manager"],
        )

        position_controller_spawner= Node(
            package="controller_manager",
            executable="spawner",
            arguments=["agv_controller", "-c", "/" + agv_number + "/controller_manager"],
        )

        # Spawn robot
        gazebo_spawn_robot = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            # name="spawn_"+agv_number,
            arguments=["-entity", agv_number,"-robot_namespace", agv_number,  "-topic", "/" + agv_number + "/robot_description"],
            output="screen",
        )

        # AGV Move service node
        # agv_mover = Node(
        #     package="ariac_gazebo",
        #     executable="AGV_mover.py",
        #     name=agv_number + "_mover",
        #     arguments=[agv_number])

        nodes_to_start.append(robot_state_publisher_node)
        nodes_to_start.append(joint_state_broadcaster_spawner)
        nodes_to_start.append(position_controller_spawner)
        # nodes_to_start.append(gazebo_spawn_robot)
        # nodes_to_start.append(agv_mover)

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])