from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes_to_start = []

    # Controllers for the UR robots
    ariac_robots_joint_state_broadcaster_spawner = Node(
        package="ariac_gazebo",
        executable="spawner",
        name="ariac_robots_joint_state_broadcaster_spawner",
        arguments=["ariac_robots_joint_state_broadcaster"],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    nodes_to_start.append(ariac_robots_joint_state_broadcaster_spawner)

    floor_robot_trajectory_controller_spawner = Node(
        package="ariac_gazebo",
        executable="spawner",
        name="floor_robot_trajectory_controller_spawner",
        arguments=['floor_robot_joint_trajectory_controller'],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    nodes_to_start.append(floor_robot_trajectory_controller_spawner)

    ceiling_robot_trajectory_controller_spawner = Node(
        package="ariac_gazebo",
        executable="spawner",
        name="ceiling_robot_trajectory_controller_spawner",
        arguments=['ceiling_robot_joint_trajectory_controller'],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    nodes_to_start.append(ceiling_robot_trajectory_controller_spawner)
    
    # Controllers for the AGVS
    agv_names = [
        'agv1',
        'agv2',
        'agv3',
        'agv4'
    ]

    for name in agv_names:
        joint_state_broadcaster_spawner = Node(
            package="ariac_gazebo",
            executable="spawner",
            namespace=name,
            name=name + "_joint_state_broadcaster_spawner",
            arguments=["joint_state_broadcaster", "-n", name],
            parameters=[
                {"use_sim_time": True},
            ],
        )

        nodes_to_start.append(joint_state_broadcaster_spawner)
        
        controller_spawner = Node(
            package="ariac_gazebo",
            executable="spawner",
            namespace=name,
            name=name + "_controller_spawner",
            arguments=['velocity_controller', "-n", name],
            parameters=[
                {"use_sim_time": True},
            ],
        )
        
        nodes_to_start.append(controller_spawner)


    return LaunchDescription(nodes_to_start)