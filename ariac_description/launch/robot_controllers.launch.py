from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_names = [
        'floor_robot', 
        'ceiling_robot', 
        'agv1',
        'agv2',
        'agv3',
        'agv4'
        ]


    nodes_to_start = []

    for name in robot_names:
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

        if name.count('agv') == 0:
            controller_type = 'joint_trajectory_controller'
        else:
            controller_type = 'velocity_controller'
        
        controller_spawner = Node(
            package="ariac_gazebo",
            executable="spawner",
            namespace=name,
            name=name + "_controller_spawner",
            arguments=[controller_type, "-n", name],
            parameters=[
                {"use_sim_time": True},
            ],
        )
      
        nodes_to_start.append(joint_state_broadcaster_spawner)
        nodes_to_start.append(controller_spawner)

    return LaunchDescription(nodes_to_start)