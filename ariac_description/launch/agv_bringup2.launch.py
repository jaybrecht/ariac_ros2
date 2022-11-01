from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Generate urdf from xacro
    urdf_file = get_package_share_directory("ariac_description") + "/urdf/AGV/agv.urdf.xacro"

    locations = {"agv1":"4.707484", "agv2":"1.302086", "agv3":"-1.295472", "agv4":"-4.696062"}

    urdfs = {}

    for name in locations.keys():
        urdfs[name] = xacro.process_file(urdf_file, mappings={'agv_number': name, 'y_position':locations[name]}).toxml()

    nodes = []

    for name in urdfs.keys():
        nodes.append(
            Node(
                name="robot_state_publisher",
                namespace=name,
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {'robot_description': urdfs[name]},
                    {"use_sim_time": True},
                ],
                arguments=[urdf_file]
            )
        )
        
        nodes.append(
            Node(
                name=name+"_joint_state_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/" + name + "/controller_manager"],
            )
        )

        nodes.append(
            Node(
                name=name+"_velocity_controller_spawner",
                package="controller_manager",
                executable="spawner",
                arguments=["velocity_controller", "-c", "/" + name + "/controller_manager"],
            )
        )
        

    return LaunchDescription(nodes)
