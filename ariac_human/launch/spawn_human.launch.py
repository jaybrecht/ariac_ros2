#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    navigation_package_dir = get_package_share_directory('ariac_human')
    navigation_launch_dir = os.path.join(navigation_package_dir, 'launch')

    
    # Get the sdf file
    urdf_path = os.path.join(
        get_package_share_directory('ariac_human'),
        'models',
        'human',
        'model.sdf'
    )
    
    # print("urdf_path: {}".format(urdf_path))
    
    x_pose = '-15.0'
    y_pose = '-10.0'

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_launch_dir,
                         'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    


    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name = 'human_spawner',
        arguments=[
            '-entity', 'human',
            '-file', urdf_path,
            "-robot_namespace", "human",
            # '-topic', "/human/robot_description",
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05'
        ],
        output='screen',
    )

    ld = LaunchDescription()
    
    # Add any conditioned actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
