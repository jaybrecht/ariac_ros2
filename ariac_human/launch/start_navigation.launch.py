#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_package_dir = get_package_share_directory('ariac_human')
    navigation_launch_dir = os.path.join(navigation_package_dir, 'launch')
    
    # set to true if you want to use gazebo clock
    map_file = os.path.join(
        navigation_package_dir,
        'map',
        'ariac_map.yaml')
    
    params_file = os.path.join(
        navigation_package_dir,
        'config',
        'human_params.yaml')


    rviz_file = os.path.join(
        navigation_package_dir,
        'rviz',
        'navigation.rviz')
    
    LaunchConfiguration('map', default=map_file)
    LaunchConfiguration('params_file', default=params_file)

    return LaunchDescription([
        
        # start rviz with navigation configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_navigation',
            arguments=['-d', rviz_file],
            # parameters=[{'use_sim_time': 'true'}],
            output='screen'),
        
        Node(
            package='ariac_gazebo',
            executable='navigation_frame_broadcaster.py',
            output='screen',
            arguments=[],
        ),
        
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [navigation_launch_dir, '/spawn_human.launch.py'])
        ),
        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [navigation_launch_dir, '/navigation_bringup.launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                # 'namespace': 'human',
                # 'use_namespace': 'true',
                'params_file': params_file}.items(),
        ),

        
    ])
