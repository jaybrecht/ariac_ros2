#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

import rclpy

from ariac_gazebo.gazebo_robot_spawner import GazeboRobotSpawner
from ariac_gazebo.spawn_params import GazeboSpawnParams

def main():
    rclpy.init()

    robot_spawner = GazeboRobotSpawner()

    robot_params = []
    robot_names = []
    
    ## Create spawn params for the mobile robot
    # model_path = os.path.join(get_package_share_directory('ariac_mobile_robot'), 'models', "mobile_robot", 'model.sdf')
    # robot_params.append(GazeboSpawnParams('mobile_robot', file_path=model_path, xyz=[-4.0, 3.5, 0], rpy = [0, 0, 3.14]))

    # Create spawn params for the URDF robots
    # robot_names.append('floor_robot')
    robot_names.append('ceiling_robot')
    robot_names.append('agv1')
    robot_names.append('agv2')
    robot_names.append('agv3')
    robot_names.append('agv4')
    
    for name in robot_names:
        robot_params.append(GazeboSpawnParams(name))

    # Spawn the robots into gazebo
    robot_spawner.get_logger().info("Spawner started")

    for params in robot_params:
        if not robot_spawner.spawn_from_params(params):
            robot_spawner.get_logger().error(f"Unable to spawn {params.name}")

    robot_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()