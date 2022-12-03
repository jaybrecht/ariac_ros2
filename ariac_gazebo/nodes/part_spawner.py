#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from ariac_gazebo.spawn_params import (
    SpawnParams, 
    PartSpawnParams)

class PartSpawner(Node):
    def __init__(self):
        super().__init__('part_spawner')
        
        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn_part(self, name, type, color, xyz, rpy):
        params = PartSpawnParams(name, type, color, xyz=xyz, rpy=rpy)

        self.spawn_entity(params)
            
    def spawn_entity(self, params: SpawnParams) -> bool:
        self.spawn_client.wait_for_service()

        self.get_logger().info(f'Spawning: {params.name}')

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().success

if __name__ == "__main__":
    rclpy.init()

    part_spawner = PartSpawner()

    part_spawner.spawn_part("blue_battery_99", "battery", "blue", [-7.8490, 2.9837, 1.0527], [0, 0, 1.57])
    part_spawner.spawn_part("red_pump_99", "pump", "red", [-7.5512, 3.0055, 1.0320], [0, 0, -1.57])
    part_spawner.spawn_part("purple_reg_99", "regulator", "purple", [-7.5256, 2.7749, 1.2623], [1.57, 0, -1.57])
    part_spawner.spawn_part("orange_sensor_99", "sensor", "orange", [-7.7645, 3.4301, 1.0916], [1.57, 0, -1.57])

    part_spawner.destroy_node()
    rclpy.shutdown()

