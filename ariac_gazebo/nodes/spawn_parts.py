#!/usr/bin/env python3

import os
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.spawn_params import GazeboSpawnParams

#!/usr/bin/env python3

import os

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from ariac_gazebo.spawn_params import GazeboSpawnParams

from gazebo_msgs.srv import SpawnEntity

class PartSpawner(Node):
    def __init__(self):
        super().__init__('part_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def spawn_from_params(self, params: GazeboSpawnParams) -> bool:        
        # Send spawn request
        req = SpawnEntity.Request()

        req.name = params.name
        req.robot_namespace = params.robot_namespace
        req.initial_pose = params.initial_pose
        req.reference_frame = params.reference_frame

        req.xml = self.get_xml_from_file(params.file_path)

        if req.xml == '':

            return False

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            return True
        else:
            self.get_logger().error(future.result().status_message)
            return False   

    def get_xml_from_file(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError as e:
            self.get_logger().error(f'Error reading file from {file_path}: {e}')
            return ''
        
        return entity_xml

def main():
    rclpy.init()

    part_spawner = PartSpawner()

    path = get_package_share_directory("ariac_gazebo") + "/models/assembly_battery_blue_ariac/model.sdf"

    blue_battery = GazeboSpawnParams("blue_battery", file_path=path, rf="world", xyz=[-1.9, 3.38, 0.76])

    if not part_spawner.spawn_from_params(blue_battery):
        part_spawner.get_logger().error("Unable to read xml")

    rclpy.shutdown()


if __name__ == '__main__':
    main()