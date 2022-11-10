#!/usr/bin/env python3

import sys
import os

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity

from ariac_gazebo.utilities import pose_info

class PartSpawnParams:
    def __init__(self, name, file_path=None, xyz=[0,0,0], rpy=[0,0,0], color=None, ns='', rf=''):
        self.name = name
        self.robot_namespace = ns
        self.initial_pose = pose_info(xyz, rpy)
        self.file_path = file_path
        self.reference_frame = rf
        self.color = color

class PartSpawner(Node):
    colors = {
        'blue': (0, 0, 168),
        'green': (0, 100, 0),
        'red': (139, 0, 0),
        'purple': (138, 0, 226),
        'orange': (255, 140, 0)   
    }

    types = ['battery', 'pump', 'regulator', 'sensor']

    def __init__(self):
        super().__init__('part_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def spawn_from_params(self, params: PartSpawnParams) -> bool:        
        # Send spawn request
        req = SpawnEntity.Request()

        req.name = params.name
        req.robot_namespace = params.robot_namespace
        req.initial_pose = params.initial_pose
        req.reference_frame = params.reference_frame
        
        req.xml = self.modify_xml(params)

        if req.xml == '':
            return False

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            return True
        else:
            self.get_logger().error(future.result().status_message)
            return False   
    
    def modify_xml(self, params: PartSpawnParams):
        xml = ET.fromstring(self.get_xml_from_file(params.file_path))

        r, g, b = self.colors[params.color]
        color_string = str(r/255) + " " + str(g/255) + " " + str(b/255) + " 1" 

        for elem in xml.find('model').find('link').findall('visual'):
            if elem.attrib['name'] == "base":
                elem.find("material").find("ambient").text = color_string
                elem.find("material").find("diffuse").text = color_string

        return ET.tostring(xml, encoding="unicode")
    
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

    part_type = sys.argv[1]
    color = sys.argv[2]

    if part_type not in PartSpawner.types:
        print("type not valid")
        exit()

    if color not in PartSpawner.colors.keys():
        print("type not valid")
        exit()

    part_spawner = PartSpawner()

    sdf = get_package_share_directory("ariac_gazebo") + "/models/" + part_type + "/model.sdf"

    part = PartSpawnParams(color + '_' + part_type, color=color, file_path=sdf, rf="world", xyz=[-1.9, 3.38, 0.73])

    if not part_spawner.spawn_from_params(part):
        part_spawner.get_logger().error("Unable to read xml")

    rclpy.shutdown()


if __name__ == '__main__':
    main()