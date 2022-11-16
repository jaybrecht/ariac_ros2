#!/usr/bin/env python3

import os

import yaml

from copy import deepcopy

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity

from ariac_gazebo.utilities import pose_info

class TraySpawnParams:
    def __init__(self, name, file_path=None, xyz=[0,0,0], rpy=[0,0,0], id="00", ns='', rf=''):
        self.name = name
        self.robot_namespace = ns
        self.initial_pose = pose_info(xyz, rpy)
        self.file_path = file_path
        self.reference_frame = rf
        self.marker_id = id

class TraySpawner(Node):
    marker_ids = ["00", "01", "02", "03", "04", "05", "06", "07", "08", "09"]

    table_poses = {
        'table_1': {'xyz': [-1.3, -5.84, 0.725], 'rpy': [0, 0, 'pi']},
        'table_2': {'xyz': [-1.3, 5.84, 0.725], 'rpy': [0, 0, 0]}
    }
    
    slots = {
        'slot_1': {"table": "table_1", "offset": 0.43},
        'slot_2': {"table": "table_1", "offset": 0.0},
        'slot_3': {"table": "table_1", "offset": -0.43},
        'slot_4': {"table": "table_2", "offset": -0.43},
        'slot_5': {"table": "table_2", "offset": 0.0},
        'slot_6': {"table": "table_2", "offset": 0.43},
    }

    def __init__(self):
        super().__init__('tray_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def spawn_from_params(self, params: TraySpawnParams) -> bool:        
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
    
    def modify_xml(self, params: TraySpawnParams):
        xml = ET.fromstring(self.get_xml_from_file(params.file_path))

        marker_string = "model://kit_tray/meshes/markers/marker_" + params.marker_id + ".dae"
        for elem in xml.find('model').find('link').findall('visual'):
            if elem.attrib['name'] == "marker":
                elem.find("geometry").find("mesh").find("uri").text = marker_string

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

    tray_spawner = TraySpawner()

    config = os.path.join(get_package_share_directory('ariac_gazebo'), 'config', "trays.yaml")

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    tray_ids = data["kitting_trays"]["tray_ids"]
    slots = data["kitting_trays"]["slots"]

    # Check that lists are the same length 
    if not len(tray_ids) == len(slots):
        tray_spawner.get_logger().error("Number of tray_ids must match slots")
        rclpy.shutdown()
        exit()

    # Check for no duplicates in slots
    if not len(slots) == len(set(slots)):
        tray_spawner.get_logger().error("Tray slots are not unique")
        rclpy.shutdown()
        exit()

    sdf = get_package_share_directory("ariac_gazebo") + "/models/kit_tray/model.sdf"

    count = 0

    tray_params = []
    for id, slot in zip(tray_ids, slots):
        id_string = str(id).zfill(2)
        if not id_string in tray_spawner.marker_ids:
            tray_spawner.get_logger().warn(f"Marker {id} is not an option for the trays")
            continue
        
        slot_string = "slot_" + str(slot)

        if not slot_string in tray_spawner.slots.keys():
            tray_spawner.get_logger().warn(f"Slot {slot} is not an option for the trays")
            continue

        slot_info = tray_spawner.slots[slot_string]
        xyz = deepcopy(tray_spawner.table_poses[slot_info["table"]]["xyz"])

        xyz[0] += slot_info["offset"]

        rpy = deepcopy(tray_spawner.table_poses[slot_info["table"]]["rpy"])
        
        name = "kit_tray_" + id_string + "_" + str(count)

        tray_params.append(TraySpawnParams(name=name, file_path=sdf, id=id_string, xyz=xyz, rpy=rpy))

        count += 1

    for params in tray_params:
        if not tray_spawner.spawn_from_params(params):
            tray_spawner.get_logger().error("Unable to spawn tray")

    rclpy.shutdown()


if __name__ == '__main__':
    main()