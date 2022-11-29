#!/usr/bin/env python3

import math
import yaml
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from ariac_gazebo.tf2_geometry_msgs import do_transform_pose
from ariac_gazebo.utilities import quaternion_from_euler, euler_from_quaternion, convert_pi_string_to_float

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose

from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Empty
from ariac_gazebo.spawn_params import (
    SpawnParams, 
    RobotSpawnParams, 
    SensorSpawnParams,
    PartSpawnParams,
    TraySpawnParams)

class EnvironmentStartup(Node):
    def __init__(self):
        super().__init__('environment_startup_node')

        self.trial_config = {}
        self.user_config = {}

        self.declare_parameter('floor_robot_description', '', ParameterDescriptor(description='Floor robot description'))
        self.declare_parameter('ceiling_robot_description', '', ParameterDescriptor(description='Ceiling robot description'))
        self.declare_parameter('agv1_description', '', ParameterDescriptor(description='AGV1 robot description'))
        self.declare_parameter('agv2_description', '', ParameterDescriptor(description='AGV2 robot description'))
        self.declare_parameter('agv3_description', '', ParameterDescriptor(description='AGV3 robot description'))
        self.declare_parameter('agv4_description', '', ParameterDescriptor(description='AGV4 robot description'))

        self.robot_names = [
            'floor_robot', 
            'ceiling_robot', 
            'agv1', 
            'agv2', 
            'agv3', 
            'agv4']

        # Read parameters for robot descriptions
        self.get_robot_descriptions_from_parameters()
        
        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Create pause and unpause clients
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.unpause_client = self.create_client(Empty, "/unpause_physics")

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spawn_sensors(self):
        try:
            sensors = self.user_config['sensors']
        except KeyError:
            self.get_logger().warn("No sensors found in config")

        for sensor_name in sensors:
            sensor_type = sensors[sensor_name]['type']
            xyz = sensors[sensor_name]['pose']['xyz']
            rpy = sensors[sensor_name]['pose']['rpy']
            
            if 'visualize_fov' in sensors[sensor_name].keys():
                vis = sensors[sensor_name]['visualize_fov']
            else:
                vis = False

            params = SensorSpawnParams(sensor_name, sensor_type, visualize=vis, xyz=xyz, rpy=rpy)
            self.spawn_entity(params)

    def spawn_kit_trays(self):
        possible_slots = [1, 2, 3, 4, 5, 6]
        possible_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

        slot_info = {
            1: {"table": "table_1", "offset": -0.43},
            2: {"table": "table_1", "offset": 0.0},
            3: {"table": "table_1", "offset": 0.43},
            4: {"table": "table_2", "offset": -0.43},
            5: {"table": "table_2", "offset": 0.0},
            6: {"table": "table_2", "offset": 0.43},
        }

        # Check that input is valid
        try:
            ids = self.trial_config["kitting_trays"]["tray_ids"]
            slots = self.trial_config["kitting_trays"]["slots"]
        except KeyError:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if len(ids) == 0 or len(slots) == 0:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if not (len(ids) == len(slots)):
            self.get_logger().warn("Number of trays does not equal number of slots")
            return

        for id, slot in zip(ids, slots):
            if not type(id) == int or not type(slot) == int:
                self.get_logger().warn("Tray ids and slots must be integers")
                return
            elif id not in possible_ids:
                self.get_logger().warn("Tray id must be between 0 and 9")
                return
            elif slot not in possible_slots:
                self.get_logger().warn("Tray slot must be between 1 and 6")
                return

        # Calculate location of tables using tf
        transforms = {}
        try:
            transforms['table_1'] = self.tf_buffer.lookup_transform('world', "kts1_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform kts1_table_frame to world: {ex}')
            return

        try:
            transforms['table_2'] = self.tf_buffer.lookup_transform('world', "kts2_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform kts1_table_frame to world: {ex}')
            return

        # Spawn trays 
        num_trays = 0
        for id, slot in zip(ids, slots):
            rel_pose = Pose()
            rel_pose.position.x = slot_info[slot]["offset"]
            rel_pose.position.z = .01

            world_pose = do_transform_pose(rel_pose, transforms[slot_info[slot]["table"]])
            
            # Create unique name for each tray that includes the id
            marker_id = str(id).zfill(2)
            name = "kit_tray_" +  marker_id + "_" + str(num_trays)
            num_trays += 1

            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            rpy = euler_from_quaternion(world_pose.orientation)

            params = TraySpawnParams(name, marker_id, xyz=xyz, rpy=rpy)

            self.spawn_entity(params)

    def spawn_bin_parts(self):
        possible_slots = list(range(1,10))
        possible_bins = ['bin1', 'bin2', 'bin3', 'bin4', 'bin5', 'bin6', 'bin7', 'bin8']

        part_heights = {
            'battery': 0.04,
            'sensor': 0.07,
            'pump': 0.12,
            'regulator': 0.07,
        }

        slot_info = {
            1: {"x_offset": -0.18, "y_offset": -0.18},
            2: {"x_offset": -0.18, "y_offset": 0.0},
            3: {"x_offset": -0.18, "y_offset": 0.18},
            4: {"x_offset": 0.0, "y_offset": -0.18},
            5: {"x_offset": 0.0, "y_offset": 0.0},
            6: {"x_offset": 0.0, "y_offset": 0.18},
            7: {"x_offset": 0.18, "y_offset": -0.18},
            8: {"x_offset": 0.18, "y_offset": 0.0},
            9: {"x_offset": 0.18, "y_offset": 0.18},
        }

        # Validate input
        try:
            bin_parts = self.trial_config["parts"]["bins"]
        except KeyError:
            self.get_logger().warn("No bin parts found in configuration")
            return

        part_count = 0
        for bin_name in bin_parts.keys():
            if not bin_name in possible_bins:
                self.get_logger().warn(f"{bin_name} is not a valid bin name")
                continue
        
            try:
                bin_transform = self.tf_buffer.lookup_transform('world', bin_name + "_frame", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {bin_name}_frame to world: {ex}')
                return

            available_slots = list(range(1,10))
            for part_info in bin_parts[bin_name]:
                
                try:
                    part_type = part_info['type']
                except KeyError:
                    self.get_logger().warn("Part type is not specified")
                    continue

                try:
                    part_color = part_info['color']
                except KeyError:
                    self.get_logger().warn("Part color is not specified")
                    continue

                try:
                    slots = part_info['slots']
                    if not type(slots) == list:
                        self.get_logger().warn("slots parameter should be a list of integers")
                        continue
                except KeyError:
                    self.get_logger().warn("Part slots are not specified")
                    continue

                try:
                    yaw = convert_pi_string_to_float(str(part_info['rotation']))
                except KeyError:
                    yaw = 0

                try:
                    flipped = part_info['flipped']
                    if not type(flipped) == bool:
                        self.get_logger().warn("flipped parameter should be either true or false")
                        flipped = False
                except KeyError:
                    flipped = False

                if not part_type in PartSpawnParams.part_types:
                    self.get_logger().warn(f"{part_info['type']} is not a valid part type")
                    continue
                
                if  not part_color in PartSpawnParams.colors:
                    self.get_logger().warn(f"{part_info['color']} is not a valid part color")
                    continue

                # Spawn parts into slots
                for slot in slots:
                    if not slot in possible_slots:
                        self.get_logger().warn(f"Slot {slot} is not a valid option")
                        continue
                    elif not slot in available_slots:
                        self.get_logger().warn(f"Slot {slot} is already occupied")
                        continue
                    
                    available_slots.remove(slot)
                    
                    part_name = part_type + "_" + part_color + "_b" + str(part_count).zfill(2)
                    part_count += 1

                    if flipped:
                        roll = math.pi
                    else:
                        roll = 0
                    
                    q = quaternion_from_euler(roll, 0, yaw)
                    rel_pose = Pose()
                    rel_pose.position.x = slot_info[slot]["x_offset"]
                    rel_pose.position.y = slot_info[slot]["y_offset"]

                    if flipped:
                        rel_pose.position.z = part_heights[part_type]

                    rel_pose.orientation.w = q[0]
                    rel_pose.orientation.x = q[1]
                    rel_pose.orientation.y = q[2]
                    rel_pose.orientation.z = q[3]

                    world_pose = do_transform_pose(rel_pose, bin_transform)

                    xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
                    rpy = euler_from_quaternion(world_pose.orientation)

                    params = PartSpawnParams(part_name, part_type, part_color, xyz=xyz, rpy=rpy)

                    self.spawn_entity(params)

    def spawn_robots(self):
        for name in self.robot_names:
            urdf = ET.fromstring(self.robot_descriptions[name])
            params = RobotSpawnParams(name, ET.tostring(urdf, encoding="unicode"))
            self.spawn_entity(params)

    def spawn_parts_on_agvs(self):
        pass

    def get_robot_descriptions_from_parameters(self):
        self.robot_descriptions = {}

        for name in self.robot_names:
            self.robot_descriptions[name] = self.get_parameter(name + '_description').value

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

    def read_yaml(self, path):
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}

    def pause_physics(self):
        self.pause_client.wait_for_service()

        request = Empty.Request()

        self.pause_client.call_async(request)
    
    def unpause_physics(self):
        self.unpause_client.wait_for_service()

        request = Empty.Request()

        self.unpause_client.call_async(request)