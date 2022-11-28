#!/usr/bin/env python3

import yaml
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from ariac_gazebo.tf2_geometry_msgs import do_transform_pose
from ariac_gazebo.utilities import quaternion_from_euler, euler_from_quaternion

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

            world_pose = do_transform_pose(rel_pose, transforms[slot_info[slot]["table"]])
            
            # Create unique name for each tray that includes the id
            marker_id = str(id).zfill(2)
            name = "kit_tray_" +  marker_id + "_" + str(num_trays)

            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            rpy = euler_from_quaternion(world_pose.orientation)

            params = TraySpawnParams(name, marker_id, xyz=xyz, rpy=rpy)

            self.spawn_entity(params)

    def spawn_bin_parts(self):
        pass

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