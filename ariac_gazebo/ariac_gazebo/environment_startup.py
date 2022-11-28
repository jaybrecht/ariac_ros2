#!/usr/bin/env python3

import yaml
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        pass

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
        
    def get_position_of_object(self, object_name):
        try:
            t = self.tf_buffer.lookup_transform('world', object_name + "_frame", rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(f'Could not find {object_name} in tf_tree')
            return [0, 0, 0]

        return [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]

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