#!/usr/bin/env python3

import math
import yaml 
from random import randint

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

from ariac_msgs.msg import ConveyorBeltState

from ariac_gazebo.tf2_geometry_msgs import do_transform_pose
from ariac_gazebo.utilities import (
    convert_pi_string_to_float, 
    euler_from_quaternion, 
    quaternion_from_euler, 
    Part)

from ariac_gazebo.spawn_params import (
    SpawnParams, 
    PartSpawnParams)

class ConveyorBeltPopulator(Node):
    def __init__(self):
        super().__init__('conveyor_belt_populator')

        self.active = False
        self.spawn_rate = None
        self.parts_to_spawn = []
        self.conveyor_transform = None
        self.conveyor_enabled = False
        self.spawn_order_types = ['sequential', 'random']
        self.conveyor_width = 0.28

        # Create subscription to conveyor belt status topic
        self.conveyor_status_sub = self.create_subscription(ConveyorBeltState, 
            '/ariac/conveyor_state', self.conveyor_status, 10)
        
        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.transform_timer = self.create_timer(1.0, self.get_transform)

    def get_transform(self):
        try:
            self.conveyor_transform = self.tf_buffer.lookup_transform('world', 
                "conveyor_belt_part_spawn_frame", rclpy.time.Time())
        except TransformException:
            return
        
        self.get_logger().info("Received conveyor belt transform")

    def read_config(self, config):
        d = self.read_yaml(config)

        if not d:
            return False

        try: 
            conveyor_config = d['parts']['conveyor_belt']
        except KeyError:
            self.get_logger().error("Unable to find conveyor belt params in configuration file")
            return False

        try:
            self.active = conveyor_config['active']
        except KeyError:
            self.get_logger().error("Active paramater not set in conveyor belt configuration")
            return False
        
        if not self.active:
            return False

        try:
            spawn_rate = conveyor_config['spawn_rate']
        except KeyError:
            self.get_logger().error("Spawn rate paramater not set in conveyor belt configuration")
            return False
        
        try:
            self.spawn_rate = float(spawn_rate)
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        self.spawn_timer = self.create_timer(self.spawn_rate, self.populate)

        try:
            if conveyor_config['order'] in self.spawn_order_types:
                self.spawn_order = conveyor_config['order']
            else:
                self.get_logger().error(f"Order paramater must be of type: {self.spawn_order_types}")
                return False
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        try:
            parts = conveyor_config['parts_to_spawn']
        except KeyError:
            self.get_logger().error("Parts to spawn not found in configuration")
            return False

        part_count = 0
        for part_info in parts:
            ret, part = self.parse_part_info(part_info)

            if not ret:
                continue

            if part.flipped:
                self.get_logger().info(f"{part.type} is flipped")
            else:
                self.get_logger().info(f"{part.type} is not flipped")

            try:
                amount = part_info['number']
            except KeyError:
                continue

            try:
                offset = part_info['offset']
                if not -1 <= offset <= 1:
                    self.get_logger().error("Offset must be in range [-1, 1]")
                    offset = 0
            except KeyError:
                offset = 0

            for i in range(amount):
                part_name = part.type + "_" + part.color + "_c" + str(part_count).zfill(2)
                part_count += 1

                roll = math.pi

                if part.flipped:
                    roll = math.pi
                else:
                    roll = 0

                yaw = convert_pi_string_to_float(part.rotation)
                q = quaternion_from_euler(roll, 0, yaw)
                rel_pose = Pose()

                rel_pose.position.y = offset * (self.conveyor_width/2)

                if part.flipped:
                    rel_pose.position.z = part.height

                rel_pose.orientation.w = q[0]
                rel_pose.orientation.x = q[1]
                rel_pose.orientation.y = q[2]
                rel_pose.orientation.z = q[3]
            
                world_pose = do_transform_pose(rel_pose, self.conveyor_transform)

                xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
                rpy = euler_from_quaternion(world_pose.orientation)

                self.parts_to_spawn.append(PartSpawnParams(part_name, part.type, part.color, xyz=xyz, rpy=rpy))
        
        if len(self.parts_to_spawn) > 0:
            return True
        else:
            return False

    def conveyor_status(self, msg: ConveyorBeltState):
        if not self.conveyor_enabled and msg.enabled:
            self.spawn_timer.reset()
        
        self.conveyor_enabled = msg.enabled

    def populate(self):
        if self.conveyor_enabled:
            if self.spawn_order == 'sequential':
                part_params = self.parts_to_spawn.pop(0)
            elif self.spawn_order == 'random':
                part_params = self.parts_to_spawn.pop(randint(0, len(self.parts_to_spawn) - 1))
            
            self.spawn_entity(part_params)

    def parse_part_info(self, part_info):
        part = Part()
        
        try:
            part.type = part_info['type']
            part.height = Part.part_heights[part.type]
        except KeyError:
            self.get_logger().warn("Part type is not specified")
            return (False, part)

        try:
            part.color = part_info['color']
        except KeyError:
            self.get_logger().warn("Part color is not specified")
            return (False, part)

        try:
            part.rotation = str(part_info['rotation'])
        except KeyError:
            pass

        try:
            part.flipped = part_info['flipped']
            if not type(part.flipped) == bool:
                self.get_logger().warn("flipped parameter should be either true or false")
                part.flipped = False
        except KeyError:
            pass

        if not part.type in PartSpawnParams.part_types:
            self.get_logger().warn(f"{part_info['type']} is not a valid part type")
            return (False, part)
        
        if  not part.color in PartSpawnParams.colors:
            self.get_logger().warn(f"{part_info['color']} is not a valid part color")
            return (False, part)
        
        return (True, part)
            
    def spawn_entity(self, params: SpawnParams) -> bool:
        self.spawn_client.wait_for_service()

        self.get_logger().info(f'Spawning: {params.name}')

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        self.spawn_client.call_async(req)
    
    def read_yaml(self, path):
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}