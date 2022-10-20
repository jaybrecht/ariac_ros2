#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from ariac_gazebo.spawn_params import GazeboSpawnParams

from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity

class GazeboRobotSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_robot_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass
            # self.get_logger().info('spawn service not available, waiting again...')

    def spawn_from_params(self, params: GazeboSpawnParams) -> bool:
        req = SpawnEntity.Request()

        req.name = params.name
        req.robot_namespace = params.robot_namespace
        req.initial_pose = params.initial_pose
        req.reference_frame = params.reference_frame

        if not params.file_path:
            req.xml = self.get_xml_from_topic(params.topic_name) 
        else:
            req.xml = self.get_xml_from_file(params.file_path)

        if req.xml == '':
            return False

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().success

    def get_xml_from_topic(self, topic: str) -> str:
        entity_xml = ''

        def entity_xml_cb(msg):
            nonlocal entity_xml
            entity_xml = msg.data

        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription = self.create_subscription(String, topic, entity_xml_cb, latched_qos)

        while rclpy.ok() and entity_xml == '':
            # self.get_logger().info(f'Waiting for entity xml on {topic}', throttle_duration_sec=1)
            rclpy.spin_once(self)
            pass

        return entity_xml

    def get_xml_from_file(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError as e:
            self.get_logger().error(f'Error reading file from {file_path}: {e}')
            return ''
        
        return entity_xml