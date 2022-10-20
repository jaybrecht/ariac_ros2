#!/usr/bin/env python3

import os

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from ariac_gazebo.spawn_params import GazeboSpawnParams

from gazebo_msgs.srv import SpawnEntity
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class GazeboSensorSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_sensor_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def spawn_from_params(self, params: GazeboSpawnParams) -> bool:        
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
        
        return future.result().success

    def modify_xml(self, params: GazeboSpawnParams):
        xml = ET.fromstring(self.get_xml_from_file(params.file_path))

        xml.find('model').find('link').find('sensor').find("visualize").text = str(params.visualize)

        if params.sensor_type == "break_beam":
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(params.name + "_ros_plugin"))
            plugin.find('ros').find('namespace').text = "/ariac" 
            plugin.find('status_topic').text = params.name + "_status"
            plugin.find('change_topic').text = params.name + "_change"
            plugin.find('frame_name').text = params.name + "_frame"

        ray_sensors = ["proximity_sensor", "laser_profiler", "depth_camera"]
        if params.sensor_type in ray_sensors:
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(params.name + "_ros_plugin"))
            plugin.find('ros').find('namespace').text = "/ariac" 
            plugin.find('ros').find('remapping').text = "~/out:=" + params.name
            plugin.find('frame_name').text = params.name + "_frame"

        if params.sensor_type == 'rgb_camera' or params.sensor_type == 'logical_camera':
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(params.name + "_ros_plugin"))
            plugin.find('ros').find('namespace').text = "/ariac" 
            plugin.find('camera_name').text = params.name
            plugin.find('frame_name').text = params.name + "_frame"

        return ET.tostring(xml, encoding="unicode")

    def get_xml_from_file(self, file_path: str) -> str:
        try:
            f = open(file_path, 'r')
            entity_xml = f.read()
        except IOError as e:
            self.get_logger().error(f'Error reading file from {file_path}: {e}')
            return ''
        
        return entity_xml