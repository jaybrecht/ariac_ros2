#!/usr/bin/env python3

import math
import os
import yaml

import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class SensorSpawnParams:
    def __init__(self, name, sensor_type, ns='', rf='', x=0.0, y=0.0, z=0.0, R=0.0, P=0.0, Y=0.0):
        self.name = name
        self.sensor_type = sensor_type
        self.robot_namespace = ns
        self.initial_pose = self.set_initial_pose(x, y, z, R, P, Y)
        self.file_path = os.path.join(get_package_share_directory('ariac_sensors'), 'models', sensor_type, 'model.sdf')
        self.reference_frame = rf

    def set_initial_pose(self, x, y, z, R, P, Y):
        initial_pose = Pose()
        initial_pose.position.x = x
        initial_pose.position.y = y
        initial_pose.position.z = z

        q = quaternion_from_euler(R, P, Y)
        initial_pose.orientation.w = q[0]
        initial_pose.orientation.x = q[1]
        initial_pose.orientation.y = q[2]
        initial_pose.orientation.z = q[3]

        return initial_pose
    
def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class GazeboSensorSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_sensor_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            pass

    def spawn_from_params(self, params: SensorSpawnParams) -> bool:        
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

    def modify_xml(self, params: SensorSpawnParams):
        xml = ET.fromstring(self.get_xml_from_file(params.file_path))

        if params.sensor_type == "break_beam":
            plugin = xml.find('model').find('link').find('sensor').find('plugin')

            plugin.set('name', str(params.name + "_ros_plugin"))
            plugin.find('ros').find('namespace').text = "ariac/" + params.name
            plugin.find('ros').find('remapping').text = "~/out:=scan"
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

    def generate_transform(self, params: SensorSpawnParams):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = params.name + "_frame"

        t.transform.translation.x = params.initial_pose.position.x
        t.transform.translation.y = params.initial_pose.position.y
        t.transform.translation.z = params.initial_pose.position.z
        t.transform.rotation.x = params.initial_pose.orientation.x
        t.transform.rotation.y = params.initial_pose.orientation.y
        t.transform.rotation.z = params.initial_pose.orientation.z
        t.transform.rotation.w = params.initial_pose.orientation.w
        
        return t 


def main():
    rclpy.init()

    sensor_spawner = GazeboSensorSpawner()
    sensor_params = []
    transforms = []

    config = os.path.join(get_package_share_directory('ariac_gazebo'), 'config', "sensors.yaml")

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    try:
        sensors = data['sensors']
    except KeyError:
        sensor_spawner.get_logger().warn("No sensors found in config")

    for sensor_name in sensors:
        type = sensors[sensor_name]['type']
        x, y, z = sensors[sensor_name]['pose']['xyz']
        roll, pitch, yaw = sensors[sensor_name]['pose']['rpy']

        sensor_params.append(SensorSpawnParams(sensor_name, type, x=float(x), y=float(y), z=float(z), R=float(roll), P=float(pitch), Y=float(yaw)))

    # Spawn the robots into gazebo
    for params in sensor_params:
        if not sensor_spawner.spawn_from_params(params):
            sensor_spawner.get_logger().error(f"Unable to spawn {params.name}")
        else:
            transforms.append(sensor_spawner.generate_transform(params))

    # Send tf transforms
    sensor_spawner.tf_static_broadcaster.sendTransform(transforms)

    try:
        rclpy.spin(sensor_spawner)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()