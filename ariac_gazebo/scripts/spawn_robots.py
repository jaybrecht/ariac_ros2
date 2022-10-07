#!/usr/bin/env python3

import math
import os
import time

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class RobotSpawnParams:
    def __init__(self, name, ns='', file_path=None, rf='', x=0.0, y=0.0, z=0.0, R=0.0, P=0.0, Y=0.0):
        self.name = name
        self.robot_namespace = ns
        self.initial_pose = self.set_initial_pose(x, y, z, R, P, Y)
        self.file_path = file_path
        self.topic_name = None
        self.reference_frame = rf
        
        if not self.file_path:
            self.topic_name = self.name + '/robot_description'

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

class GazeboRobotSpawner(Node):
    def __init__(self):
        super().__init__('gazebo_robot_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('spawn service not available, waiting again...')

    def spawn_from_params(self, params: RobotSpawnParams) -> bool:
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
            self.get_logger().info(f'Waiting for entity xml on {topic}', throttle_duration_sec=1)
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

def main():
    rclpy.init()

    # Create spawn params for the URDF robots
    robot_params = []
    robot_names = ['floor_robot', 'ceiling_robot', 'agv1', 'agv2', 'agv3', 'agv4']
    
    for name in robot_names:
        robot_params.append(RobotSpawnParams(name))

    # Spawn the robots into gazebo
    robot_spawner = GazeboRobotSpawner()
    robot_spawner.get_logger().info("Spawner started")

    for params in robot_params:
        if not robot_spawner.spawn_from_params(params):
            robot_spawner.get_logger().error(f"Unable to spawn {params.name}")

    time.sleep(2)

    # Create spawn params for the mobile robot
    model_path = os.path.join(get_package_share_directory('ariac_mobile_robot'), 'models', "mobile_robot", 'model.sdf')

    mobile_robot_params = RobotSpawnParams('mobile_robot', file_path=model_path, x=-4.0, y=3.5, Y=3.14)
    robot_spawner.spawn_from_params(mobile_robot_params)

    robot_spawner.get_logger().info("Spawned all robots")

    robot_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()