#!/usr/bin/env python3

from rclpy.node import Node

from ariac_gazebo.spawn_params import GazeboSpawnParams
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('sensor_tf_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.transforms = []
    
    def generate_transform(self, params: GazeboSpawnParams):
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
        
        self.transforms.append(t)
    
    def send_transforms(self):
        self.tf_static_broadcaster.sendTransform(self.transforms)