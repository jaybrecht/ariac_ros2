#!/usr/bin/env python
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from timeit import default_timer as timer
import geometry_msgs.msg
import tf2_ros
import math
import rclpy
from rclpy.duration import Duration
from ariac_gazebo.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Vector3
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.msg import *

from ariac_msgs.srv import TeleportHuman 
#human_teleport_plugin 
#ariac.teleport_human

# https://navigation.ros.org/commander_api/index.html

class stopbase_server(Node):
	def __init__(self):
		#global pub_eom, tfBuffer, listener
		self.name='stopbase_server'
		super().__init__(self.name)
		#self.pub_eom = self.create_publisher(topic='move_base_result', msg_type=Bool, qos_profile = 10)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer,node=self)
		#self.create_subscription(topic='jason_stop_human_old'    , msg_type=Vector3, callback=self.goal_stop, qos_profile = 10)
		self.create_subscription(topic='jason_teleport_human_old', msg_type=Bool, callback=self.goal_teleport, qos_profile = 10)

		#Teleport service client:
		self.cli = self.create_client(TeleportHuman, '/ariac/teleport_human')
		while not self.cli.wait_for_service(timeout_sec=1.0):
		    self.get_logger().info('teleport service not available, waiting again...')
		self.req = TeleportHuman.Request()


	#LB: callback function to perform human TELEPORT
	#1) import the service from the given package: from ariac_human import 
	#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html
	def goal_teleport(self, data):		
		self.get_logger().info("Testing: Calling Teleport service")	
		#response = send_request()	
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return

	def send_request(self):
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	#LB: callback function to perform human STOP
	#def goal_stop(self, data):		
	#	self.get_logger().info("First cancelTask")
	#	#self._navigator.cancelTask()
	#	#self._navigator.spin(0.25)  #spin_dist=0.25, time_allowance=10)
	#	#self.get_logger().info("After spin")
	#	return

if __name__ == '__main__':
    global node
    rclpy.init(args=None)
    node = stopbase_server()
    rclpy.spin(node)
    rclpy.shutdown()

