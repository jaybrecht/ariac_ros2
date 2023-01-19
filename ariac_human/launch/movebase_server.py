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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ariac_gazebo.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Vector3
# import tf2_geometry_msgs  # import support for transforming geometry_msgs stamped msgs
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Bool
from ariac_msgs.msg import *
from ariac_msgs.srv import TeleportHuman 

# https://navigation.ros.org/commander_api/index.html

class movebase_server(Node):
	def __init__(self):
		#global pub_eom, tfBuffer, listener
		self.name='movebase_server'
		super().__init__(self.name)
		self.pub_eom = self.create_publisher(topic='move_base_result', msg_type=Bool, qos_profile = 10)
		self.tfBuffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tfBuffer,node=self)
		
		my_callback_group = ReentrantCallbackGroup()
		#move_cb_group = client_cb_group
		
		#self.create_subscription(topic='jason_to_move_base', msg_type=Vector3, callback=self.goal_movebase, qos_profile = 10)
		#self.create_subscription(topic='jason_stop_human'  , msg_type=Vector3, callback=self.goal_stop, qos_profile = 10)
		
		my_subscription1 = self.create_subscription(Vector3, "/jason_stop_human",     self.goal_stop,     qos_profile=1, callback_group=my_callback_group)
		my_subscription2 = self.create_subscription(Vector3, "/jason_teleport_human", self.goal_teleport, qos_profile=1, callback_group=my_callback_group)
		my_subscription3 = self.create_subscription(Vector3, "/jason_to_move_base",   self.goal_movebase, qos_profile=1, callback_group=my_callback_group)
		
		#Teleport service client:
		self.cli = self.create_client(TeleportHuman, '/ariac/teleport_human')
		while not self.cli.wait_for_service(timeout_sec=1.0):
		    self.get_logger().info('teleport service not available, waiting again...')
		self.req = TeleportHuman.Request()

		self._navigator = BasicNavigator()
		# Set our demo's initial pose
		self.initial_pose = PoseStamped()
		self.initial_pose.header.frame_id = 'map'
		self.initial_pose.header.stamp = self._navigator.get_clock().now().to_msg()
		self.initial_pose.pose.position.x = -15.0
		self.initial_pose.pose.position.y = -10.0
		self.initial_pose.pose.position.z = 0.0
		self.initial_pose.pose.orientation.x = 0.0
		self.initial_pose.pose.orientation.y = 0.0
		self.initial_pose.pose.orientation.z = 0.0
		self.initial_pose.pose.orientation.w = 1.0
		self.current_pose = self.initial_pose
		self._navigator.setInitialPose(self.current_pose)
		# Wait for navigation to fully activate, since autostarting nav2
		self._navigator.waitUntilNav2Active()
			
	#LB: callback function to perform human TELEPORT
	def goal_teleport(self, data):		
		self.get_logger().info("Calling Teleport service")	
		self._navigator.cancelTask()
		self.stopMove = True
		#self._navigator.clearAllCostmaps()
		self.future = self.cli.call_async(self.req)
		#rclpy.spin_until_future_complete(self, self.future)
		self._navigator.setInitialPose(self.initial_pose)
		return

	#LB: callback function to perform human STOP
	def goal_stop(self, data):		
		self.get_logger().info("Stop goal activated")
		self.stopMove = True
		#self._navigator.cancelTask()
		#self._navigator.spin(spin_dist=1.25)
		return

	#LB: callback function to perform human MOVE
	def goal_movebase(self, data):		
		msg = Bool()
		msg.data = True
		#self._navigator.cancelTask()
		try:
			self.stopMove = False
			result = self.send_to_movebase(data.x,data.y,data.z)
			if result == TaskResult.SUCCEEDED:
				self.get_logger().info("Movebase Goal execution done!")
				self.pub_eom.publish(msg)

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
			return

	def send_to_movebase(self, x,y,z):
		print('send-to-move is active ')  
		self.get_logger().info('Starting send-to-move')

		# Go to our demos first goal pose
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = self._navigator.get_clock().now().to_msg()
		goal_pose.pose.position.x = x
		goal_pose.pose.position.y = y
		# goal_pose.pose.orientation.w = z #LB: before as 1.0
		# new try on 13/01
		goal_pose.pose.orientation.z = 1.0
		goal_pose.pose.orientation.w = 0.0
		

		# sanity check a valid path exists
		path = self._navigator.getPath(self.current_pose, goal_pose)
		self.current_pose = goal_pose

		self._navigator.goToPose(goal_pose)

		i = 0
		#read time
		# nooooooooooooooow()- previous > 30s
		#iniTime = datetime.datetime.now()
		iniTime = timer()
		
		while not self._navigator.isTaskComplete():
#		while not (self._navigator.isTaskComplete() && (datetime.datetime.now() - iniTime).totalseconds() > 30):
			################################################
			#
			# Implement some code here for your application!
			#
			################################################
			#if (timer() - iniTime) > 80.0:
			if self.stopMove: 
				#print('50s Timeout in movebase_server: cancelling task')
				print('Stopping movement')
				self._navigator.cancelTask()
				#return TaskResult.SUCCEEDED;
 
			# Do something with the feedback
			i = i + 1
			feedback = self._navigator.getFeedback()
			if feedback and i % 5 == 0:
				print('Estimated time of arrival: ' + '{0:.0f}'.format(
					Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
					+ ' seconds.')

				# Some navigation timeout to demo cancellation
				if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
					self._navigator.cancelTask()

				# Some navigation request change to demo preemption
				#if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
				#	goal_pose.pose.position.x = -3.0
				#	self._navigator.goToPose(goal_pose)

		# Do something depending on the return code
		result = self._navigator.getResult()
		if result == TaskResult.SUCCEEDED:
			print('Goal succeeded!')
		elif result == TaskResult.CANCELED:
			print('Goal was canceled!')
		elif result == TaskResult.FAILED:
			print('Goal failed!')
		else:
			print('Goal has an invalid return status!')

		return result


if __name__ == '__main__':
    global node
    rclpy.init(args=None)
    node = movebase_server()
    #rclpy.spin(node)
    #rclpy.shutdown()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    executor.spin()

    #try:
    #    node.get_logger().info('Beginning client, shut down with CTRL-C')
    #    executor.spin()
    #except KeyboardInterrupt:
    #    node.get_logger().info('Keyboard interrupt, shutting down.\n')
        
    node.destroy_node()
    rclpy.shutdown()

