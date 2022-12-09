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

import geometry_msgs.msg
import tf2_ros
import math
import rclpy
from rclpy.duration import Duration
from ariac_gazebo.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Vector3
# import tf2_geometry_msgs  # import support for transforming geometry_msgs stamped msgs
import numpy as np
from rclpy.node import Node
from std_msgs.msg import *
from ariac_msgs.msg import *
# from nist_gear.msg import *
#from monitor.msg import *

class movebase_server(Node):
	def __init__(self):
		global pub_speed, pub_position, tfBuffer, listener
		self.name='movebase_server'
		super().__init__(self.name)
		tfBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(tfBuffer,node=self)
		self.create_subscription(topic='jason_to_move_base', msg_type=Vector3, callback=self.goal_movebase, qos_profile = 1000)

		self._navigator = BasicNavigator()
		# Set our demo's initial pose
		initial_pose = PoseStamped()
		initial_pose.header.frame_id = 'map'
		initial_pose.header.stamp = self._navigator.get_clock().now().to_msg()
		initial_pose.pose.position.x = -15.0
		initial_pose.pose.position.y = -10.0
		initial_pose.pose.position.z = 0.0
		initial_pose.pose.orientation.x = 0.0
		initial_pose.pose.orientation.y = 0.0
		initial_pose.pose.orientation.z = 0.0
		initial_pose.pose.orientation.w = 1.0
		self.current_pose = initial_pose
		self._navigator.setInitialPose(self.current_pose)
		# Wait for navigation to fully activate, since autostarting nav2
		self._navigator.waitUntilNav2Active()
		#self._navigator.lifecycleStartup()

	#LB: callback function to support moving
	def goal_movebase(self, data):
		
		#rospy.loginfo(rospy.get_caller_id() + "I heard %f and %f and %f",data.x,data.y,data.z)
		try:
			result = self.send_to_movebase(data.x,data.y,data.z)
			if result:
				self.get_logger().info("Movebase Goal execution done!")
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
		goal_pose.pose.orientation.w = 1.0

		# sanity check a valid path exists
		path = self._navigator.getPath(self.current_pose, goal_pose)
		self.current_pose = goal_pose

		self._navigator.goToPose(goal_pose)

		i = 0
		while not self._navigator.isTaskComplete():
			################################################
			#
			# Implement some code here for your application!
			#
			################################################

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
				if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
					goal_pose.pose.position.x = -3.0
					self._navigator.goToPose(goal_pose)

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

		#rospy.loginfo("")
		#return True
	#	rospy.loginfo("Goal received")
	#	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	#	client.wait_for_server()
	#	rospy.loginfo("After wait")
	#	goal = MoveBaseGoal()
	#	goal.target_pose.header.frame_id = "map"
	#	goal.target_pose.header.stamp = rospy.Time.now()
	#	goal.target_pose.pose.position.x = x
	#	goal.target_pose.pose.position.y = y
	#	goal.target_pose.pose.position.z = z
	#	goal.target_pose.pose.orientation.w = 1.0

	#	client.send_goal(goal)
	#	wait = client.wait_for_result()
	#	if not wait:
	#		rospy.logerr("Action server not available!")
	#		rospy.signal_shutdown("Action server not available!")
	#	else:
	#		return client.get_result()


if __name__ == '__main__':
    global node
    rclpy.init(args=None)
    node = movebase_server()
    rclpy.spin(node)
    rclpy.shutdown()


