#!/usr/bin/env python3

import sys
from threading import Event

import rclpy

from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from ariac_msgs.srv import MoveAGV


class AGV_Mover(Node):
    def __init__(self, agv_number):
        super().__init__('agv_mover')
        
        self.rate = self.create_rate(1)
        self.recieved_joint_states = False

        self.action_done_event = Event()
        self.action_result = None
        self.callback_group = ReentrantCallbackGroup()

        self.agv_joint_names = []
        self.agv_positions = []

        self.joint_state_sub = self.create_subscription(JointState, agv_number + '/joint_states', self.agv_joint_state_cb, 10)
        self._action_client = ActionClient(self, FollowJointTrajectory, agv_number + '/agv_controller/follow_joint_trajectory')

        self.kitting_station = 0.0
        self.front_station = 5.6
        self.back_station = 10.6
        self.speed = 2

        self.go_to_kitting_srv = self.create_service(
            MoveAGV,
            agv_number + '/go_to_station', 
            self.go_to_station_cb, 
            callback_group=self.callback_group)

    def send_goal(self, location):
        goal_msg = FollowJointTrajectory.Goal()
        
        point = JointTrajectoryPoint()
        
        if location == "kitting":
            distance = abs(self.agv_positions[0] - self.kitting_station)
            point.positions = [self.kitting_station]
        elif location == "front":
            distance = abs(self.agv_positions[0] - self.front_station)
            point.positions = [self.front_station]
        elif location == "back":
            distance = abs(self.agv_positions[0] - self.back_station)
            point.positions = [self.back_station]
        else:
            return None

        point.time_from_start = Duration(seconds=(distance/self.speed)).to_msg()
        
        goal_msg.trajectory.points = [point]
        goal_msg.trajectory.joint_names = ["agv_joint"]
        goal_msg.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        self._action_client.wait_for_server()

        self.action_done_event.clear()

        send_goal_future = self._action_client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(self.goal_response_callback)

        self.action_done_event.wait()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Signal that action is done
        self.action_result = future.result().result
        self.action_done_event.set()

    def go_to_station_cb(self, request, response:MoveAGV.Response):        
        self.send_goal(request.station)

        if self.action_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            response.success = True
        else:
            response.success = False
            response.message = "The action was not successful"

        return response

    def agv_joint_state_cb(self, msg):
        self.recieved_joint_states = True
        self.agv_joint_names = msg.name
        self.agv_positions = msg.position

def main(args=None):
    rclpy.init(args=args)

    action_client = AGV_Mover(sys.argv[1])

    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor)

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()