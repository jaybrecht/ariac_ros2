#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose

from ariac_gazebo.utilities import convert_pi_string_to_float, quaternion_from_euler

class GazeboSpawnParams:
    def __init__(self, name, file_path=None, xyz=[0,0,0], rpy=[0,0,0], ns='', rf='', visulize=True):
        self.name = name
        self.robot_namespace = ns
        
        self.initial_pose = self.pose_info(xyz, rpy)
        
        if not file_path:
            self.topic_name = self.name + '/robot_description'
        else:    
            self.file_path = file_path

        self.reference_frame = rf
        self.visualize = visulize

    def pose_info(self, xyz: list, rpy: list) -> Pose:
        xyz_floats = []
        rpy_floats = []
        for s in xyz:
            try:
                xyz_floats.append(float(s))
            except ValueError:
                xyz_floats.append(convert_pi_string_to_float(s))
        for s in rpy:
            try:
                rpy_floats.append(float(s))
            except ValueError:
                rpy_floats.append(convert_pi_string_to_float(s))

        pose = Pose()
        pose.position.x = xyz_floats[0]
        pose.position.y = xyz_floats[1]
        pose.position.z = xyz_floats[2]
        q = quaternion_from_euler(*rpy_floats)
        pose.orientation.w = q[0]
        pose.orientation.x = q[1]
        pose.orientation.y = q[2]
        pose.orientation.z = q[3]

        return pose