#!/usr/bin/env python3

from ariac_gazebo.utilities import pose_info

class GazeboSpawnParams:
    def __init__(self, name, model_type=None, file_path=None, xyz=[0,0,0], rpy=[0,0,0], ns='', rf='', visulize=True):
        self.name = name
        self.robot_namespace = ns
        
        self.initial_pose = pose_info(xyz, rpy)
        
        if not file_path:
            self.topic_name = self.name + '/robot_description'
        
        self.file_path = file_path

        self.reference_frame = rf

        # Sensor specific 
        self.visualize = visulize
        self.model_type = model_type