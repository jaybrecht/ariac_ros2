#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sortedcontainers import SortedDict
from threading import *
#from nist_gear.msg import *
import numpy as np
from gazebo_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from ariac_msgs.msg import *

ws_lock = Lock()
dict_msgs_1 = SortedDict()
dict_msgs_2 = SortedDict()
dict_msgs_3 = SortedDict()
dict_msgs_4 = SortedDict()
dict_msgs_5 = SortedDict()

def callback(data, args):
    ws_lock.acquire()
    if data.time not in dict_msgs_1:
        dict_msgs_1[data.time] = set()
    if data.time not in dict_msgs_2:
        dict_msgs_2[data.time] = set()
    if data.time not in dict_msgs_3:
        dict_msgs_3[data.time] = set()
    if data.time not in dict_msgs_4:
        dict_msgs_4[data.time] = set()
    if data.time not in dict_msgs_5:
        dict_msgs_5[data.time] = set()
    if args in ['hs1','hp1','rs1','rp1']:
        if args in ['hp1','rp1']:
            dict_msgs_1[data.time].add((args, (data.value.position.x,data.value.position.y,data.value.position.z)))
        else:
            dict_msgs_1[data.time].add((args, data.value))
        conditional_publish1()
#    if args in ['hs2','hp2','rs1','rp1']:
#        dict_msgs_2[data.time].add((args, data.value))
#        conditional_publish2()
#    if args in ['hs1','hp1','rs2','rp2']:
#        dict_msgs_3[data.time].add((args, data.value))
#        conditional_publish3()
#    if args in ['hs2','hp2','rs2','rp2']:
#        dict_msgs_4[data.time].add((args, data.value))
#        conditional_publish4()
#    if args in ['rs1','rp1','rs2','rp2']:
#        dict_msgs_5[data.time].add((args, data.value))
#        conditional_publish5()
    ws_lock.release()

attempts1 = 0
last_human_pose1 = None
last_robot_pose1 = None
last_time1 = None
def conditional_publish1():
    global attempts1, last_human_pose1, last_robot_pose1, last_time1
    if len(dict_msgs_1.peekitem(0)[1]) == 4:
        snapshot = Snapshot()
        snapshot.time = dict_msgs_1.peekitem(0)[0]
        for i in range(4):
            e = dict_msgs_1.peekitem(0)[1].pop()
            if e[0] == 'hs1':
                snapshot.human_operator_speed = e[1]
            elif e[0] == 'hp1':
                human_pose = e[1]
            elif e[0] == 'rs1':
                snapshot.robot_speed = e[1]
            else:
                robot_pose = e[1]
        if last_human_pose1 is not None and last_robot_pose1 is not None and last_time1 is not None:
            p1 = np.array([human_pose[0], human_pose[1], human_pose[2]])
            p2 = np.array([robot_pose[0], robot_pose[1], robot_pose[2]])
            p3 = np.array([last_human_pose1[0], last_human_pose1[1], last_human_pose1[2]])
            p4 = np.array([last_robot_pose1[0], last_robot_pose1[1], last_robot_pose1[2]])

            squared_dist = np.sum((p1-p2)**2, axis=0)
            snapshot.distance_robot_human_operator = np.sqrt(squared_dist)
            squared_dist = np.sum((p3-p2)**2, axis=0)
            last_distance_human_operator = np.sqrt(squared_dist)
            #print("last_distance_human_operator: " + str(last_distance_human_operator))
            squared_dist = np.sum((p4-p1)**2, axis=0)
            last_distance_robot = np.sqrt(squared_dist)
            #print("last_distance_robot: " + str(last_distance_robot))

            snapshot.robot_speed = (last_distance_robot - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time1) * 0.33) # derived by clock rate 3
            snapshot.human_operator_speed = (last_distance_human_operator - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time1) * 0.33) # derived by clock rate 3
            #print(snapshot)
            pub1.publish(snapshot)
            dict_msgs_1.popitem(0)
        last_human_pose1 = human_pose
        last_robot_pose1 = robot_pose
        last_time1 = snapshot.time
        attempts1 = 0
    elif attempts1 > 4:
        attempts1 = 0
        dict_msgs_1.popitem(0)
    else:
        attempts1 += 1

attempts2 = 0
last_human_pose2 = None
last_robot_pose2 = None
last_time2 = None
def conditional_publish2():
    global attempts2, last_human_pose2, last_robot_pose2, last_time2
    if len(dict_msgs_2.peekitem(0)[1]) == 4:
        snapshot = Snapshot()
        snapshot.time = dict_msgs_2.peekitem(0)[0]
        for i in range(4):
            e = dict_msgs_2.peekitem(0)[1].pop()
            if e[0] == 'hs2':
                snapshot.human_operator_speed = e[1]
            elif e[0] == 'hp2':
                human_pose = e[1]
            elif e[0] == 'rs1':
                snapshot.robot_speed = e[1]
            else:
                robot_pose = e[1]
        if last_human_pose2 is not None and last_robot_pose2 is not None and last_time2 is not None:
            p1 = np.array([human_pose.position.x, human_pose.position.y, human_pose.position.z])
            p2 = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
            p3 = np.array([last_human_pose2.position.x, last_human_pose2.position.y, last_human_pose2.position.z])
            p4 = np.array([last_robot_pose2.position.x, last_robot_pose2.position.y, last_robot_pose2.position.z])

            squared_dist = np.sum((p1-p2)**2, axis=0)
            snapshot.distance_robot_human_operator = np.sqrt(squared_dist)
            squared_dist = np.sum((p3-p2)**2, axis=0)
            last_distance_human_operator = np.sqrt(squared_dist)
            #print("last_distance_human_operator: " + str(last_distance_human_operator))
            squared_dist = np.sum((p4-p1)**2, axis=0)
            last_distance_robot = np.sqrt(squared_dist)
            #print("last_distance_robot: " + str(last_distance_robot))

            snapshot.robot_speed = (last_distance_robot - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time2) * 0.33) # derived by clock rate 3
            snapshot.human_operator_speed = (last_distance_human_operator - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time2) * 0.33) # derived by clock rate 3
            #print(snapshot)
            pub2.publish(snapshot)
            dict_msgs_2.popitem(0)
        last_human_pose2 = human_pose
        last_robot_pose2 = robot_pose
        last_time2 = snapshot.time
        attempts2 = 0
    elif attempts2 > 4:
        attempts2 = 0
        dict_msgs_2.popitem(0)
    else:
        attempts2 += 1

attempts3 = 0
last_human_pose3 = None
last_robot_pose3 = None
last_time3 = None
def conditional_publish3():
    global attempts3, last_human_pose3, last_robot_pose3, last_time3
    if len(dict_msgs_3.peekitem(0)[1]) == 4:
        snapshot = Snapshot()
        snapshot.time = dict_msgs_3.peekitem(0)[0]
        for i in range(4):
            e = dict_msgs_3.peekitem(0)[1].pop()
            if e[0] == 'hs1':
                snapshot.human_operator_speed = e[1]
            elif e[0] == 'hp1':
                human_pose = e[1]
            elif e[0] == 'rs2':
                snapshot.robot_speed = e[1]
            else:
                robot_pose = e[1]
        if last_human_pose3 is not None and last_robot_pose3 is not None and last_time3 is not None:
            p1 = np.array([human_pose.position.x, human_pose.position.y, human_pose.position.z])
            p2 = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
            p3 = np.array([last_human_pose3.position.x, last_human_pose3.position.y, last_human_pose3.position.z])
            p4 = np.array([last_robot_pose3.position.x, last_robot_pose3.position.y, last_robot_pose3.position.z])

            squared_dist = np.sum((p1-p2)**2, axis=0)
            snapshot.distance_robot_human_operator = np.sqrt(squared_dist)
            squared_dist = np.sum((p3-p2)**2, axis=0)
            last_distance_human_operator = np.sqrt(squared_dist)
            #print("last_distance_human_operator: " + str(last_distance_human_operator))
            squared_dist = np.sum((p4-p1)**2, axis=0)
            last_distance_robot = np.sqrt(squared_dist)
            #print("last_distance_robot: " + str(last_distance_robot))

            snapshot.robot_speed = (last_distance_robot - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time3) * 0.33) # derived by clock rate 3
            snapshot.human_operator_speed = (last_distance_human_operator - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time3) * 0.33) # derived by clock rate 3
            #print(snapshot)
            pub3.publish(snapshot)
            dict_msgs_3.popitem(0)
        last_human_pose3 = human_pose
        last_robot_pose3 = robot_pose
        last_time3 = snapshot.time
        attempts3 = 0
    elif attempts3 > 4:
        attempts3 = 0
        dict_msgs_3.popitem(0)
    else:
        attempts3 += 1

attempts4 = 0
last_human_pose4 = None
last_robot_pose4 = None
last_time4 = None
def conditional_publish4():
    global attempts4, last_human_pose4, last_robot_pose4, last_time4
    if len(dict_msgs_4.peekitem(0)[1]) == 4:
        snapshot = Snapshot()
        snapshot.time = dict_msgs_4.peekitem(0)[0]
        for i in range(4):
            e = dict_msgs_4.peekitem(0)[1].pop()
            if e[0] == 'hs2':
                snapshot.human_operator_speed = e[1]
            elif e[0] == 'hp2':
                human_pose = e[1]
            elif e[0] == 'rs2':
                snapshot.robot_speed = e[1]
            else:
                robot_pose = e[1]
        if last_human_pose4 is not None and last_robot_pose4 is not None and last_time4 is not None:
            p1 = np.array([human_pose.position.x, human_pose.position.y, human_pose.position.z])
            p2 = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z])
            p3 = np.array([last_human_pose4.position.x, last_human_pose4.position.y, last_human_pose4.position.z])
            p4 = np.array([last_robot_pose4.position.x, last_robot_pose4.position.y, last_robot_pose4.position.z])

            squared_dist = np.sum((p1-p2)**2, axis=0)
            snapshot.distance_robot_human_operator = np.sqrt(squared_dist)
            squared_dist = np.sum((p3-p2)**2, axis=0)
            last_distance_human_operator = np.sqrt(squared_dist)
            #print("last_distance_human_operator: " + str(last_distance_human_operator))
            squared_dist = np.sum((p4-p1)**2, axis=0)
            last_distance_robot = np.sqrt(squared_dist)
            #print("last_distance_robot: " + str(last_distance_robot))

            snapshot.robot_speed = (last_distance_robot - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time4) * 0.33) # derived by clock rate 3
            snapshot.human_operator_speed = (last_distance_human_operator - snapshot.distance_robot_human_operator) / ((snapshot.time - last_time4) * 0.33) # derived by clock rate 3
            #print(snapshot)
            pub4.publish(snapshot)
            dict_msgs_4.popitem(0)
        last_human_pose4 = human_pose
        last_robot_pose4 = robot_pose
        last_time4 = snapshot.time
        attempts4 = 0
    elif attempts4 > 4:
        attempts4 = 0
        dict_msgs_4.popitem(0)
    else:
        attempts4 += 1

attempts5 = 0
last_robot1_pose5 = None
last_robot2_pose5 = None
last_time5 = None
def conditional_publish5():
    global attempts5, last_robot1_pose5, last_robot2_pose5, last_time5
    if len(dict_msgs_5.peekitem(0)[1]) == 4:
        snapshot = Snapshot2()
        snapshot.time = dict_msgs_5.peekitem(0)[0]
        for i in range(4):
            e = dict_msgs_5.peekitem(0)[1].pop()
            if e[0] == 'rs1':
                snapshot.robot_speed1 = e[1]
            elif e[0] == 'rp1':
                robot_pose1 = e[1]
            elif e[0] == 'rs2':
                snapshot.robot_speed2 = e[1]
            else:
                robot_pose2 = e[1]
        if last_robot1_pose5 is not None and last_robot2_pose5 is not None and last_time5 is not None:
            p1 = np.array([robot_pose1.position.x, robot_pose1.position.y, robot_pose1.position.z])
            p2 = np.array([robot_pose2.position.x, robot_pose2.position.y, robot_pose2.position.z])
            p3 = np.array([last_robot1_pose5.position.x, last_robot1_pose5.position.y, last_robot1_pose5.position.z])
            p4 = np.array([last_robot2_pose5.position.x, last_robot2_pose5.position.y, last_robot2_pose5.position.z])

            squared_dist = np.sum((p1-p2)**2, axis=0)
            snapshot.distance_robot_robot = np.sqrt(squared_dist)
            squared_dist = np.sum((p3-p2)**2, axis=0)
            last_distance_robot1 = np.sqrt(squared_dist)
            #print("last_distance_human_operator: " + str(last_distance_human_operator))
            squared_dist = np.sum((p4-p1)**2, axis=0)
            last_distance_robot2 = np.sqrt(squared_dist)
            #print("last_distance_robot: " + str(last_distance_robot))

            snapshot.robot_speed1 = (last_distance_robot1 - snapshot.distance_robot_robot) / ((snapshot.time - last_time5) * 0.33) # derived by clock rate 3
            snapshot.robot_speed2 = (last_distance_robot2 - snapshot.distance_robot_robot) / ((snapshot.time - last_time5) * 0.33) # derived by clock rate 3
            #print(snapshot)
            pub5.publish(snapshot)
            dict_msgs_5.popitem(0)
        last_robot1_pose5 = robot_pose1
        last_robot2_pose5 = robot_pose2
        last_time5 = snapshot.time
        attempts5 = 0
    elif attempts5 > 4:
        attempts5 = 0
        dict_msgs_5.popitem(0)
    else:
        attempts5 += 1

class SnapshotNode(Node):
    def __init__(self):
        global pub1, pub2, pub3, pub4, pub5
        self.name='snapshot_node'
        super().__init__(self.name)
        pub1 = self.create_publisher(topic='/snapshot', msg_type=Snapshot, qos_profile = 1000)
        self.create_subscription(topic='/monitor/robot/speed', msg_type=TimedFloat, callback=(lambda msg: callback(msg, 'rs1')), qos_profile = 1000)
        self.create_subscription(topic='/monitor/robot/pose', msg_type=TimedPose, callback=(lambda msg: callback(msg, 'rp1')), qos_profile = 1000)
        self.create_subscription(topic='/monitor/human/speed', msg_type=TimedFloat, callback=(lambda msg: callback(msg, 'hs1')), qos_profile = 1000)
        self.create_subscription(topic='/monitor/human/pose', msg_type=TimedPose, callback=(lambda msg: callback(msg, 'hp1')), qos_profile = 1000)

if __name__ == '__main__':
    global node
    rclpy.init(args=None)
    node = SnapshotNode()
    rclpy.spin(node)
    rclpy.shutdown()
