#!/usr/bin/env python3

import math
import yaml
import xml.etree.ElementTree as ET
from random import randint

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from ariac_gazebo.tf2_geometry_msgs import do_transform_pose
from ariac_gazebo.utilities import quaternion_from_euler, euler_from_quaternion, convert_pi_string_to_float

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Pose

from ariac_msgs.msg import (
    Part,
    PartLot,
    BinInfo,
    BinParts,
    ConveyorParts,
    ConveyorBeltState,
)

from gazebo_msgs.srv import SpawnEntity
from std_srvs.srv import Empty
from ariac_gazebo.spawn_params import (
    SpawnParams, 
    RobotSpawnParams, 
    SensorSpawnParams,
    PartSpawnParams,
    TraySpawnParams)

class PartInfo:
    part_heights = {
        'battery': 0.04,
        'sensor': 0.07,
        'pump': 0.12,
        'regulator': 0.07,
    }

    def __init__(self):
        self.type = None
        self.color = None
        self.rotation = '0'
        self.flipped = False
        self.height = None

class EnvironmentStartup(Node):
    def __init__(self, trial_config_path, user_config_path):
        super().__init__('environment_startup_node')

        self.declare_parameter('floor_robot_description', '', 
            ParameterDescriptor(description='Floor robot description'))
        self.declare_parameter('ceiling_robot_description', '', 
            ParameterDescriptor(description='Ceiling robot description'))
        self.declare_parameter('agv1_description', '', 
            ParameterDescriptor(description='AGV1 robot description'))
        self.declare_parameter('agv2_description', '', 
            ParameterDescriptor(description='AGV2 robot description'))
        self.declare_parameter('agv3_description', '', 
            ParameterDescriptor(description='AGV3 robot description'))
        self.declare_parameter('agv4_description', '', 
            ParameterDescriptor(description='AGV4 robot description'))
        
        self.declare_parameter('trial_config_path', '', 
            ParameterDescriptor(description='Path of the current trial\'s configuration yaml file'))
        self.declare_parameter('user_config_path', '', 
            ParameterDescriptor(description='Path of the user\'s configuration yaml file'))

        self.trial_config = self.read_yaml(
            self.get_parameter('trial_config_path').get_parameter_value().string_value)
        self.user_config = self.read_yaml(
            self.get_parameter('user_config_path').get_parameter_value().string_value)

        self.robot_names = [
            'floor_robot', 
            'ceiling_robot', 
            'agv1', 
            'agv2', 
            'agv3', 
            'agv4']

        # Conveyor 
        self.conveyor_spawn_rate = None
        self.conveyor_parts_to_spawn = []
        self.conveyor_transform = None
        self.conveyor_enabled = False
        self.conveyor_spawn_order_types = ['sequential', 'random']
        self.conveyor_width = 0.28

        self.conveyor_status_sub = self.create_subscription(ConveyorBeltState, 
            '/ariac/conveyor_state', self.conveyor_status, 10)

        # Create publishers for bin and conveyor parts
        self.bin_parts = BinParts()
        self.bin_parts_publisher = self.create_publisher(BinParts, 
            '/ariac/bin_parts', 10)

        self.conveyor_parts = ConveyorParts()
        self.conveyor_parts_publisher = self.create_publisher(ConveyorParts, 
            '/ariac/conveyor_parts', 10)

        self.bin_parts_pub_timer = self.create_timer(1.0, self.publish_bin_parts)
        self.conveyor_parts_pub_timer = self.create_timer(1.0, self.publish_conveyor_parts)
        
        # Read parameters for robot descriptions
        self.robot_descriptions = {}
        self.get_robot_descriptions_from_parameters()
        
        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        # Create pause and unpause clients
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.unpause_client = self.create_client(Empty, "/unpause_physics")

        # Setup TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spawn_sensors(self):
        try:
            user_sensors = self.user_config['sensors']
        except KeyError:
            self.get_logger().warn("No sensors found in config")
            user_sensors = []

        # Spawn user sensors
        for sensor_name in user_sensors:
            sensor_type = user_sensors[sensor_name]['type']
            xyz = user_sensors[sensor_name]['pose']['xyz']
            rpy = user_sensors[sensor_name]['pose']['rpy']
            
            if 'visualize_fov' in user_sensors[sensor_name].keys():
                vis = user_sensors[sensor_name]['visualize_fov']
            else:
                vis = False

            params = SensorSpawnParams(sensor_name, sensor_type, visualize=vis, xyz=xyz, rpy=rpy)
            self.spawn_entity(params)
        
        # Spawn quality control sensors
        for i in range(1, 5):
            sensor_name = "quality_control_sensor" + str(i)
            sensor_type = "quality_control"
            xyz = [0, 0, 1]
            vis = False

            params = SensorSpawnParams(sensor_name, sensor_type, visualize=vis, xyz=xyz)
            params.reference_frame = "agv" + str(i) + "_tray"
            self.spawn_entity(params)

    def spawn_kit_trays(self):
        possible_slots = [1, 2, 3, 4, 5, 6]
        possible_ids = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]

        slot_info = {
            1: {"table": "table_1", "offset": -0.43},
            2: {"table": "table_1", "offset": 0.0},
            3: {"table": "table_1", "offset": 0.43},
            4: {"table": "table_2", "offset": -0.43},
            5: {"table": "table_2", "offset": 0.0},
            6: {"table": "table_2", "offset": 0.43},
        }

        # Check that input is valid
        try:
            ids = self.trial_config["kitting_trays"]["tray_ids"]
            slots = self.trial_config["kitting_trays"]["slots"]
        except KeyError:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if len(ids) == 0 or len(slots) == 0:
            self.get_logger().warn("No kitting trays found in configuration")
            return

        if not (len(ids) == len(slots)):
            self.get_logger().warn("Number of trays does not equal number of slots")
            return

        for id, slot in zip(ids, slots):
            if not type(id) == int or not type(slot) == int:
                self.get_logger().warn("Tray ids and slots must be integers")
                return
            elif id not in possible_ids:
                self.get_logger().warn("Tray id must be between 0 and 9")
                return
            elif slot not in possible_slots:
                self.get_logger().warn("Tray slot must be between 1 and 6")
                return

        # Calculate location of tables using tf
        transforms = {}
        try:
            transforms['table_1'] = self.tf_buffer.lookup_transform('world', "kts1_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform kts1_table_frame to world: {ex}')
            return

        try:
            transforms['table_2'] = self.tf_buffer.lookup_transform('world', "kts2_table_frame", rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform kts1_table_frame to world: {ex}')
            return

        # Spawn trays 
        num_trays = 0
        kit_tray_thickness = 0.01
        for id, slot in zip(ids, slots):
            rel_pose = Pose()
            rel_pose.position.x = slot_info[slot]["offset"]
            rel_pose.position.z = kit_tray_thickness

            world_pose = do_transform_pose(rel_pose, transforms[slot_info[slot]["table"]])
            
            # Create unique name for each tray that includes the id
            marker_id = str(id).zfill(2)
            name = "kit_tray_" +  marker_id + "_" + str(num_trays)
            num_trays += 1

            xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
            rpy = euler_from_quaternion(world_pose.orientation)

            params = TraySpawnParams(name, marker_id, xyz=xyz, rpy=rpy)

            self.spawn_entity(params, wait=False)

    def spawn_bin_parts(self):
        possible_bins = ['bin1', 'bin2', 'bin3', 'bin4', 'bin5', 'bin6', 'bin7', 'bin8']

        slot_info = {
            1: {"x_offset": 0.18, "y_offset": 0.18},
            2: {"x_offset": 0.18, "y_offset": 0.0},
            3: {"x_offset": 0.18, "y_offset": -0.18},
            4: {"x_offset": 0.0, "y_offset": 0.18},
            5: {"x_offset": 0.0, "y_offset": 0.0},
            6: {"x_offset": 0.0, "y_offset": -0.18},
            7: {"x_offset": -0.18, "y_offset": 0.18},
            8: {"x_offset": -0.18, "y_offset": 0.0},
            9: {"x_offset": -0.18, "y_offset": -0.18},
        }

        # Validate input
        try:
            bin_parts_config = self.trial_config["parts"]["bins"]
        except KeyError:
            self.get_logger().warn("No bin parts found in configuration")
            return
        
        if not bin_parts_config:
            return

        part_count = 0
        for bin_name in bin_parts_config.keys():
            if not bin_name in possible_bins:
                self.get_logger().warn(f"{bin_name} is not a valid bin name")
                continue
        
            try:
                bin_transform = self.tf_buffer.lookup_transform('world', bin_name + "_frame", rclpy.time.Time())
            except TransformException as ex:
                self.get_logger().info(f'Could not transform {bin_name}_frame to world: {ex}')
                return

            # Fill bin info msg for each bin that has parts
            bin_info = BinInfo()
            bin_info.bin_number = int(bin_name[-1])

            available_slots = list(range(1,10))
            for part_info in bin_parts_config[bin_name]:
                ret, part = self.parse_part_info(part_info)
                if not ret:
                    continue
                
                try:
                    slots = part_info['slots']
                    if not type(slots) == list:
                        self.get_logger().warn("slots parameter should be a list of integers")
                        continue
                except KeyError:
                    self.get_logger().warn("Part slots are not specified")
                    continue
                
                # Spawn parts into slots
                num_parts_in_bin = 0
                for slot in slots:
                    if not slot in slot_info.keys():
                        self.get_logger().warn(f"Slot {slot} is not a valid option")
                        continue
                    elif not slot in available_slots:
                        self.get_logger().warn(f"Slot {slot} is already occupied")
                        continue

                    num_parts_in_bin += 1
                    
                    available_slots.remove(slot)
                    
                    part_name = part.type + "_" + part.color + "_b" + str(part_count).zfill(2)
                    part_count += 1

                    if part.flipped:
                        roll = math.pi
                    else:
                        roll = 0

                    yaw = convert_pi_string_to_float(part.rotation)
                    
                    q = quaternion_from_euler(roll, 0, yaw)
                    rel_pose = Pose()
                    rel_pose.position.x = slot_info[slot]["x_offset"]
                    rel_pose.position.y = slot_info[slot]["y_offset"]

                    if part.flipped:
                        rel_pose.position.z = part.height

                    rel_pose.orientation.w = q[0]
                    rel_pose.orientation.x = q[1]
                    rel_pose.orientation.y = q[2]
                    rel_pose.orientation.z = q[3]

                    world_pose = do_transform_pose(rel_pose, bin_transform)

                    xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
                    rpy = euler_from_quaternion(world_pose.orientation)

                    params = PartSpawnParams(part_name, part.type, part.color, xyz=xyz, rpy=rpy)

                    self.spawn_entity(params, wait=False)
            
                bin_info.parts.append(self.fill_part_lot_msg(part, num_parts_in_bin))
            
            self.bin_parts.bins.append(bin_info)

    def fill_part_lot_msg(self, part: PartInfo, quantity: int):
        part_colors = {
            'red': Part.RED,
            'green': Part.GREEN,
            'blue': Part.BLUE,
            'orange': Part.ORANGE,
            'purple': Part.PURPLE,
        }
        
        part_types = {
            'battery': Part.BATTERY,
            'pump': Part.PUMP,
            'sensor': Part.SENSOR,
            'regulator': Part.REGULATOR,
        }

        lot = PartLot()
        lot.part.type = part_types[part.type]
        lot.part.color = part_colors[part.color]
        lot.quantity = quantity

        return lot
        
    def spawn_conveyor_part(self):
        if self.conveyor_enabled:
            if self.conveyor_spawn_order == 'sequential':
                part_params = self.conveyor_parts_to_spawn.pop(0)
            elif self.conveyor_spawn_order == 'random':
                part_params = self.conveyor_parts_to_spawn.pop(randint(0, len(self.conveyor_parts_to_spawn) - 1))
            
            self.spawn_entity(part_params, wait=False)

        if not self.conveyor_parts_to_spawn:
            self.conveyor_spawn_timer.cancel()

    def spawn_robots(self):
        for name in self.robot_names:
            urdf = ET.fromstring(self.robot_descriptions[name])
            params = RobotSpawnParams(name, ET.tostring(urdf, encoding="unicode"))
            self.spawn_entity(params)

    def spawn_parts_on_agvs(self):
        quadrant_info = {
            1: {"x_offset": -0.0925, "y_offset": 0.1275},
            2: {"x_offset": 0.0925, "y_offset": 0.1275},
            3: {"x_offset": -0.0925, "y_offset": -0.1275},
            4: {"x_offset": 0.0925, "y_offset": -0.1275},
        }

        possible_agvs = ['agv1', 'agv2', 'agv3', 'agv4']

        # Validate input
        try:
            agv_parts = self.trial_config["parts"]["agvs"]
        except KeyError:
            self.get_logger().log("No agv parts found in configuration")
            return
        
        part_count = 0
        for agv in agv_parts:
            if not agv in possible_agvs:
                self.get_logger().warn(f"{agv} is not a valid agv name")
                continue

            # Spawn a kit tray onto the AGV
            try:
                tray_id = agv_parts[agv]['tray_id']
            except KeyError:
                tray_id = 0
            
            marker_id = str(tray_id).zfill(2)
            name = "kit_tray_" +  marker_id + "_a" + agv[-1]

            xyz = [0, 0, 0.01]
            reference_frame = agv + "_tray"
            params = TraySpawnParams(name, marker_id, xyz=xyz, rf=reference_frame)
            self.spawn_entity(params)

            # Spawn parts onto kit tray
            available_quadrants = list(range(1,5))
            for part_info in agv_parts[agv]['parts']:
                ret, part = self.parse_part_info(part_info)
                if not ret:
                    continue
                
                try:
                    quadrant = part_info['quadrant']
                    if not quadrant in quadrant_info.keys():
                        self.get_logger().warn(f"Quadrant {quadrant} is not an option")
                        continue
                except KeyError:
                    self.get_logger().warn("Quadrant is not specified")
                    continue

                available_quadrants.remove(quadrant)

                if part.flipped:
                    roll = math.pi
                    z_height = part.height + 0.01
                else:
                    roll = 0
                    z_height = 0.01
                
                part_name = part.type + "_" + part.color + "_a" + str(part_count).zfill(2)
                part_count += 1

                xyz = [quadrant_info[quadrant]["x_offset"], quadrant_info[quadrant]["y_offset"], z_height]
                rpy = [roll, 0, part.rotation]

                params = PartSpawnParams(part_name, part.type, part.color, xyz=xyz, rpy=rpy, rf=reference_frame)

                self.spawn_entity(params, wait=False)
            
    def parse_conveyor_config(self):
        # Parse Conveyor Configuration
        try: 
            conveyor_config = self.trial_config['parts']['conveyor_belt']
        except KeyError:
            self.get_logger().error("Unable to find conveyor belt params in configuration file")
            return False

        try:
            active = conveyor_config['active']
            if not active:
                return False
        except KeyError:
            self.get_logger().error("Active paramater not set in conveyor belt configuration")
            return False

        try:
            spawn_rate = conveyor_config['spawn_rate']
        except KeyError:
            self.get_logger().error("Spawn rate paramater not set in conveyor belt configuration")
            return False
        
        try:
            self.spawn_rate = float(spawn_rate)
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        # Get conveyor transform
        try:
            conveyor_transform = self.tf_buffer.lookup_transform('world', 
                "conveyor_belt_part_spawn_frame", rclpy.time.Time())
        except TransformException:
            return

        try:
            if conveyor_config['order'] in self.conveyor_spawn_order_types:
                self.conveyor_spawn_order = conveyor_config['order']
            else:
                self.get_logger().error(f"Order paramater must be of type: {self.conveyor_spawn_order_types}")
                return False
        except ValueError:
            self.get_logger().error("Spawn rate paramater must be a number")
            return False

        try:
            parts = conveyor_config['parts_to_spawn']
        except KeyError:
            self.get_logger().error("Parts to spawn not found in configuration")
            return False

        part_count = 0
        for part_info in parts:
            ret, part = self.parse_part_info(part_info)

            if not ret:
                continue

            try:
                amount = part_info['number']
            except KeyError:
                continue

            try:
                offset = part_info['offset']
                if not -1 <= offset <= 1:
                    self.get_logger().error("Offset must be in range [-1, 1]")
                    offset = 0
            except KeyError:
                offset = 0

            self.conveyor_parts.parts.append(self.fill_part_lot_msg(part, amount))

            for i in range(amount):
                part_name = part.type + "_" + part.color + "_c" + str(part_count).zfill(2)
                part_count += 1

                if part.flipped:
                    roll = math.pi
                else:
                    roll = 0

                yaw = convert_pi_string_to_float(part.rotation)
                q = quaternion_from_euler(roll, 0, yaw)
                rel_pose = Pose()

                rel_pose.position.y = offset * (self.conveyor_width/2)

                if part.flipped:
                    rel_pose.position.z = part.height

                rel_pose.orientation.w = q[0]
                rel_pose.orientation.x = q[1]
                rel_pose.orientation.y = q[2]
                rel_pose.orientation.z = q[3]
            
                world_pose = do_transform_pose(rel_pose, conveyor_transform)

                xyz = [world_pose.position.x, world_pose.position.y, world_pose.position.z]
                rpy = euler_from_quaternion(world_pose.orientation)

                self.conveyor_parts_to_spawn.append(PartSpawnParams(part_name, part.type, part.color, xyz=xyz, rpy=rpy))

        if len(self.conveyor_parts_to_spawn) > 0:
            # Create Spawn Timer
            self.conveyor_spawn_timer = self.create_timer(self.spawn_rate, self.spawn_conveyor_part)
            return True
        
        return False

    def conveyor_status(self, msg: ConveyorBeltState):        
        self.conveyor_enabled = msg.enabled

    def get_robot_descriptions_from_parameters(self):
        for name in self.robot_names:
            self.robot_descriptions[name] = self.get_parameter(name + '_description').value

    def spawn_entity(self, params: SpawnParams, wait=True) -> bool:
        self.spawn_client.wait_for_service()

        # self.get_logger().info(f'Spawning: {params.name}')

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        future = self.spawn_client.call_async(req)

        if wait: 
            rclpy.spin_until_future_complete(self, future)
            return future.result().success
        else:
            return True

    def parse_part_info(self, part_info):
        part = PartInfo()
        
        try:
            part.type = part_info['type']
            part.height = PartInfo.part_heights[part.type]
        except KeyError:
            self.get_logger().warn("Part type is not specified")
            return (False, part)

        try:
            part.color = part_info['color']
        except KeyError:
            self.get_logger().warn("Part color is not specified")
            return (False, part)

        try:
            part.rotation = str(part_info['rotation'])
        except KeyError:
            pass

        try:
            part.flipped = part_info['flipped']
            if not type(part.flipped) == bool:
                self.get_logger().warn("flipped parameter should be either true or false")
                part.flipped = False
        except KeyError:
            pass

        if not part.type in PartSpawnParams.part_types:
            self.get_logger().warn(f"{part_info['type']} is not a valid part type")
            return (False, part)
        
        if  not part.color in PartSpawnParams.colors:
            self.get_logger().warn(f"{part_info['color']} is not a valid part color")
            return (False, part)
        
        return (True, part)

    def read_yaml(self, path):
        with open(path, "r") as stream:
            try:
                return yaml.safe_load(stream)
            except yaml.YAMLError:
                self.get_logger().error("Unable to read configuration file")
                return {}

    def pause_physics(self):
        self.pause_client.wait_for_service()

        request = Empty.Request()

        self.pause_client.call_async(request)
    
    def unpause_physics(self):
        self.unpause_client.wait_for_service()

        request = Empty.Request()

        self.unpause_client.call_async(request)
    
    def publish_bin_parts(self):
        self.bin_parts_publisher.publish(self.bin_parts)
    
    def publish_conveyor_parts(self):
        self.conveyor_parts_publisher.publish(self.conveyor_parts)