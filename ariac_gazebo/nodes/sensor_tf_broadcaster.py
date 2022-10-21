#!/usr/bin/env python3

import os
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.tf_broadcaster import TFBroadcaster
from ariac_gazebo.spawn_params import GazeboSpawnParams

def main():
    rclpy.init()

    sensor_tf_broadcaster = TFBroadcaster()
    sensor_params = []

    config = os.path.join(get_package_share_directory('ariac_gazebo'), 'config', "sensors.yaml")

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    try:
        sensors = data['sensors']
    except KeyError:
        sensor_tf_broadcaster.get_logger().warn("No sensors found in config")

    for sensor_name in sensors:
        type = sensors[sensor_name]['type']
        xyz = sensors[sensor_name]['pose']['xyz']
        rpy = sensors[sensor_name]['pose']['rpy']
        if 'visualize_fov' in sensors[sensor_name].keys():
            vis = sensors[sensor_name]['visualize_fov']
        else:
            vis = True

        sensor_params.append(GazeboSpawnParams(sensor_name, type, xyz=xyz, rpy=rpy, visulize=vis))

    # Spawn the robots into gazebo
    for params in sensor_params:
        sensor_tf_broadcaster.generate_transform(params)

    # Send tf transforms
    sensor_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(sensor_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()