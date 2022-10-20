#!/usr/bin/env python3

import os
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.spawn_params import GazeboSpawnParams
from ariac_gazebo.gazebo_sensor_spawner import GazeboSensorSpawner

def main():
    rclpy.init()

    sensor_spawner = GazeboSensorSpawner()
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
        sensor_spawner.get_logger().warn("No sensors found in config")

    for sensor_name in sensors:
        type = sensors[sensor_name]['type']
        file_path = os.path.join(get_package_share_directory('ariac_sensors'), 'models', type, 'model.sdf')
        
        xyz = sensors[sensor_name]['pose']['xyz']
        rpy = sensors[sensor_name]['pose']['rpy']
        
        if 'visualize_fov' in sensors[sensor_name].keys():
            vis = sensors[sensor_name]['visualize_fov']
        else:
            vis = True

        sensor_params.append(GazeboSpawnParams(sensor_name, file_path=file_path, xyz=xyz, rpy=rpy, visulize=vis))

    # Spawn the robots into gazebo
    for params in sensor_params:
        if not sensor_spawner.spawn_from_params(params):
            sensor_spawner.get_logger().error(f"Unable to spawn {params.name}")

    sensor_spawner.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()