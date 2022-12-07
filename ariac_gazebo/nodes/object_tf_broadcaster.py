#!/usr/bin/env python3

import os
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.tf_broadcaster import TFBroadcaster
from ariac_gazebo.utilities import pose_info

def main():
    rclpy.init()

    objects_tf_broadcaster = TFBroadcaster("objects_tf_broadcaster")

    config = os.path.join(get_package_share_directory('ariac_gazebo'), 'config', "object_poses.yaml")

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    # Generate static transforms for bins
    try:
        bins = data['bins']
    except KeyError:
        objects_tf_broadcaster.get_logger().warn("No bins found in config")

    for bin_name in bins:
        xyz = bins[bin_name]['pose']['xyz']
        rpy = bins[bin_name]['pose']['rpy']

        pose = pose_info(xyz, rpy)

        objects_tf_broadcaster.generate_transform("world", bin_name+"_frame", pose)

    # Generate static transforms for assembly_stations
    try:
        stations = data['assembly_stations']
    except KeyError:
        objects_tf_broadcaster.get_logger().warn("No assembly_stations found in config")

    for station_name in stations:
        table_pose = pose_info(
            stations[station_name]["table"]['pose']['xyz'], 
            stations[station_name]["table"]['pose']['rpy'])

        insert_pose = pose_info(
            stations[station_name]["insert"]['pose']['xyz'], 
            stations[station_name]["insert"]['pose']['rpy'])

        table_frame_name = station_name + "_table_frame"
        insert_frame_name = station_name + "_insert_frame"
        objects_tf_broadcaster.generate_transform("world", table_frame_name, table_pose)
        objects_tf_broadcaster.generate_transform(table_frame_name, insert_frame_name, insert_pose)

    # Generate static transforms for assembly_stations
    try:
        stations = data['kit_tray_stations']
    except KeyError:
        objects_tf_broadcaster.get_logger().warn("No kit_tray_stations found in config")

    for station_name in stations:
        table_pose = pose_info(
            stations[station_name]["table"]['pose']['xyz'], 
            stations[station_name]["table"]['pose']['rpy'])

        tool_changer_parts_pose = pose_info(
            stations[station_name]["tool_changer_parts"]['pose']['xyz'], 
            stations[station_name]["tool_changer_parts"]['pose']['rpy'])

        tool_changer_trays_pose = pose_info(
            stations[station_name]["tool_changer_trays"]['pose']['xyz'], 
            stations[station_name]["tool_changer_trays"]['pose']['rpy'])

        table_frame_name = station_name + "_table_frame"
        tool_changer_parts_frame_name = station_name + "_tool_changer_parts_frame"
        tool_changer_trays_frame_name = station_name + "_tool_changer_trays_frame"

        objects_tf_broadcaster.generate_transform("world", table_frame_name, table_pose)
        objects_tf_broadcaster.generate_transform(
            table_frame_name, 
            tool_changer_parts_frame_name, 
            tool_changer_parts_pose)
        objects_tf_broadcaster.generate_transform(
            table_frame_name, 
            tool_changer_trays_frame_name, 
            tool_changer_trays_pose)

    # Send tf transforms
    objects_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(objects_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()