#!/usr/bin/env python3

import os
import yaml

import rclpy

from ament_index_python.packages import get_package_share_directory

from ariac_gazebo.tf_broadcaster import TFBroadcaster
from ariac_gazebo.utilities import pose_info

def main():
    rclpy.init()

    bins_tf_broadcaster = TFBroadcaster("bins_tf_broadcaster")

    config = os.path.join(get_package_share_directory('ariac_gazebo'), 'config', "bin_poses.yaml")

    with open(config, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    # Generate static transforms from config file
    try:
        bins = data['bins']
    except KeyError:
        bins_tf_broadcaster.get_logger().warn("No sensors found in config")

    for bin_name in bins:
        xyz = bins[bin_name]['pose']['xyz']
        rpy = bins[bin_name]['pose']['rpy']

        pose = pose_info(xyz, rpy)

        bins_tf_broadcaster.generate_transform("world", bin_name+"_frame", pose)

    # Send tf transforms
    bins_tf_broadcaster.send_transforms()

    try:
        rclpy.spin(bins_tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()