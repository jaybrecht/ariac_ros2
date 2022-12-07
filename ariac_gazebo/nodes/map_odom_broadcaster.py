#!/usr/bin/env python3

import rclpy
from ariac_gazebo.tf_broadcaster import TFBroadcaster
from geometry_msgs.msg import Pose


def main():
    rclpy.init()

    tf_broadcaster = TFBroadcaster("map_odom_broadcaster")
    initial_pose = Pose()
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.0
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0

    tf_broadcaster.generate_transform("map", "odom", initial_pose)
    tf_broadcaster.send_transforms()

    try:
        rclpy.spin(tf_broadcaster)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
