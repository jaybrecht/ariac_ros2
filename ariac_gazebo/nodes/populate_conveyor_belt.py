#!/usr/bin/env python3

import sys
import rclpy
from ariac_gazebo.conveyor_belt_populator import ConveyorBeltPopulator

if __name__ == "__main__":
    rclpy.init()

    populator = ConveyorBeltPopulator()

    # Wait for conveyor transform to be published to tf
    while True:
        try:
            rclpy.spin_once(populator)
        except KeyboardInterrupt:
            populator.destroy_node()
            rclpy.shutdown()
            exit()

        if populator.conveyor_transform is not None:
            populator.transform_timer.cancel()
            break
            
    if populator.read_config(sys.argv[1]):
        while len(populator.parts_to_spawn) > 0:
            try:
                rclpy.spin_once(populator)
            except KeyboardInterrupt:
                break

    populator.destroy_node()
    rclpy.shutdown()

