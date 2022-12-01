#!/usr/bin/env python3

import sys
import rclpy

from ariac_gazebo.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup(sys.argv[1], sys.argv[2])

    # Spawn robots
    startup_node.spawn_robots()

    # Spawn sensors
    startup_node.spawn_sensors()

    # Spawn parts in bins
    startup_node.spawn_bin_parts()

    # Spawn kit trays
    startup_node.spawn_kit_trays()

    # Spawn trays and parts on AGVs
    startup_node.spawn_parts_on_agvs()

    # Read conveyor part config
    startup_node.parse_conveyor_config()

    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()