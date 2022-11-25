#!/usr/bin/env python3

import sys
import rclpy

from ariac_gazebo.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    # Read trial configuration
    # config_path = sys.argv[1]
    # startup_node.read_config(config_path)

    # Pause physics
    startup_node.pause_physics()
    # rclpy.spin(startup_node)

    # Spawn sensors
    startup_node.spawn_sensors()

    # Spawn parts in bins
    startup_node.spawn_bin_parts()

    # Spawn kit trays
    startup_node.spawn_kit_trays()

    # Spawn robots
    startup_node.spawn_robots()

    # Spawn trays and parts on AGVs
    startup_node.spawn_parts_on_agvs()

    # Unpause physics
    startup_node.unpause_physics()

    startup_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()