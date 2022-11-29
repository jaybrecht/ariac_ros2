#!/usr/bin/env python3

import sys
import rclpy

from ariac_gazebo.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    # Read trial configuration
    startup_node.trial_config = startup_node.read_yaml(sys.argv[1])

    # Read user configuration
    startup_node.user_config = startup_node.read_yaml(sys.argv[2])

    # Pause physics
    # startup_node.pause_physics()

    # Spawn robots
    startup_node.spawn_robots()

    # Spawn sensors
    startup_node.spawn_sensors()

    # Spawn parts in bins
    startup_node.spawn_bin_parts() #TODO

    # Spawn kit trays
    startup_node.spawn_kit_trays()

    # Spawn trays and parts on AGVs
    startup_node.spawn_parts_on_agvs() #TODO

    # Unpause physics
    # startup_node.unpause_physics()

    startup_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()