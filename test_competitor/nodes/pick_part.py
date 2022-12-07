# usr/bin/python3

import rclpy
from test_competitor.floor_robot_commander import FloorRobotCommander

def main():
    rclpy.init()

    commander = FloorRobotCommander()

    commander.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()