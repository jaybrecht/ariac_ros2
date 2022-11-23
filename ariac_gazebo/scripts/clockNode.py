#!/usr/bin/env python
import rclpy
import sys
from rclpy.node import Node
#from nist_gear.msg import *
from std_msgs.msg import Int64
import threading

class clock_node(Node):
	def __init__(self):
		global pub
		self.name='clock_node'
		super().__init__(self.name)
		pub = self.create_publisher(topic='stream_clock', msg_type=Int64, qos_profile = 1000)

def main(argv):
	rclpy.init(args=None)
	clockn = clock_node()
	thread = threading.Thread(target=rclpy.spin, args=(clockn, ), daemon=True)
	thread.start()
	rate = clockn.create_rate(3)
	count = Int64()
	count.data = 0
	while rclpy.ok():
#		print('Time:', count.data)
		pub.publish(count)
		count.data += 1
		rate.sleep()
	rclpy.shutdown()

if __name__ == '__main__':
	main(sys.argv)
