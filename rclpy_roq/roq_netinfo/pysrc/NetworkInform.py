#!/usr/bin/env /usr/bin/python3

import os
import time

import rclpy
from rclpy.node import Node

from roq_msgsrv.msg import NwProcMsg

import traceback

class NwtworkInform(Node):
	SELFNODE = 'netinfo'
	SELFTOPIC = 'nw_proc'

	def __init__(self):
		super().__init__(self.SELFNODE)
		self.get_logger().info("{} initializing...".format((self.SELFNODE)))
		self.pub = self.create_publisher(NwProcMsg, self.SELFTOPIC, 10)
		self.create_timer(1.00, self.pub_callback)
		self.get_logger().info("{} do...".format(self.SELFNODE))

	def __del__(self):
		self.get_logger().info("{} done.".format(self.SELFNODE))
	
	def read_status(self, filename, mode):
		fd = open(filename, mode)

	def pub_callback(self):
		start = time.time()
		
		## MemProcMsg setup


		end = time.time()
		self.get_logger().info('time: {:.4f}'.format(end - start))


def main(args = None):
	rclpy.init(args = args)
	node = NetworkInform()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		print('\nGot Ctrl+C.  System is stopped..')
	except Exception:
		print('\nException raised..  System will be shutdown..')
		traceback.print_exc()
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

"""
NwProcMsg:
	int8 is_valid
	int32 n_send
	int32 n_receive
"""
