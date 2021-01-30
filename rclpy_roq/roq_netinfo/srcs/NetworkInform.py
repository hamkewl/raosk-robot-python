#!/usr/bin/env /usr/bin/python3

import os
import time

import rclpy
from rclpy.node import Node

from roq_msgsrv.msg import NwProcMsg

import traceback

class NetworkInform(Node):
	SELFNODE = 'netinfo'
	SELFTOPIC = 'nw_proc'
	SELFNIF = 'wlan0'
	sendCum_, receCum_ = 0, 0
	sendDiff_, receDiff_ = 0, 0

	def __init__(self):
		super().__init__(self.SELFNODE)
		self.get_logger().info("{} initializing...".format((self.SELFNODE)))
		self.pub = self.create_publisher(NwProcMsg, self.SELFTOPIC, 10)
		self.create_timer(1.00, self.pub_callback)
		self.get_logger().info("{} do...".format(self.SELFNODE))

	def __del__(self):
		self.get_logger().info("{} done.".format(self.SELFNODE))
	
	## acquire n_send, n_receive
	def read_netproc(self, ifname):
		filename = '/proc/net/dev'
		mode = 'r'
		fd = open(filename, mode)
		line = fd.read().replace(':', '')
		line_n = line.split('\n')

		dic_netproc = {}
		for line_ns in line_n:
			line_ns = line_ns.split()
			if len(line_ns) >= (1 + 9):
				try:
					dic_netproc[line_ns[0]] = ( int(line_ns[9]), int(line_ns[1]) )	# n_send, n_receive
				except ValueError:	# line_ns[1] と line_ns[9] が文字列の場合にcatch
					pass
		return dic_netproc[ifname]

	def pub_callback(self):
		start = time.time()
		
		## NwProcMsg setup
		t_netproc = self.read_netproc(self.SELFNIF)
		self.sendDiff_ = t_netproc[0] - self.sendCum_
		self.sendCum_ = t_netproc[0]
		self.receDiff_ = t_netproc[1] - self.receCum_
		self.receCum_ = t_netproc[1]
		
		msg = NwProcMsg()
		msg.is_valid = 0
		msg.n_send = self.sendDiff_ / (1e+3)	# Byte->KB
		msg.n_receive = self.receDiff_ / (1e+3)	# Byte->KB	
		self.pub.publish(msg)
		self.get_logger().info('n_send: {:6.4f}KB,  n_receive: {:6.4f}KB'.format(msg.n_send, msg.n_receive))

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
