#!/usr/bin/env /usr/bin/python3

import os
import time
import numpy as np
import psutil
import subprocess
import rclpy

from . import smem
from subprocess import PIPE
from rclpy.node import Node
from roq_msgsrv.msg import MemProcMsg

import traceback

class MemoryInform(Node):
	SELFPID = os.getpid()
	SELFNODE = 'meminfo_{}'.format(SELFPID)
	SELFTOPIC = 'mem_proc'

	## to evaluation
	exec_time = []

	def __init__(self):
		super().__init__(self.SELFNODE)
		self.get_logger().info("{} initializing...".format((self.SELFNODE)))
		self.pub = self.create_publisher(MemProcMsg, self.SELFTOPIC, 10)
		self.create_timer(1.00, self.pub_callback)

		# execute subprocess
		slam_bringup = subprocess.Popen("ros2 launch turtlebot3_bringup robot.launch.py", \
			shell = True, stdout = PIPE, stderr = PIPE, text = True)
		self.TOPPARENT = slam_bringup.pid
		self.get_logger().info("{} do...  @{}".format(self.SELFNODE, self.TOPPARENT))

	def __del__(self):
		"""
			msg = MemProcMsg()
			msg.vgid = self.SELFPID
			msg.is_valid = 1
			msg.system = 0.00
			msg.buffer_sz = 0
			msg.cache_sz = 0
			msg.heap_sz = 0
			msg.stack_sz = 0
			self.pub.publish(msg)
		"""
		self.get_logger().info("{} done.".format(self.SELFNODE))
		print('data size[] = {}, mean([]): {:.6f}, max([]): {:.6f}'.format(
			len(self.exec_time), np.mean(np.array(self.exec_time)), max(self.exec_time)
		))

	def ptree_dfs(self, proc, pid):
		plist, search = [], [pid]
		while len(search) > 0:
			k = search.pop(0)
			plist.append(k)
			search.extend(smem.smem.get_ppid_to_pids(proc)[k])
		return set(plist)
	
	def read_status(self, filename, mode):
		fd = open(filename, mode)
		replace_ch = ['\r', '\n', '\t', 'kB', ' ']
		dic_line = {}

		while True:
			try:
				line = fd.readline()
				for ch in replace_ch:
					line = line.replace(ch, '')
				line = line.split(':')
				dic_line[line[0]] = line[1]
			except Exception:
				break

		return ( float(dic_line['VmData']), float(dic_line['VmStk']) )


	def pub_callback(self):
		start = time.time()

		## get mem-information
		# a1, a2: buffer, cache
		memory_obj = psutil.virtual_memory()	# byte

		# b: heap, stack
		heap = 0.00
		stack = 0.00
		smem.smem.set_options(None)
		proc = smem.smem.procdata(None)
		pid_list = self.ptree_dfs(proc, self.TOPPARENT)

		for pid in pid_list:
			filename, mode = '/proc/{}/status'.format(pid), 'r'
			ret = self.read_status(filename, mode)	# kB
			heap += ret[0]
			stack += ret[1]
		
		## MemProcMsg setup
		try:
			msg = MemProcMsg()
			msg.vgid = self.SELFPID
			msg.system = float(memory_obj.percent)

			# convert byte, kB -> MB
			msg.buffer_sz = memory_obj.buffers / (1e+6)
			msg.cache_sz = memory_obj.cached / (1e+6)
			msg.heap_sz = heap / (1e+3)
			msg.stack_sz = stack / (1e+3)
			msg.is_valid = 0
			self.get_logger().info("pgid: {}, sy: {:.4f}, b: {:.4f}, c: {:.4f}, h: {:.4f}, st: {:.4f}".format( \
				msg.vgid, msg.system, msg.buffer_sz, msg.cache_sz, msg.heap_sz, msg.stack_sz
			))
			self.pub.publish(msg)
		except Exception:
			pass
		finally:
			end = time.time()
		self.exec_time.append(end - start)
		self.get_logger().info('time: {:.4f}'.format(end - start))


def main(args = None):
	rclpy.init(args = args)
	node = MemoryInform()
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
MemProcMsg:
	int32 vgid
	int32 is_valid
	float32 system
	int32 buffer_sz
	int32 cache_sz
	int32 heap_sz
	int32 stack_sz
"""
