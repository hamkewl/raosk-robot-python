#!/usr/bin/env /usr/bin/python3

import os
import signal
import time
import queue as que
import numpy as np
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node

from raosk_msgsrv.msg import CopiedBinaryMsg ## Publish
from raosk_msgsrv.msg import AbortPidsMsg	## Subscribe

## debugging
import traceback
import pprint

class Offloader(Node):
  SELFNODE = 'raosk_offloader'
  PUBTOPIC = 'core_path'
  SUBTOPIC = 'abort_pids'

  ## File discriptor & thread_list
  pid_queue = que.Queue()
  offload_executor = ThreadPoolExecutor(max_workers = 256)

  ## to evaluation
  pub_exec_time, sub_exec_time, binread_exec_time = [0.], [0.], [0.]

  def __init__(self):
    super().__init__(self.SELFNODE)
    self.get_logger().info("{} initializing...".format((self.SELFNODE)))
    self.pub = self.create_publisher(CopiedBinaryMsg, self.PUBTOPIC, 10)
    self.create_timer(1.00, self.pub_callback)
    self.sub = self.create_subscription(AbortPidsMsg, self.SUBTOPIC, self.sub_callback, 10)
    self.get_logger().info("{} do...".format(self.SELFNODE))

  def __del__(self):
    self.get_logger().info("{} done.".format(self.SELFNODE))
    print('  (Pub)   data size[] = {}, mean([]): {:.6f}, max([]): {:.6f}'.format(
      len(self.pub_exec_time), np.mean(np.array(self.pub_exec_time)), max(self.pub_exec_time)
    ))
    print('  (Sub)   data size[] = {}, mean([]): {:.6f}, max([]): {:.6f}'.format(
      len(self.sub_exec_time), np.mean(np.array(self.sub_exec_time)), max(self.sub_exec_time)
    ))
    print('(binread) data size[] = {}, mean([]): {:.6f}, max([]): {:.6f}'.format(
      len(self.binread_exec_time), np.mean(np.array(self.binread_exec_time)), max(self.binread_exec_time)
    ))
  
  ## Multi-threaded processing of sending core dump binaries
  def binread_publish(self, pid):
    corepath, mode = '/tmp/core.{}.bin'.format(pid), 'rb'	## Binary-Read mode
    read_len = (1000 * 10**3)	## 1000KB = 1MB

    try:
      fd = open(corepath, mode)
      while True:
        start = time.time()
        msg = CopiedBinaryMsg()
        bin_data = fd.read(read_len)
        
        msg.pid = pid
        for by in bin_data:
          msg.core_data.append( bytes(by) )
        
        if not bin_data:
          msg.status = 1
          self.pub.publish(msg)
          break
        else:
          msg.status = 0
          self.pub.publish(msg)
        
        end = time.time()
        self.binread_exec_time.append(end - start)
        time.sleep(1.)
      
      fd.close()
      os.remove(corepath)
    except FileNotFoundError:
      pass


  def pub_callback(self):
    start = time.time()
    print(self.pid_queue.qsize())
    
    while not self.pid_queue.empty():
      pid = self.pid_queue.get()
      self.offload_executor.submit(self.binread_publish, pid)

    end = time.time()
    self.pub_exec_time.append(end - start)
    self.get_logger().info('(Pub) time: {:.4f}, que: {}'.format(end - start, self.pid_queue.empty()))

  def sub_callback(self, msg):
    start = time.time()

    abort_target = msg.vgid
    try:
      os.killpg(abort_target, signal.SIGABRT)
      for pid in msg.abort_pid:
        self.pid_queue.put(pid)
    except ProcessLookupError:
      self.get_logger().info('vgid {} is not exist'.format(abort_target))
    except FileNotFoundError:
      self.get_logger().info('corefile {} is not exist'.format(corename))

    end = time.time()
    self.sub_exec_time.append(end - start)
    self.get_logger().info('(Sub) time: {:.4f}'.format(end - start))

def main(args = None):
  rclpy.init(args = args)
  node = Offloader()
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
