import os
import sys
import time
import signal
import random as rd

start, end = 0, 0
clear_line = rd.randint(5, 15)
def receive_signal(signum, stack):
	if signum in [30, 10, 16]:
		end = time.time()
		passed, diff = end - start, abs(clear_line - end + start)
		if(diff <= 0.5000):
			print('congrats! time = {:.2f}s (diff: {:.2f}s)'.format(passed, diff))
		else:
			print('oh :( time = {:.2f}s (diff: {:.2f}s)'.format(passed, diff))
		exit(0)
	else:
		print('this signal is not needed :(')

signal.signal(signal.SIGUSR1, receive_signal)
signal.signal(signal.SIGUSR2, receive_signal)
signal.signal(signal.SIGALRM, receive_signal)

print('Enter /start to game start')
cin = input()
PID_list = []

while cin != "/start":
	print('Re-Enter /start..')
	cin = input()

print('stop the {} seconds using SIGUSR1'.format(clear_line))
start = time.time()
PID_list.append(os.getpid())
print('PID_list: {}'.format(PID_list))

time.sleep(clear_line + 5)
print('timeover..')
