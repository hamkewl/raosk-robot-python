import subprocess
from subprocess import PIPE

# start subprocess
proc = subprocess.Popen("sleep 3; ls *.py aa", shell = True, stdout = PIPE, stderr = PIPE, text = True)
print(proc, type(proc))

# wait until subprocess finish
result = proc.communicate()
print(result, type(result))		# tuple

(stdout, stderr) = (result[0], result[1])
print('STDOUT: {}'.format(stdout))
print('STDERR: {}'.format(stderr))

# the output can be converted to a binary string by using .encode ()
print('\ntype(stdout: {}) is {}'.format(stdout.encode(), type(stdout.encode())))		# bytes
