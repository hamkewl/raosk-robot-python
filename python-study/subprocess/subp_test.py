import subprocess
from subprocess import PIPE

proc = subprocess.run("date", shell = True, stdout = PIPE, stderr = PIPE, text = True)
date = proc.stdout
print('STDOUT: {}'.format(date))
