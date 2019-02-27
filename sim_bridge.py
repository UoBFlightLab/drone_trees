import subprocess
import time

cmd=["../ardupilot/build/sitl/bin/arducopter.exe", "--model quad --home 51.454962,-2.627718,584,270"]
cmd=["../ardupilot/build/sitl/bin/arducopter.exe", "--model", "quad"]
#cmd=["ls", "-a", "-l"]

print("Starting SITL and waiting 20 seconds")
sitl_pid = subprocess.Popen(cmd)
time.sleep(20)
print("Time up - killing SITL")
sitl_pid.kill()
