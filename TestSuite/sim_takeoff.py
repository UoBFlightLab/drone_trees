import time
import socket
from dronekit import connect
import threading
import signal

connection_string = 'udp:127.0.0.1:14552'

print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None

print("Basic pre-arm checks")
# Don't try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

print ("Arming motors")
# Copter should arm in GUIDED mode
vehicle.mode = 'GUIDED'
vehicle.armed   = True  

# Take-off
vehicle.simple_takeoff(5)

time.sleep(10)
# Change mode to auto
vehicle.parameters['SIM_SPEEDUP']=1
vehicle.mode = 'AUTO'

vehicle.close()