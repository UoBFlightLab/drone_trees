import time
import socket
from dronekit import connect
import threading
import signal

connection_string = 'udp:127.0.0.1:14553'

print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None

# Wait for the vehicle to be armed and flying towards waypoint 4
while not (vehicle.is_armable and vehicle.commands.next == 4):
    time.sleep(1)

# Set distance sensor reading to 1m by varying the scaling
vehicle.parameters['RNGFND2_SCALING']=0.2

vehicle.close()