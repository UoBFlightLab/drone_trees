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

# Wait for the vehicle to be armed and flying towards waypoint 5
while not (vehicle.is_armable and vehicle.commands.next > 3):
    time.sleep(1)

# Set mode to guided whilst battery level is above 31%
while vehicle.battery.level > 31:
    vehicle.mode = 'GUIDED'
    time.sleep(1)

vehicle.mode = 'AUTO'

vehicle.close()