import py_trees
import time
import socket
import sys
from drone_trees import *
from dronekit import connect
import pyttsx3
from queue import Queue
import threading
import signal


# Connect to the Vehicle.
connection_string = 'udp:127.0.0.1:14551'

print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None

#Initialising the Voice Assistant
va = VoiceAssistant()
# Starting Voice Assistant Thread
va.start()

# Generate executable mission file
m = MissionUtility(vehicle)
wp_count = m.gen_exe_mission("seg2.txt", 7, 10)
SAFTI = wp_count-2 # SAFTI waypoint number

def cleanup():
    global va
    global vehicle
    va.kill()
    #va.join()
    l.terminate()
    vehicle.close()
    sys.exit()

def handler(signum, frame):
    print('Signal handler called with signal', signum)
    cleanup()

signal.signal(signal.SIGINT, handler)

# build tree

# pre-flight


preflight_GPS_Check = preflight_Module(vehicle, va, 
                                        name="Preflight GPS Check",
                                        safety_check=CheckGPS(vehicle, 4),
                                        fallback=PlaySound('No RTK', va, returnFailure=True))

preflight_EKF_Check = preflight_Module(vehicle, va, 
                                        name="Preflight EKF Check",
                                        safety_check=CheckEKF(vehicle),
                                        fallback=PlaySound('Bad EKF', va, returnFailure=True))

preflight = py_trees.composites.Sequence(name="Pre-flight",
                                         children=[preflight_GPS_Check,
                                                   preflight_EKF_Check,
                                                   MissionUpload(vehicle, m, "output_mission.txt")])
# Flight Manager

safety_low_battery = safety_module(va, name="Low Battery",
                           safety_check=BatteryLevelAbove(vehicle, 30),
                           mishap_tts="Low battery", 
                           fallback=go_SAFTI(vehicle, va, SAFTI))

safety_obstacle_check = safety_module(va, name="Obstacle Check", 
                              safety_check=CheckObstacle(vehicle, 2),
                              mishap_tts="Obstacle ahead",
                              fallback=go_SAFTI(vehicle, va, SAFTI))


flight_manager = fm_behaviour(vehicle, va, wp_count, safety_modules=[safety_low_battery, safety_obstacle_check])
root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="OPS",
                                                                children=[preflight,
                                                                          take_off(vehicle, va),
                                                                          flight_manager,
                                                                          landing(vehicle, va)]))
# piccies
py_trees.display.render_dot_tree(root, name='Sim_Demo')

# stop here if vehicle never connected
if vehicle==None:
  exit()

# tree
behaviour_tree = py_trees.trees.BehaviourTree(root)
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)

# Initiate Log
l = log(root, sys.argv[1])

# run the thing
# and every second for five minutes, print stuff
for ii in range(300):
    print("******* %i ********" % ii)
    print(vehicle.battery)
    print(vehicle.mode.name)
    print(vehicle.location.global_frame.alt)
    print(vehicle.velocity)
    # now the tree bit
    print("+++++++++++++++++++++")
    behaviour_tree.tick()
    unicode_tree = py_trees.display.unicode_tree(behaviour_tree.root,
                                            visited=snapshot_visitor.visited,
                                            previously_visited=snapshot_visitor.visited)
    print(unicode_tree)
    # log
    l.logging(ii)
    # pause
    time.sleep(1)

# Close vehicle object before exiting script
cleanup()