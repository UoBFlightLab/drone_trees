import py_trees
import time
import socket
from drone_trees import *
from battery_caller import battery_caller
from dronekit import connect
from mission_utility import *
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

def cleanup():
    global va
    global vehicle
    va.kill()
    va.join()
    vehicle.close()
    sys.exit()

def handler(signum, frame):
    print('Signal handler called with signal', signum)
    cleanup()

signal.signal(signal.SIGINT, handler)

# build tree

# pre-flight

SAFTI = 7 # SAFTI waypint number

preflight_GPS_Check = preflight_Module(vehicle, va, 
                                        name="Preflight GPS Check",
                                        safety_check=CheckGPS(vehicle, 4),
                                        fallback=PlaySound('No RTK', va, returnFailure=True))

preflight_EKF_Check = preflight_Module(vehicle, va, 
                                        name="Preflight EKF Check",
                                        safety_check=CheckEKF(vehicle),
                                        fallback=PlaySound('Bad EKF', va, returnFailure=True))

preflight = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Pre-flight",
                                                                     children=[preflight_GPS_Check,
                                                                               preflight_EKF_Check,
                                                                               MissionUpload(vehicle, 'seg1.txt')]))
# Flight Manager

safety_low_battery = safty_module(name="Low Battery",
                           safety_check=BatteryLevelAbove(vehicle, 1), 
                           fallback=go_SAFTI(vehicle, va, SAFTI, "Low Battery"))

safety_obstacle_check = safty_module(name="Obstacle Check", 
                              safety_check=CheckObstacle(vehicle, 2),
                              fallback=go_SAFTI(vehicle, va, SAFTI, "Obstacle Ahead"))


def wp_precond_rtk_check(wp_n):
    rtk_wait_and_resolve = wait_resolve_or_goSafti(vehicle, va, wp_n, SAFTI, CheckGPS(vehicle, 5),
                                                   "No RTK Fix", "RTK Fix Recovered",
                                                    name="RTK Wait & Resolve or Go Safti")
    rtk_check = precond_module(name="RTK Check",
                         safety_check=CheckGPS(vehicle, 5),
                         fallback=rtk_wait_and_resolve)
    return rtk_check


flight_manager = flight_manager(vehicle, va,
                safty_modules=[safety_low_battery, safety_obstacle_check],
                legs=[at_wp(vehicle, va, 2),
                      leg_handler(vehicle, va, 3, "SAFTI", precond_next_wp=wp_precond_rtk_check(3)),
                      leg_handler(vehicle, va, 4, "TWENTI", precond_next_wp=wp_precond_rtk_check(4)),
                      leg_handler(vehicle, va, 5, "TENNA", precond_next_wp=wp_precond_rtk_check(5)),
                      leg_handler(vehicle, va, 6, "ALPHA", precond_next_wp=wp_precond_rtk_check(6)),
                      at_wp(vehicle, va, 7),
                      at_wp(vehicle, va, 8)])

root = py_trees.composites.Sequence(name="OPS",
                                    children=[preflight,
                                              flight_manager])
# piccies
py_trees.display.render_dot_tree(root, name='Sim_Demo')

# stop here if vehicle never connected
if vehicle==None:
  exit()

# tree
behaviour_tree = py_trees.trees.BehaviourTree(root)
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)

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
    # pause
    time.sleep(1)

# Close vehicle object before exiting script
cleanup()