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

"""
def wp_precond_rtk_check(wp_n):
    rtk_wait_and_resolve = wait_resolve_or_goSafti(vehicle, va, wp_n, SAFTI, CheckGPS(vehicle, 5),
                                                   "No RTK Fix", "RTK Fix Recovered",
                                                    name="RTK Wait & Resolve or Go Safti")
    rtk_check = precond_module(va, name="RTK Check",
                         safety_check=CheckGPS(vehicle, 5),
                         mishab_voice="No RTK Fix"
                         fallback=rtk_wait_and_resolve)
    
    rtk_check.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return rtk_check

def wp_precond_clearance(wp_clearance):
    precond = precond_module(va, name=f"WP Clearance > {wp_clearance}?", 
                        safety_check=CheckObstacle(vehicle, wp_clearance),
                        fallback=go_SAFTI(vehicle, va, SAFTI, "Clearance Fail"))
    
    precond.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return precond

flight_manager = flight_manager(vehicle, va,
                safety_modules=[safety_low_battery, safety_obstacle_check],
                legs=[at_wp(vehicle, va, 2),
                      leg_handler(vehicle, va, 3, "SAFTI", precond_next_wp=[wp_precond_rtk_check(3), wp_precond_clearance(45)]),
                      leg_handler(vehicle, va, 5, "TWENTI", precond_next_wp=[wp_precond_rtk_check(5), wp_precond_clearance(15)]),
                      leg_handler(vehicle, va, 7, "TENNA", precond_next_wp=[wp_precond_rtk_check(7), wp_precond_clearance(8)]),
                      leg_handler(vehicle, va, 9, "ALPHA", precond_next_wp=[wp_precond_rtk_check(9), wp_precond_clearance(4)]),
                      at_wp(vehicle, va, 11),
                      at_wp(vehicle, va, 12)])
"""
flight_manager = fm_behaviour(vehicle, va, wp_count, safety_modules=[safety_low_battery, safety_obstacle_check])
root = py_trees.composites.Sequence(name="OPS",
                                    children=[py_trees.decorators.OneShot(preflight, name="OneShot \n Pre-flight"),
                                              py_trees.decorators.OneShot(take_off(vehicle, va), name="OneShot \n Take-off"),
                                              py_trees.decorators.OneShot(flight_manager, name="OneShot \n Flight Manager"),
                                              py_trees.decorators.OneShot(landing(vehicle, va), name="OneShot \n Landing")])
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