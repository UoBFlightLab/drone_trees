#!/usr/bin/env python

import py_trees
import time
import socket
from drone_trees import *
from battery_caller import battery_caller
from dronekit import connect
from mission_utility import *

# Connect to the Vehicle.
connection_string = 'udp:127.0.0.1:14551'

print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None

# build tree

# pre-flight

SAFTI = 7 # SAFTI waypint number

def preflight_GPS(vehicle):
    bt = py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(children=[CheckGPS(vehicle, 4),
                                                                                     WarningSound('sounds/NoRTK.wav')]))
    return bt

def preflight_EKF(vehicle):
    bt = py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(children=[CheckEKF(vehicle),
                                                                                     WarningSound('sounds/BadEKF.wav')]))
    return bt

def preflight_IsArmable(vehicle):
    bt = py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(children=[IsArmable(vehicle),
                                                                                     WarningSound('sounds/DroneNotArmable')]))
    return bt

preflight = py_trees.composites.Sequence(name="Pre-flight",
                                         children=[preflight_GPS(vehicle),
                                                   preflight_EKF(vehicle),
                                                   preflight_IsArmable(vehicle)])
# mission

def mission_GPS(vehicle):
    bt = py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(children=[CheckGPS(vehicle, 5),
                                                                                     GoSAFTI(vehicle, SAFTI)])))
    return bt

def mission_EKF(vehicle):
    bt = py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(children=[CheckEKF(vehicle),
                                                                                     GoSAFTI(vehicle, SAFTI)])))
    return bt

def wpChecklist(vehicle):
    bt = py_trees.composites.Sequence(name='WayPoint Checklist', children=[mission_GPS(vehicle),
                                                                           mission_EKF(vehicle)])
    return bt

def wpNode(vehicle, wpn):
    bt = py_trees.decorators.OneShot(py_trees.composites.Sequence(name=('Waypoint %i' % wpn),
                                                                  children=[py_trees.decorators.FailureIsRunning(CheckCounter(vehicle, wpn)),
                                                                            py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                                            py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle, 1)),
                                                                            wpChecklist(vehicle)]))
    return bt


def wpNode_noCheck(vehicle, wpn):
    bt = py_trees.decorators.OneShot(py_trees.composites.Sequence(name=('Waypoint %i' % wpn),
                                                                  children=[py_trees.decorators.FailureIsRunning(CheckCounter(vehicle, wpn)),
                                                                            py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                                            py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle, 1))]))
    return bt

mission = py_trees.composites.Parallel(name='Mission', children=[MissionUpload(vehicle, 'seg1.txt'),
                                                                 wpNode_noCheck(vehicle, 1),
                                                                 wpNode(vehicle, 2),
                                                                 wpNode(vehicle, 3),
                                                                 wpNode(vehicle, 4),
                                                                 wpNode(vehicle, 5),
                                                                 wpNode(vehicle, 6),
                                                                 wpNode_noCheck(vehicle, 7),
                                                                 wpNode_noCheck(vehicle, 8),
                                                                 wpNode_noCheck(vehicle, 9)])

root = py_trees.composites.Sequence(name="OPS",
                                    children=[preflight,
                                              mission])
# piccies
py_trees.display.render_dot_tree(root, name='farm_flight')

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
    print(vehicle.armed)
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
vehicle.close()