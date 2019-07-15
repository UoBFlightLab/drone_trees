#!/usr/bin/env python

import py_trees
from drone_trees import *
from dronekit import connect
import socket
import time

# utility for commonly used decoration
def wait_while(target_behaviour):
    return py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(target_behaviour))


# battery caller utility
def battery_level_caller(vehicle,level,word):
    bt = py_trees.decorators.FailureIsRunning(py_trees.decorators.OneShot(py_trees.composites.Sequence(children=[BatteryLevelAbove(vehicle,level),
                                                                                                                 wait_while(BatteryLevelAbove(vehicle,level)),
                                                                                                                 PlaySound("sounds/%s.wav" % word),
                                                                                                                 PlaySound("sounds/percent.wav")],
                                                                                                       name="call_%s" % word)))
    return bt


def battery_caller(vehicle):
    bt = py_trees.composites.Parallel(children=[battery_level_caller(vehicle,90,'ninety'),
                                                battery_level_caller(vehicle,80,'eighty'),
                                                battery_level_caller(vehicle,70,'seventy'),
                                                battery_level_caller(vehicle,60,'sixty'),
                                                battery_level_caller(vehicle,50,'fifty'),
                                                battery_level_caller(vehicle,40,'forty'),
                                                battery_level_caller(vehicle,30,'thirty'),
                                                battery_level_caller(vehicle,20,'twenty'),
                                                battery_level_caller(vehicle,10,'ten')])
    return bt


# if run as script, run this tree only
if __name__=='__main__':
    connection_string = 'tcp:127.0.0.1:14550'
    print("Connecting to vehicle on: %s" % (connection_string,))
    try:
        vehicle = connect(connection_string, wait_ready=True)
    except socket.error as e:
        print(e)
        # proceed just with a blank object so I can render the tree
        vehicle=None

    # make a tree with only battery caller
    root = battery_caller(vehicle)

    # if vehicle never connected, render the tree and stop
    if vehicle==None:
        py_trees.display.render_dot_tree(root, name='battery_caller')
        exit()

    # tree
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    snapshot_visitor = py_trees.visitors.SnapshotVisitor()
    behaviour_tree.visitors.append(snapshot_visitor)

    # run the thing
    # and every second for ten minutes, print stuff
    for ii in range(600):
        print("******* %i ********" % ii)
        print(vehicle.armed)
        print(vehicle.battery)
        print(vehicle.mode.name)
        # now the tree bit
        print("+++++++++++++++++++++")
        behaviour_tree.tick()
        ascii_tree = py_trees.display.ascii_tree(behaviour_tree.root,snapshot_information=snapshot_visitor)
        print(ascii_tree)
        # pause
        time.sleep(1)

    # Close vehicle object before exiting script
    vehicle.close()
