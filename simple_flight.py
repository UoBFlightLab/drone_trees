#!/usr/bin/env python

import py_trees
import time
from drone_trees import *
from dronekit import connect

# Connect to the Vehicle.
connection_string = 'tcp:127.0.0.1:5760'
#connection_string = 'tcp:127.0.0.1:5762' # if mission planner on
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# build tree

# parameter setting
config = py_trees.composites.Sequence(name="Config",
                                        children=[SetParam(vehicle,'FRAME_CLASS',1),
                                                  SetParam(vehicle,'FRAME_TYPE',0),
                                                  SetParam(vehicle,'ARMING_CHECK',0),
                                                  SetParam(vehicle,'FS_THR_ENABLE',3)])

# keep sending arm until returns armed=true
arm_drone = py_trees.decorators.FailureIsRunning(py_trees.composites.Sequence(name="Arming",
                                                                              children=[ArmDrone(vehicle),
                                                                                        IsArmed(vehicle)]))

# start-up and take-off sequence, waiting until climb to given altitude
launch = py_trees.composites.Sequence(name="Launch",
                                    children=[ChangeMode(vehicle,'GUIDED'),
                                              py_trees.decorators.FailureIsRunning(IsArmable(vehicle)),
                                              arm_drone,
                                              SimpleTakeoff(vehicle,20),
                                              py_trees.decorators.FailureIsRunning(AltGlobalAbove(vehicle,600))])

# utility function for behaviour to move by offset and wait until almost stationary
def move_behaviour(vehicle, dNorth, dEast, dDown):    
    move = py_trees.composites.Sequence(name="move",
                                        children=[MoveDrone(vehicle,dNorth, dEast, dDown),                                        
                                                  py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                  py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle,0.1))])
    return move

# land, including wait until disarm
land = py_trees.composites.Sequence(name="land",
                                    children=[ChangeMode(vehicle,'RTL'),                                        
                                              py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(IsArmed(vehicle)))])

# put a one-shot over the whole mission, else it takes off again after landing
root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Simple Flight",
                                   children=[config,
                                             launch,
                                             move_behaviour(vehicle,20,20,0),
                                             move_behaviour(vehicle,20,-20,0),
                                             land]))

# piccies
py_trees.display.render_dot_tree(root)

# tree
behaviour_tree = py_trees.trees.BehaviourTree(root)
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)

# run the thing
# and every second for five minutes, print stuff
for ii in range(300):
    print "******* %i ********" % ii
    print vehicle.armed
    print vehicle.battery
    print vehicle.mode.name
    print vehicle.location.global_frame.alt
    print vehicle.velocity
    print vehicle.rangefinder.distance
    # now the tree bit
    print "+++++++++++++++++++++"
    behaviour_tree.tick()
    ascii_tree = py_trees.display.ascii_tree(behaviour_tree.root,snapshot_information=snapshot_visitor)
    print(ascii_tree)
    # pause
    time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
