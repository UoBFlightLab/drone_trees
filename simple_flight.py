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

# set the frame class http://ardupilot.org/copter/docs/parameters.html
vehicle.parameters['FRAME_CLASS']=1 # quad
vehicle.parameters['FRAME_TYPE']=0 # plus

# disable vehicle arming checks to prevent accel hangup
vehicle.parameters['ARMING_CHECK']=0

# set to always land if RC lost
# http://ardupilot.org/copter/docs/parameters.html
vehicle.parameters['FS_THR_ENABLE']=3

# build tree        
init_guided = ChangeMode(vehicle,'GUIDED')
wait_armable = py_trees.decorators.FailureIsRunning(IsArmable(vehicle))
arm_drone = ArmDrone(vehicle)
take_off = SimpleTakeoff(vehicle,20)
climb = py_trees.decorators.FailureIsRunning(AltGlobalAbove(vehicle,600))
launch = py_trees.composites.Sequence(name="Launch",
                                    children=[init_guided,
                                              wait_armable,
                                              arm_drone,
                                              take_off,
                                              climb])

def move_behaviour(vehicle,dNorth, dEast, dDown):    
    move = py_trees.composites.Sequence(name="move",
                                        children=[MoveDrone(vehicle,dNorth, dEast, dDown),                                        
                                                  py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                  py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle,0.1))])
    return move

root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Simple Flight",
                                   children=[launch,
                                             move_behaviour(vehicle,20,20,0),
                                             move_behaviour(vehicle,20,-20,0),
                                             ChangeMode(vehicle,'RTL')]))

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
