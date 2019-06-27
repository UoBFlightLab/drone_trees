#!/usr/bin/python

import py_trees
import time
import socket
from drone_trees import *
from battery_caller import battery_caller
from dronekit import connect

# Connect to the Vehicle.
#connection_string = 'tcp:127.0.0.1:5762'
connection_string = 'udp:127.0.0.1:14551'
#connection_string = 'tcp:127.0.0.1:5762' # if mission planner on
print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None

# build tree

# utility function: change mode and keep retrying until confirmed
def ensure_mode(vehicle,mode_name):
    b = py_trees.decorators.FailureIsRunning(py_trees.composites.Sequence(name="Ensure %s" % mode_name,
                                                                          children=[ChangeMode(vehicle,mode_name),
                                                                                    CheckMode(vehicle,mode_name)]))
    return b

# utility function for behaviour to move by offset and wait until almost stationary
def move_behaviour(vehicle, dNorth, dEast, dDown):    
    move = py_trees.composites.Sequence(name="move",
                                        children=[MoveDrone(vehicle,dNorth, dEast, dDown),                                        
                                                  py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                  py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle,0.1)),
                                                  PlaySound('sounds/movecompleted.wav')])
    return move

def wait_for_alt_local_over(vehicle,alt):
    return py_trees.decorators.FailureIsRunning(AltLocalAbove(vehicle,alt))

# put a one-shot over the whole mission, else it takes off again after landing
root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Simple Flight",
                                                                  children=[py_trees.decorators.FailureIsRunning(AltLocalAbove(vehicle,10)),
                                                                            py_trees.decorators.FailureIsRunning(CheckMode(vehicle,'GUIDED')),
                                                                            move_behaviour(vehicle,-20,20,-4),
                                                                            move_behaviour(vehicle,-20,-20,4)]),
                                     name='OneShot')

# piccies
py_trees.display.render_dot_tree(root, name='simple_moves')

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
    print('Battery {} V'.format(vehicle.battery.voltage))
    print(vehicle.mode.name)
    print('Altitude rel home {} m'.format(vehicle.location.global_relative_frame.alt))
    print('Velocity NED: {}'.format(vehicle.velocity))
    # now the tree bit
    print("+++++++++++++++++++++")
    behaviour_tree.tick()
    #ascii_tree = py_trees.display.ascii_tree(behaviour_tree.root,snapshot_information=snapshot_visitor)
    #print(ascii_tree)
    # pause
    time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
