#!/usr/bin/env python

import py_trees
import time
import socket
from drone_trees import *
from dronekit import connect

# Connect to the Vehicle.
#connection_string = 'tcp:127.0.0.1:5762'
connection_string = 'tcp:127.0.0.1:14550'
#connection_string = 'tcp:127.0.0.1:5762' # if mission planner on
print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print e
    # proceed just with a blank object so I can render the tree
    vehicle=None

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

# utility function: change mode and keep retrying until confirmed
def ensure_mode(vehicle,mode_name):
    b = py_trees.decorators.FailureIsRunning(py_trees.composites.Sequence(name="Ensure %s" % mode_name,
                                                                          children=[ChangeMode(vehicle,mode_name),
                                                                                    CheckMode(vehicle,mode_name)]))
    return b
                                            

# start-up and take-off sequence, waiting until climb to given altitude
launch = py_trees.composites.Sequence(name="Launch",
                                    children=[ensure_mode(vehicle,'GUIDED'),
                                              py_trees.decorators.FailureIsRunning(IsArmable(vehicle)),
                                              arm_drone,
                                              SimpleTakeoff(vehicle,20),
                                              PlaySound('sounds/TakeOff.wav'),
                                              py_trees.decorators.FailureIsRunning(AltLocalAbove(vehicle,18))])

# utility function for behaviour to move by offset and wait until almost stationary
def move_behaviour(vehicle, dNorth, dEast, dDown):    
    move = py_trees.composites.Sequence(name="move",
                                        children=[MoveDrone(vehicle,dNorth, dEast, dDown),                                        
                                                  py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(vehicle,1.0))),
                                                  py_trees.decorators.FailureIsRunning(LatSpeedUnder(vehicle,0.1)),
                                                  PlaySound('sounds/MoveCompleted.wav')])
    return move

# utility for commonly used decoration
def wait_while(target_behaviour):
    return py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(target_behaviour))

# utility func for announcing altitude on descent
def call_descent_behaviour(vehicle, alt, filename):
    b = py_trees.composites.Sequence(name="call alt %f" % alt,
                                     children=[wait_while(AltLocalAbove(vehicle,alt)),
                                               PlaySound(filename)])
    return b


# land, including wait until disarm
land = py_trees.composites.Sequence(name="land",
                                    children=[ensure_mode(vehicle,'RTL'),                                        
                                              call_descent_behaviour(vehicle,10,'sounds/Alt10m.wav'),
                                              call_descent_behaviour(vehicle,5,'sounds/Alt5m.wav'),
                                              py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(IsArmed(vehicle)))])

# put a one-shot over the whole mission, else it takes off again after landing
flight = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Simple Flight",
                                                                  children=[config,
                                                                            launch,
                                                                            move_behaviour(vehicle,20,20,0),
                                                                            move_behaviour(vehicle,20,-20,0),
                                                                            move_behaviour(vehicle,20,20,0),
                                                                            move_behaviour(vehicle,20,-20,0),
                                                                            land]),
                                     name='OneShot')

# battery caller utility
def battery_caller(vehicle,level,word):
    bt = py_trees.decorators.FailureIsRunning(py_trees.decorators.OneShot(py_trees.composites.Sequence(children=[BatteryLevelAbove(vehicle,level),
                                                                            wait_while(BatteryLevelAbove(vehicle,level)),
                                                                            PlaySound("sounds/%s.wav" % word),
                                                                            PlaySound("sounds/percent.wav")]),
                                     name="call_%s" % word))
    return bt

battery_callers = py_trees.composites.Parallel(children=[battery_caller(vehicle,80,'eighty'),
                                                         battery_caller(vehicle,60,'sixty'),
                                                         battery_caller(vehicle,40,'forty'),
                                                         battery_caller(vehicle,20,'twenty')],
                                               name='Call each level')

root = py_trees.composites.Parallel(children=[flight,
                                              battery_callers],
                                    name='Parallel')

# piccies
#py_trees.display.render_dot_tree(root, name='simple_flight')

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
