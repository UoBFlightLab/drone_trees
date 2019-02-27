#!/usr/bin/env python

import py_trees
import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative

# Connect to the Vehicle.
connection_string = 'tcp:127.0.0.1:5760'
connection_string = 'tcp:127.0.0.1:5762' # if mission planner on
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


class ChangeMode(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(ChangeMode, self).__init__(mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode = VehicleMode(mode_name)

    def update(self):
        self._vehicle.mode = self._mode
        return py_trees.common.Status.SUCCESS


class IsArmable(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(IsArmable, self).__init__('is_armable')
        self._vehicle = vehicle
        
    def update(self):
        if self._vehicle.is_armable:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class ArmDrone(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(ArmDrone, self).__init__('Arm')
        self._vehicle = vehicle
        
    def update(self):
        self._vehicle.armed=True
        return py_trees.common.Status.SUCCESS
    

class SimpleTakeoff(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(SimpleTakeoff, self).__init__("Take off %i" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        print("TAKEOFF")
        self._vehicle.simple_takeoff(self._altitude)
        return py_trees.common.Status.SUCCESS

class AltGlobalAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(AltGlobalAbove, self).__init__("Over %f (global)" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        if self._vehicle.location.global_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation;
    
class MoveDrone(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, dNorth, dEast, dDown):
        super(MoveDrone, self).__init__("Move (%f,%f,%f) NED" % (dNorth, dEast, dDown))
        self._vehicle = vehicle
        self._dNorth = dNorth
        self._dEast = dEast
        self._dDown = dDown    

    def update(self):
        current_loc = self._vehicle.location.global_frame
        target_loc = get_location_metres(current_loc, self._dNorth, self._dEast)
        target_loc.alt = target_loc.alt - self._dDown
        self._vehicle.simple_goto(target_loc)
        return py_trees.common.Status.SUCCESS

class LatSpeedUnder(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, max_speed):
        super(LatSpeedUnder, self).__init__("Lat speed < %f" % max_speed)
        self._vehicle = vehicle
        self._max_speed = max_speed

    def update(self):
        (vx,vy,vz) = self._vehicle.velocity
        current_speed = math.sqrt(vx*vx + vy*vy)
        if current_speed < self._max_speed:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

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

root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="Flight",
                                    children=[launch,
                                              move_behaviour(vehicle,20,20,0),
                                              move_behaviour(vehicle,20,-20,0),
                                              ChangeMode(vehicle,'RTL')]),
                                   name="Mission")

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
