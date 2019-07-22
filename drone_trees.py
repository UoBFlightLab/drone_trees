#!/usr/bin/env python

import py_trees
import math
import os
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from mission_utility import upload_mission, readmission

class SetParam(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, param_name, new_value):
        # use name of mode as label for behaviour
        super(SetParam, self).__init__("%s=%i" % (param_name, new_value))
        self._vehicle = vehicle
        self._param_name = param_name
        self._new_value = new_value

    def update(self):
        self._vehicle.parameters[self._param_name]=self._new_value
        return py_trees.common.Status.SUCCESS


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

class CheckGPS(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, fixType):
        super(CheckGPS, self).__init__("GPS Status Check > %i" % fixType)
        self._vehicle = vehicle
        self._fixType = fixType

    def update(self):
        if self._vehicle.gps_0.fix_type > self._fixType:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
class CheckEKF(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckEKF, self).__init__("EKF Check")
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.ekf_ok:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckCounter(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(CheckCounter, self).__init__("Check Counter %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        if self._wpn == self._vehicle.commands.next:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class SetCounter(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(SetCounter, self).__init__("Set wp counter to %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        self._vehicle.commands.next = self._wpn
        return py_trees.common.Status.SUCCESS

class GoSAFTI(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(GoSAFTI, self).__init__("Go SAFTI (WP %i)" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        self._vehicle.commands.next = self._wpn
        return py_trees.common.Status.SUCCESS

class MissionUpload(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mission_file):
        super(MissionUpload, self).__init__('Mission Upload')
        self._vehicle = vehicle
        self._mission_file = mission_file
        
    def update(self):
        upload_mission(self._vehicle, self._mission_file)
        return py_trees.common.Status.SUCCESS

class CheckMode(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckMode, self).__init__("Is mode %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        if self._vehicle.mode.name == self._mode_name:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


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


class PlaySound(py_trees.behaviour.Behaviour):

    def __init__(self, filename):
        super(PlaySound, self).__init__("%s" % filename)
        self._filename = filename

    def update(self):
        os.system("cat %s > /dev/dsp" % self._filename)
        return py_trees.common.Status.SUCCESS

class WarningSound(py_trees.behaviour.Behaviour):

    def __init__(self, filename):
        super(WarningSound, self).__init__("%s" % filename)
        self._filename = filename

    def update(self):
        os.system("cat %s > /dev/dsp" % self._filename)
        return py_trees.common.Status.FAILURE

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


class AltLocalAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(AltLocalAbove, self).__init__("Over %f (local)" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        if self._vehicle.location.global_relative_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class BatteryLevelAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, level):
        super(BatteryLevelAbove, self).__init__("Bat over %f pc" % level)
        self._vehicle = vehicle
        self._level = level
        
    def update(self):
        if self._vehicle.battery.level >= self._level:
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
