# -*- coding: utf-8 -*-
###############################################################################
# License: MIT License
#    https://raw.githubusercontent.com/UoBFlightLab/drone_trees/master/LICENSE
###############################################################################
# Author: Hirad Goudarzi
# Role: PhD Candidate
# Organisation: University of Bristol
# Version: 2.0.0
# Email: hirad.goudarzi@bristol.ac.uk
###############################################################################
"""

leaf_node.py:

Low-level leaf nodes representing drone actions and conditions.  The designer
can use these in their flight idioms or to directly populate their tree
templates.

"""
###############################################################################

import py_trees
import math
from dronekit import VehicleMode, LocationGlobal, LocationGlobalRelative


class MissionUpload(py_trees.behaviour.Behaviour):
    """
    Leaf node for uploading mission. Returns SUCCESS when mission is uploaded,
    returns FAILURE when armed since uploading is only permitted when drone
    is unarmed.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wplist : list of mavlink command waypoints

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, wplist):
        super(MissionUpload, self).__init__('Mission Upload')
        self._vehicle = vehicle
        self._wplist = wplist

    def update(self):
        # only permitted to upload if unarmed
        if self._vehicle.armed:
            return py_trees.common.Status.FAILURE
        else:
            cmds = self._vehicle.commands
            cmds.clear()
            # add a dummy command on the front
            # just the first WP used as a placeholder
            # upload seems to skip the first WP
            cmds.add(self._wplist[0])
            for wp in self._wplist:
                cmds.add(wp)
            cmds.upload()
            self.feedback_message = f'Uploaded {cmds.count} WPs'
            return py_trees.common.Status.SUCCESS


class MissionVerify(py_trees.behaviour.Behaviour):
    """
    Leaf node for verifying mission upload by checking the Mission
    Acknowledmnet message (link below) and having a non-zero waypoint.
        FAILURE: No mission detected onboard
        RUNNING: Mission detected but no acknowledment
        SUCCESS: Mission detected and acknowledged

    https://mavlink.io/en/messages/common.html#MISSION_ACK
    https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle):
        super(MissionVerify, self).__init__('Mission Verify')
        self._vehicle = vehicle

    def initialise(self):
        self._vehicle.commands.download()

    def update(self):
        if self._vehicle.commands.count < 1:
            self.feedback_message = 'No mission detected'
            return py_trees.common.Status.FAILURE
        else:
            if self._vehicle.mission_ack_type is None:
                self.feedback_message = 'Mission acknowledgement not received'
                return py_trees.common.Status.RUNNING
            # mission_ack_type of 0 means mission accepted
            # https://mavlink.io/en/messages/common.html#MAV_MISSION_ACCEPTED
            elif self._vehicle.mission_ack_type == 0:
                self.feedback_message = 'MAV misssion accepted'
                return py_trees.common.Status.SUCCESS


class SetParam(py_trees.behaviour.Behaviour):
    """
    Leaf node for setting ardupilot parameter.
    <https://ardupilot.org/copter/docs/parameters.html>
    Returns SUCCESS when the parameter is set.
    is set.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    param_name : str
        Name of the ardupilot parameter

    new_value : int or float or bool
        New value to be set. Type depends on the parameter

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """

    def __init__(self, vehicle, param_name, new_value):
        # use name of mode as label for behaviour
        super(SetParam, self).__init__("%s=%i" % (param_name, new_value))
        self._vehicle = vehicle
        self._param_name = param_name
        self._new_value = new_value

    def update(self):
        self._vehicle.parameters[self._param_name] = self._new_value
        return py_trees.common.Status.SUCCESS


class Land(py_trees.behaviour.Behaviour):
    """
    Action leaf node: switches flight mode to Return-to-Launch and returns
    SUCCESS
    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

"""
    def __init__(self, vehicle):
        super(Land, self).__init__()
        self._vehicle = vehicle
        self._mode = VehicleMode('RTL')

    def update(self):
        self._vehicle.mode = self._mode
        return py_trees.common.Status.SUCCESS


class CheckGPS(py_trees.behaviour.Behaviour):
    """
    Condition leaf node for checking GPS fix type
        FAILURE: gps fix status is below the specified limit
        SUCCESS: gps fix status is above the specified limit

    Type of GPS fix and status number
        0: NO GPS
        1: NO FIX
        2: 2D FIX
        3: 3D FIX
        4: DGPS
        5: RTK FLOAT
        6: RTK FIX

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    fixType : int
        GPS fix status number ranging from 0-6

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """

    def __init__(self, vehicle, fixType):
        super(CheckGPS, self).__init__(f"GPS Status Check > {fixType} ?")
        self._vehicle = vehicle
        self._fixType = fixType

    def update(self):
        gps_fix = self._vehicle.gps_0.fix_type
        if gps_fix > self._fixType:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No,only {}'.format(gps_fix)
            return py_trees.common.Status.FAILURE


class CheckDistance(py_trees.behaviour.Behaviour):
    """
    Condition leaf node for checking distance sensor reading against a
    specified limit.
        FAILURE: measured distance is less than the specified limit or no
                 distance sensor
        SUCCESS: measured distance is greater than the specified limit

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    sensor_id : int
        Distance sensor id

    clearance : float
        Distance limit in metres

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, sensor_id, clearance):
        super(CheckDistance, self).__init__(
            "Distance {} > {} ?".format(sensor_id, clearance))
        self._veh = vehicle
        self._id = sensor_id
        self._clearance = clearance

    def update(self):
        current_dist = self._veh.distance_sensors[self._id].current_distance
        if current_dist is None:
            self.feedback_message = ('Nothing from sensor {}'.format(self._id))
            return py_trees.common.Status.FAILURE
        elif current_dist/100. > self._clearance:
            self.feedback_message = 'Yes, it''s {}'.format(current_dist/100.)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}'.format(current_dist/100.)
            return py_trees.common.Status.FAILURE


class CheckEKF(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks EKF health status using dronekit ekf_ok
    method
        FAILURE: EKF status is not acceptable
        SUCCESS: EKF status is considered Healthy

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle):
        super(CheckEKF, self).__init__("EKF healthy ?")
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.ekf_ok:
            self.feedback_message = 'Healthy'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'Bad EKF'
            return py_trees.common.Status.FAILURE


class CheckCounter(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks waypoint index against specified number
        FAILURE: WP index does not match the entry
        SUCCESS: WP index matches the entry

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wpn : int
        The index of the waypoint in question.

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, wpn):
        super(CheckCounter, self).__init__(f"Is Counter {wpn}?")
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        next_wp = self._vehicle.commands.next
        if self._wpn == next_wp:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"No, it's {next_wp}"
            return py_trees.common.Status.FAILURE


class CheckCounterLessThan(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks waypoint index against specified number
        FAILURE: WP index is greater than equal to the entry
        SUCCESS: WP index is less than the entry

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wpn : int
        The index of the waypoint in question.

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, wpn):
        super(CheckCounterLessThan, self).__init__(f"Check Counter < {wpn}")
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        next_wp = self._vehicle.commands.next
        if next_wp < self._wpn:
            self.feedback_message = f"Yes, it's {next_wp}."
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"No, it's {next_wp}."
            return py_trees.common.Status.FAILURE


class WaitForWaypoint(py_trees.behaviour.Behaviour):
    """
    Condition leaf node to wait for arrival at specified waypoint:
        RUNNING: WP index matches the entry, the drone is flying towards WP
        SUCCESS: WP reached matches the entry, the drone is at the specified WP
        FAILURE: Otherwise

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wpn : int
        The index of the waypoint in question.

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, wpn):
        super(WaitForWaypoint, self).__init__("Is vehicle at\n"
                                              f"waypoint {wpn} ?")
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        next_wp = self._vehicle.commands.next
        if self._wpn == next_wp:
            self.feedback_message = 'Flying towards it'
            return py_trees.common.Status.RUNNING
        elif self._vehicle.mission_item_reached == self._wpn:
            self.feedback_message = f'Yes, reached waypoint {self._wpn}'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"No, it's {next_wp}"
            return py_trees.common.Status.FAILURE


class SetCounter(py_trees.behaviour.Behaviour):
    """
    Action leaf node that sets the waypoint index to specified. Note that only
    forward movement of the counter is permitted.
        FAILURE: WP index is greater than equal to the entry
        SUCCESS: WP index is set to desired

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wpn : int
        The index of the waypoint in question.

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, wpn):
        super(SetCounter, self).__init__("Set WP Counter to %i" % int(wpn))
        self._vehicle = vehicle
        self._wpn = int(wpn)

    def update(self):
        cur_wpn = self._vehicle.commands.next
        if cur_wpn < self._wpn:
            self.feedback_message = ('Advancing WP counter from {} to {}'
                                     .format(cur_wpn, self._wpn))
            print(self.feedback_message)
            self._vehicle.commands.next = self._wpn
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'Cannot move WP counter backwards from '\
                                    '{} to {}'.format(cur_wpn, self._wpn)
            return py_trees.common.Status.FAILURE


class TriggerCamera(py_trees.behaviour.Behaviour):
    """
    Action leaf node that takes a picture using the DO_DIGICAM_CONTROL command
    and returns SUCCESS. Note that camera must be setup according to following:
    https://ardupilot.org/copter/docs/common-cameras-and-gimbals.html

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle):
        super(TriggerCamera, self).__init__("Trigger Camera")
        self._vehicle = vehicle

    def update(self):
        self._vehicle.message_factory.digicam_control_send(0, 0, 0, 0, 0, 0, 1,
                                                           0, 0, 0)
        return py_trees.common.Status.SUCCESS


class CheckMode(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle flight mode is in agreement with
    entry.
        FAILURE: flight mode does not match entry
        SUCCESS: flight mode matches entry

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    mode_name : str
        Name of the mode in question

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckMode, self).__init__("Is mode %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        current_mode = self._vehicle.mode.name
        if current_mode == self._mode_name:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}'.format(current_mode)
            return py_trees.common.Status.FAILURE


class CheckModeNot(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle flight mode is not in agreement
    with entry.
        FAILURE: flight mode matches entry
        SUCCESS: flight mode does not match entry

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    mode_name : str
        Name of the mode in question

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckModeNot, self).__init__("Is mode NOT %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        current_mode = self._vehicle.mode.name
        if current_mode == self._mode_name:
            self.feedback_message = 'Fail: mode is {}'.format(current_mode)
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = 'OK, it''s {}'.format(current_mode)
            return py_trees.common.Status.SUCCESS


class IsArmable(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle is armable. This is based on
    dronekit is_armable attribute that wraps a number of pre-arm checks,
    ensuring that the vehicle has booted, has a good GPS fix, and that the EKF
    pre-arm is complete.
        FAILURE: otherwise
        SUCCESS: vehicle has booted, GPS and EKF are converged

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle):
        super(IsArmable, self).__init__('Can drone be armed ?')
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.is_armable:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No'
            return py_trees.common.Status.FAILURE


class IsArmed(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks if vehicle is armed
        FAILURE: vehicle is not armed
        SUCCESS: vehicle is armed

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle):
        super(IsArmed, self).__init__('Is drone armed ?')
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.armed:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No'
            return py_trees.common.Status.FAILURE


# class ArmDrone(py_trees.behaviour.Behaviour):

#     def __init__(self, vehicle):
#         super(ArmDrone, self).__init__('Arm')
#         self._vehicle = vehicle

#     def update(self):
#         self._vehicle.armed=True
#         return py_trees.common.Status.SUCCESS


# TODO return failure if mode is not GUIDED and not armed
class SimpleTakeoff(py_trees.behaviour.Behaviour):
    """
    Action leaf node for taking off to a specified altitude expressed in
    relative frame.
    <https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_RELATIVE_ALT>
    The vehicle must be in GUIDED mode and armed before this is called. Note
    that this behaviour should only be used on copter vehicles.
        SUCCESS: when executed

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    altitude : int
        Demanded take-off altitude in metres

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, altitude):
        super(SimpleTakeoff, self).__init__(f'Take off {altitude}')
        self._vehicle = vehicle
        self._altitude = altitude

    def update(self):
        self._vehicle.simple_takeoff(self._altitude)
        self.feedback_message = f'Takeoff and hold {self._altitude}'
        return py_trees.common.Status.SUCCESS


#class PlaySound(py_trees.behaviour.Behaviour):
#
#    def __init__(self, msg, voiceAst, returnFailure=None):
#        super(PlaySound, self).__init__("Play sound %s" % msg)
#        self._msg = msg
#        self._voiceAst = voiceAst
#        self._returnFailure = returnFailure
#
#    def update(self):
#        print("[WarningSound::update] Adding: \"" + self._msg + "\"")
#        # Adding message to the Voice Assistant queue
#        if self._voiceAst:
#            self._voiceAst.add_say(self._msg)
#        else:
#            print("Muted")
#        if self._returnFailure:
#            return py_trees.common.Status.FAILURE
#        else:
#            return py_trees.common.Status.SUCCESS


class AltGlobalAbove(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle is flying above a specified MSL
    altitude.
        FAILURE: flying below reference altitude
        SUCCESS: flying above reference altitude

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    altitude : int
        Altitude in question in metres

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, altitude):
        super(AltGlobalAbove, self).__init__(f'Over {altitude} (global)')
        self._vehicle = vehicle
        self._altitude = altitude

    def update(self):
        if self._vehicle.location.global_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class AltLocalAbove(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle is flying above a specified
    altitude expressed in relative frame.
    <https://mavlink.io/en/messages/common.html#MAV_FRAME_GLOBAL_RELATIVE_ALT>
        FAILURE: flying below reference altitude
        SUCCESS: flying above reference altitude

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    altitude : int
        Altitude in question in metres

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, altitude):
        super(AltLocalAbove, self).__init__(f'Over {altitude} (local)')
        self._vehicle = vehicle
        self._altitude = altitude

    def update(self):
        if self._vehicle.location.global_relative_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class BatteryLevelAbove(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle battery level is above a specified
    limit. A calibrated current sensor is needed for this behaviour.
        FAILURE: battery level is below reference percentage
        SUCCESS: battery level is above reference percentage

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    level : int
        battery percentage in question

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, level):
        super(BatteryLevelAbove, self).__init__(f"Battery over {level}% ?")
        self._vehicle = vehicle
        self._level = level

    def update(self):
        if self._vehicle.battery.level >= self._level:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth`
    and `dEast` metres from the specified `original_location`. The returned
    LocationGlobal has the same `alt` value as `original_location`.

    The function is useful when you want to move the vehicle around specifying
    locations relative to the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km)
    except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180.))
    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180./math.pi)
    newlon = original_location.lon + (dLon * 180./math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon,
                                                original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    return targetlocation


class MoveDrone(py_trees.behaviour.Behaviour):
    """
    Action leaf node for moving the vehicle relative to its current location.
    Returns SUCCESS when ticked.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    dNorth : int or float
        Distance north in m

    dEast : int or float
        Distance east in m

    dDown : int or float
        Distance down in m

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

"""
    def __init__(self, vehicle, dNorth, dEast, dDown):
        super(MoveDrone, self).__init__("Move (%f,%f,%f) NED" % (dNorth, dEast,
                                                                 dDown))
        self._vehicle = vehicle
        self._dNorth = dNorth
        self._dEast = dEast
        self._dDown = dDown

    def update(self):
        current_loc = self._vehicle.location.global_frame
        target_loc = get_location_metres(current_loc, self._dNorth,
                                         self._dEast)
        target_loc.alt = target_loc.alt - self._dDown
        self._vehicle.simple_goto(target_loc)
        return py_trees.common.Status.SUCCESS


class LatSpeedUnder(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks vehicle 2d speed is below a specified limit
        FAILURE: vehicle is flying above the speed limit
        SUCCESS: vehicle is flying below the speed limit

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    max_speed : float
        Speed limit in m/s

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, max_speed):
        super(LatSpeedUnder, self).__init__(f"Lat speed < {max_speed}?")
        self._vehicle = vehicle
        self._max_speed = max_speed

    def update(self):
        (vx, vy, vz) = self._vehicle.velocity
        current_speed = math.sqrt(vx*vx + vy*vy)
        if current_speed < self._max_speed:
            self.feedback_message = f'Yes, speed is {current_speed}'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f'No, speed is {current_speed}'
            return py_trees.common.Status.FAILURE


class CheckRCSwitch(py_trees.behaviour.Behaviour):
    """
    Condition leaf node that checks the status of a momentary switch
    (i.e. push switch)
        SUCCESS: The switch is on (activated or pushed)
        FAILURE: The switch is off or no data available

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    channel : The channel allocated to the switch

    on_pwm_limit : The PWM limit in which the switch is considered ON in µs

    Returns
    -------
    node : py_trees.common.Status
        Status of the leaf node behaviour

    """
    def __init__(self, vehicle, channel, on_pwm_limit):
        super(CheckRCSwitch, self).__init__("Is Switch ON ?")
        self._vehicle = vehicle
        self._channel = channel
        self._on = on_pwm_limit

    def update(self):
        if self._vehicle.rc_channels.chan_raw[self._channel] > self._on:
            self.feedback_message = "Yes, it's ON ___"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "No, it's OFF _/ _"
            return py_trees.common.Status.FAILURE
