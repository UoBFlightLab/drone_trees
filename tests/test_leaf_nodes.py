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

test_leaf_nodes.py:

Unit test for all leaf nodes

"""
###############################################################################

import sys
from time import sleep
from os.path import join, abspath
from dronekit import connect, VehicleMode, LocationGlobal
from py_trees.common import Status
from drone_trees.drone_tree_vehicle import DroneTreeVehicle
from drone_trees import leaf_nodes as lf


def test_CheckGPS(copter_sitl_ground):
    """Verify that CheckGPS behaviour responds SUCCESS for GPS status above
    the limit and FAILURE for below and equal to the limit"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)
    # Set GPS status to 3D Fix (status = 3)
    vehicle.parameters['SIM_GPS_TYPE'] = 2
    sleep(0.5)                        # wait for parameter change to take place

    """-----  Expect SUCCESS -----"""

    # GPS status above limit
    gps_check_above = lf.CheckGPS(vehicle, 2)    # check with limit 2D fix (=2)
    gps_check_above.tick_once()                  # tick behaviour to get status
    assert gps_check_above.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # GPS status below limit
    gps_check_below = lf.CheckGPS(vehicle, 5)  # check with limit RTK Float(=5)
    gps_check_below.tick_once()                  # tick behaviour to get status
    assert gps_check_below.status == Status.FAILURE

    # GPS status equals to limit
    gps_check_equal = lf.CheckGPS(vehicle, 3)    # check with limit 3D Fix (=3)
    gps_check_equal.tick_once()                  # tick behaviour to get status
    assert gps_check_equal.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_CheckEKF(copter_sitl_ground):
    """Verify that CheckEKF behaviour responds SUCCESS for a healthy EKF status
    and FAILURE for bad EKF status"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    """-----  Expect FAILURE -----"""

    # Bad EKF status at start-up
    ekf_check_bad = lf.CheckEKF(vehicle)         # checking EKF at start-up
    ekf_check_bad.tick_once()                    # tick behaviour to get status
    assert ekf_check_bad.status == Status.FAILURE

    """-----  Expect SUCCESS -----"""

    # Wait until healthy EKF
    while not vehicle.is_armable:
        sleep(0.5)

    # Healthy EKF status
    ekf_check_healthy = lf.CheckEKF(vehicle)
    ekf_check_healthy.tick_once()                # tick behaviour to get status
    assert ekf_check_healthy.status == Status.SUCCESS

    vehicle.close()                              # close local vehicle instance


def test_CheckMode_CheckModeNot(copter_sitl_ground):
    """
    Verify CheckMode and CheckModeNot behaviours respond as follows:
    CheckMode
        FAILURE: flight mode does not match entry
        SUCCESS: flight mode matches entry

    CheckModeNot
        FAILURE: flight mode matches entry
        SUCCESS: flight mode does not match entry
    """

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    """-----  Expect FAILURE -----"""

    # Vehicle mode is set to STABILIZE at start-up
    check_mode_false = lf.CheckMode(vehicle, 'AUTO')    # check if mode is AUTO
    check_mode_false.tick_once()                 # tick behaviour to get status
    assert check_mode_false.status == Status.FAILURE

    # Check that mode is not STABILIZE
    check_mode_not_false = lf.CheckModeNot(vehicle, 'STABILIZE')
    check_mode_not_false.tick_once()             # tick behaviour to get status
    assert check_mode_not_false.status == Status.FAILURE

    """-----  Expect SUCCESS -----"""

    # Vehicle mode is set to STABILIZE at start-up
    check_mode_true = lf.CheckMode(vehicle, 'STABILIZE')      # check STABILIZE
    check_mode_true.tick_once()                  # tick behaviour to get status
    assert check_mode_true.status == Status.SUCCESS

    # Check that mode is not AUTO
    check_mode_not_true = lf.CheckModeNot(vehicle, 'AUTO')
    check_mode_not_true.tick_once()              # tick behaviour to get status
    assert check_mode_not_true.status == Status.SUCCESS

    vehicle.close()                              # close local vehicle instance


def test_IsArmable_IsArmed(copter_sitl_ground):
    """
    Verify IsArmable and IsArmed behaviours respond as follows:
    IsArmable
        FAILURE: otherwise
        SUCCESS: vehicle has booted, GPS and EKF are converged

    IsArmed
        FAILURE: vehicle is not armed
        SUCCESS: vehicle is armed
    """

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    """-----  Expect FAILURE -----"""

    # Vehicle is not armable at start-up
    is_armable_false = lf.IsArmable(vehicle)     # check if vehicle is armable
    is_armable_false.tick_once()                 # tick behaviour to get status
    assert is_armable_false.status == Status.FAILURE

    # Vehicle is disarmed at start-up
    is_armed_false = lf.IsArmed(vehicle)         # check if vehicle is armed
    is_armed_false.tick_once()                   # tick behaviour to get status
    assert is_armed_false.status == Status.FAILURE

    """-----  Expect SUCCESS -----"""

    # Wait until armable
    while not vehicle.is_armable:
        sleep(0.5)

    is_armable_true = lf.IsArmable(vehicle)      # check if vehicle is armable
    is_armable_true.tick_once()                  # tick behaviour to get status
    assert is_armable_true.status == Status.SUCCESS

    vehicle.arm()  # arm vehicle
    vehicle.parameters['SIM_SPEEDUP'] = 1        # slow-down to real-time

    is_armed_true = lf.IsArmed(vehicle)          # check if vehicle is armed
    is_armed_true.tick_once()                    # tick behaviour to get status
    assert is_armed_true.status == Status.SUCCESS

    vehicle.close()                              # close local vehicle instance


def test_CheckDistance(copter_sitl_ground):
    """Verify that CheckDistance behaviour responds SUCCESS for distance sensor
    reading above the limit and FAILURE for below and equal to the limit"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(),
                      vehicle_class=DroneTreeVehicle)
    # Set distance sensor reading to 5m
    vehicle.parameters['RNGFND2_SCALING'] = 1
    sleep(0.5)                        # wait for parameter change to take place

    """-----  Expect FAILURE -----"""

    # Distance sensor reading None
    distance_check_none = lf.CheckDistance(vehicle, 0, 10.)  # check limit 10m
    distance_check_none.tick_once()              # tick behaviour to get status
    assert distance_check_none.status == Status.FAILURE
    assert distance_check_none.feedback_message == 'Nothing from sensor 0'

    # Distance sensor reading below limit
    distance_check_below = lf.CheckDistance(vehicle, 1, 10.)  # check limit 10m
    distance_check_below.tick_once()             # tick behaviour to get status
    distance_check_below.tick_once()             # tick behaviour to get status
    assert distance_check_below.status == Status.FAILURE
    assert distance_check_below.feedback_message != 'Nothing from sensor 1'

    # Distance sensor reading equals to limit
    distance_check_equal = lf.CheckDistance(vehicle, 1, 5.)    # check limit 5m
    distance_check_equal.tick_once()             # tick behaviour to get status
    distance_check_equal.tick_once()             # tick behaviour to get status
    assert distance_check_equal.status == Status.FAILURE
    assert distance_check_equal.feedback_message != 'Nothing from sensor 1'

    """-----  Expect SUCCESS -----"""

    # Distance sensor reading above limit
    distance_check_above = lf.CheckDistance(vehicle, 1, 2.)    # check limit 2m
    distance_check_above.tick_once()             # tick behaviour to get status
    assert distance_check_above.status == Status.SUCCESS
    assert distance_check_above.feedback_message != 'Nothing from sensor 1'

    vehicle.close()                              # close local vehicle instance


def test_AltAbove(copter_sitl_guided_to):
    """Verify AltLocalAbove (AGL) and AltGlobalAbove (MSL) behaviours respond
    SUCCESS to current altitude above reference and FAILURE for below
    reference """

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_guided_to.connection_string(),
                      wait_ready=True)

    # find current MSL alttitude
    current_global_alt = vehicle.location.global_frame.alt

    """-----  Expect SUCCESS -----"""

    # Local altitude above reference
    alt_local_above_true = lf.AltLocalAbove(vehicle, 5)    # check alt above 5m
    alt_local_above_true.tick_once()             # tick behaviour to get status
    assert alt_local_above_true.status == Status.SUCCESS

    #  Global altitude above reference (input 5m below current alt)
    alt_global_above_true = lf.AltGlobalAbove(vehicle, current_global_alt-5)
    alt_global_above_true.tick_once()            # tick behaviour to get status
    assert alt_global_above_true.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # Local altitude below reference
    alt_local_above_false = lf.AltLocalAbove(vehicle, 15)  # check above 15m
    alt_local_above_false.tick_once()            # tick behaviour to get status
    assert alt_local_above_false.status == Status.FAILURE

    # Global altitude below reference (input 5m above current alt)
    alt_global_above_false = lf.AltGlobalAbove(vehicle, current_global_alt+5)
    alt_global_above_false.tick_once()           # tick behaviour to get status
    assert alt_global_above_false.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_BatteryLevelAbove(copter_sitl_guided_to):
    """Verify BatteryLevelAbove behaviour responds SUCCESS to battery level
    above reference percentage and FAILURE for below reference percentage"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_guided_to.connection_string(),
                      wait_ready=True)

    """-----  Expect SUCCESS -----"""

    # Battery level above limit
    bat_above_true = lf.BatteryLevelAbove(vehicle, 80)    # check bat above 80%
    bat_above_true.tick_once()                   # tick behaviour to get status
    assert bat_above_true.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # wait until battery level is below 98%
    while not vehicle.battery.level < 98:
        sleep(0.5)

    # Battery level below limit
    bat_above_false = lf.BatteryLevelAbove(vehicle, 98)   # check bat above 98%
    bat_above_false.tick_once()                  # tick behaviour to get status
    assert bat_above_false.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_LatSpeedUnder(copter_sitl_guided_to):
    """Verify LatSpeedUnder behaviour responds SUCCESS when speed is below
    reference value and FAILURE when above reference value"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_guided_to.connection_string(),
                      wait_ready=True)

    """-----  Expect SUCCESS -----"""

    # Speed below limit, vehicle is hovering after take-off
    speed_below_true = lf.LatSpeedUnder(vehicle, 2)    # check speed below 2m/s
    speed_below_true.tick_once()                 # tick behaviour to get status
    assert speed_below_true.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # Raise speed by commanding an arbitrary location
    a_location = LocationGlobal(-34.364114, 149.166022, 30)
    vehicle.simple_goto(a_location)
    sleep(1)  # wait for vehicle to start moving

    # Speed above limit, vehicle is flying to an arbitrary location
    speed_below_false = lf.LatSpeedUnder(vehicle, 2)   # check speed below 2m/s
    speed_below_false.tick_once()                # tick behaviour to get status
    assert speed_below_false.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_CheckCounter(copter_sitl_auto_to):
    """Verify CheckCounter and CheckCounterLessThan behaviours respond as
    follows:
    CheckCounter
        FAILURE: WP index does not match the entry
        SUCCESS: WP index matches the entry

    CheckCounterLessThan
        FAILURE: WP index is greater than equal to the entry
        SUCCESS: WP index is less than the entry
    """

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_auto_to.connection_string(), wait_ready=True)

    """-----  Expect SUCCESS -----"""

    vehicle.commands.next = 3  # set next waypoint counter to 3
    sleep(0.5)

    # Waypoint counter equals to entry
    ctr_equal = lf.CheckCounter(vehicle, 3)      # check waypoint counter is 3
    ctr_equal.tick_once()                        # tick behaviour to get status
    assert ctr_equal.status == Status.SUCCESS

    # Waypoint counter below entry
    ctr_lt_below = lf.CheckCounterLessThan(vehicle, 5)      # check ctr below 5
    ctr_lt_below.tick_once()                     # tick behaviour to get status
    assert ctr_lt_below.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # waypoint counter above than entry
    ctr_above = lf.CheckCounter(vehicle, 9)      # check waypoint counter is 9
    ctr_above.tick_once()                        # tick behaviour to get status
    assert ctr_above.status == Status.FAILURE

    # Waypoint counter above entry
    ctr_lt_above = lf.CheckCounterLessThan(vehicle, 2)      # check ctr below 2
    ctr_lt_above.tick_once()                     # tick behaviour to get status
    assert ctr_lt_above.status == Status.FAILURE

    # waypoint counter below than entry
    ctr_below = lf.CheckCounter(vehicle, 2)      # check waypoint counter is 2
    ctr_below.tick_once()                        # tick behaviour to get status
    assert ctr_below.status == Status.FAILURE

    # Waypoint counter equals to entry
    ctr_lt_equal = lf.CheckCounterLessThan(vehicle, 3)      # check ctr below 3
    ctr_lt_equal.tick_once()                     # tick behaviour to get status
    assert ctr_lt_equal.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_SetCounter(copter_sitl_auto_to):
    """Verify that SetCounter behaviour sets waypoint counter to the desired
    index. Also check that it responds as follows:
        FAILURE: WP index is greater than equal to the entry
        SUCCESS: WP index is set to desired
    """

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_auto_to.connection_string(), wait_ready=True)

    """-----  Expect SUCCESS -----"""

    # set ctr to above current ctr
    des_ctr_above = 3  # desired waypoint index
    set_ctr_above = lf.SetCounter(vehicle, des_ctr_above)    # set counter to 3
    set_ctr_above.tick_once()              # tick behaviour to get status
    sleep(0.5)
    assert des_ctr_above == vehicle.commands.next       # check desired ctr set
    assert set_ctr_above.status == Status.SUCCESS

    """-----  Expect FAILURE -----"""

    # set ctr to below current ctr
    des_ctr_below = 2                            # desired waypoint index
    set_ctr_below = lf.SetCounter(vehicle, des_ctr_below)    # set counter to 2
    set_ctr_below.tick_once()                    # tick behaviour to get status
    assert des_ctr_below != vehicle.commands.next   # check desired ctr not set
    assert set_ctr_below.status == Status.FAILURE

    # set ctr to current ctr
    des_ctr_current = vehicle.commands.next            # desired waypoint index
    set_ctr_current = lf.SetCounter(vehicle, des_ctr_current)
    set_ctr_current.tick_once()                  # tick behaviour to get status
    assert set_ctr_current.status == Status.FAILURE

    vehicle.close()                              # close local vehicle instance


def test_TriggerCamera(copter_sitl_ground):
    """Verify that TriggerCamera behaviour captures an image and responds
    SUCCESS"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    #"""-----  Expect SUCCESS -----"""

    ekf_check_healthy = lf.TriggerCamera(vehicle)              # trigger camera
    ekf_check_healthy.tick_once()                # tick behaviour to get status

    @vehicle.on_message('CAMERA_FEEDBACK')
    def listener(self, name, msg):
        # Check single image captured
        assert msg.img_idx == 1
    
    assert ekf_check_healthy.status == Status.SUCCESS

    vehicle.close()                              # close local vehicle instance

def test_SetParam(copter_sitl_ground):
    """Verify that the SetParam behaviour changes parameter value and responds
    SUCCESS"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    vehicle.parameters['SIM_SPEEDUP'] = 5
    sleep(1)

    """-----  Expect SUCCESS -----"""

    des_param = 'SIM_GPS_NUMSATS'  # desired parameter to change
    des_value = 15                 # new value for the desired parameter
    # Check that the new value is different
    assert vehicle.parameters['SIM_GPS_NUMSATS'] != des_value
    # Set new value for the desired parameter
    set_param = lf.SetParam(vehicle, des_param, des_value)
    set_param.tick_once()          # tick behaviour to get status
    # Check desired param changed to the new value
    assert vehicle.parameters['SIM_GPS_NUMSATS'] == des_value
    # Check response
    assert set_param.status == Status.SUCCESS

    vehicle.close()                # close local vehicle instance


def test_Land(copter_sitl_guided_to):
    """Verify that the Land behaviour results in a mode change to
    Return-to-Launch and responds SUCCESS after execution"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_guided_to.connection_string(),
                      wait_ready=True)

    """-----  Expect SUCCESS -----"""

    assert vehicle.mode != 'RTL'             # check current mode is not RTL
    land = lf.Land(vehicle)                  # command land by RTL
    land.tick_once()                         # tick behaviour to get status
    sleep(0.5)                               # wait for the change to happen
    assert vehicle.mode == 'RTL'             # check mode changed to RTL
    assert land.status == Status.SUCCESS  # Check response

    vehicle.close()                          # close local vehicle instance


def test_SimpleTakeoff(copter_sitl_ground):
    """Verify that SimpleTakeoff behaviour results in takeoff to desired
    altitude and responds SUCCESS after execution"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    # wait until armable
    while not vehicle.is_armable:
        sleep(0.5)

    vehicle.parameters['SIM_SPEEDUP'] = 5    # set sim speed for rapid take-off
    vehicle.arm()                            # arm vehicle
    vehicle.mode = VehicleMode('GUIDED')     # set vehicle mode to GUIDED

    # should still be on the ground
    assert vehicle.location.global_relative_frame.alt < 0.5
    assert vehicle.is_armable
    simple_to = lf.SimpleTakeoff(vehicle, 5)  # command takeoff to 5m AGL
    simple_to.tick_once()                     # tick behaviour to get status
    sleep(1)                                  # wait for the takeoff to happen
    assert vehicle.location.global_relative_frame.alt > 4.5    # alt above 4.5m
    assert vehicle.location.global_relative_frame.alt < 5.5    # alt below 5.5m
    assert simple_to.status == Status.SUCCESS  # Check response

    vehicle.close()                           # close local vehicle instance


def test_MoveDrone(copter_sitl_guided_to):
    """Verify the move specified by the MoveDrone behaviour and observe a
    SUCCESS response"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_guided_to.connection_string(),
                      wait_ready=True)

    start_loc = vehicle.location.global_relative_frame  # reference start loc
    # Reference target location
    target_loc = lf.get_location_metres(start_loc, 20, 20)
    target_loc.alt = vehicle.location.global_relative_frame.alt - 5

    # Move drone 20m North, 20m East and 5m down
    move_drone = lf.MoveDrone(vehicle, 20, 20, 5)
    move_drone.tick_once()                   # tick behaviour to get status
    sleep(5)
    # Check final location against target
    final_loc = vehicle.location.global_relative_frame
    assert round(final_loc.lat, 5) == round(target_loc.lat, 5)
    assert round(final_loc.lon, 5) == round(target_loc.lon, 5)
    assert round(final_loc.alt) == round(target_loc.alt)

    # Check response
    assert move_drone.status == Status.SUCCESS

    vehicle.close()                          # close local vehicle instance


def test_MissionUpload(copter_sitl_ground):
    """Verify that a mission has been uploaded to the vehicle"""

    # Make a local vehicle instance
    vehicle = connect(copter_sitl_ground.connection_string(), wait_ready=True)

    # load an arbitrary mission mission file
    from pymavlink.mavwp import MAVWPLoader
    mission = MAVWPLoader()
    sitl_path = join(abspath(sys.path[0]), '..', 'examples', 'bridge')
    mission_filename = join(sitl_path, 'executable_mission.txt')
    mission.load(mission_filename)
    # bit of a hack: fix the sequence number
    for w in mission.wpoints:
        w.seq = w.seq+1
    cmds = vehicle.commands
    cmds.clear()
    wplist = mission.wpoints[:]

    """-----  Expect SUCCESS -----"""

    # make sure disarmed
    if vehicle.armed:
        vehicle.disarm()

    # Upload mission when disarmed
    mission_upload_disarmed = lf.MissionUpload(vehicle, wplist)
    mission_upload_disarmed.tick_once()          # tick behaviour to get status
    # check disarmed
    assert not vehicle.armed
    # check response
    assert mission_upload_disarmed.status == Status.SUCCESS
    # check number of waypoints on-board against the mission file
    assert cmds.count == len(wplist)

    """-----  Expect FAILURE -----"""

    cmds.clear()                                 # clear mission
    while not vehicle.is_armable:                # wait until armable
        sleep(0.5)
    vehicle.parameters['SIM_SPEEDUP'] = 5        # slow down SITL
    vehicle.armed = True                         # arm vehicle
    sleep(0.5)

    # Upload mission when armed
    mission_upload_armed = lf.MissionUpload(vehicle, wplist)
    mission_upload_armed.tick_once()             # tick behaviour to get status
    # check response
    assert mission_upload_armed.status == Status.FAILURE
    # check no mission on-board
    assert vehicle.commands.count < 2

    vehicle.close()                              # close local vehicle instance
