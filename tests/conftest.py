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

conftest.py:

Provides pytest fixtures for use in the testing framework. Fixutres include
Ardupilot SITL instances in the following states:

    1) Ground startup
    2) Flying in AUTO mode
    3) Take-off in GUIDED

"""
###############################################################################

import pytest
import sys
from os.path import join, abspath
from time import sleep
from dronekit_sitl import SITL
from dronekit import connect, VehicleMode
from pymavlink.mavwp import MAVWPLoader
from drone_trees.drone_tree_vehicle import DroneTreeVehicle
from drone_trees.sitl import sitl_file_path

# Find SITL directory
sitl_filename = sitl_file_path()
if sitl_filename is None:
    sys.exit(1)

@pytest.fixture
def copter_sitl_ground():
    """
    Returns a copter SITL instance for use in test cases
    Status: on ground
    Mode: Stabilize
    """

    # Launch a SITL using local copy of Copter 4
    sitl = SITL(sitl_filename, instance=0)
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    print(sitl.connection_string())

    # Connect to UAV to setup base parameters
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    veh.parameters['SIM_SONAR_SCALE'] = 0.00001
    veh.parameters['RNGFND2_ORIENT'] = 0
    veh.parameters['RNGFND2_SCALING'] = 10
    veh.parameters['RNGFND2_PIN'] = 0
    veh.parameters['RNGFND2_TYPE'] = 1
    veh.parameters['RNGFND2_MAX_CM'] = 5000
    veh.parameters['RNGFND2_MIN_CM'] = 5000

    veh.parameters['SIM_BATT_VOLTAGE'] = 12.59
    veh.parameters['BATT_MONITOR'] = 4

    veh.parameters['TERRAIN_ENABLE'] = 0

    veh.parameters['ARMING_CHECK'] = 16384    # mission only
    veh.parameters['SIM_SPEEDUP'] = 10        # speed up SITL for rapid startup
    veh.parameters['FRAME_CLASS'] = 2         # I'm a hex
    veh.close()          # close veh instance after setting base parameters

    # Stop and relaunch SITL to enable distance sensor and battery monitor
    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    yield sitl
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()


@pytest.fixture
def copter_sitl_guided_to():
    """
    Returns a copter SITL instance for use in test cases
    Status: hovering at 10m AGL
    Mode: Guided
    """

    # Launch a SITL using local copy of Copter 4
    sitl = SITL(sitl_filename, instance=0)
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    # Connect to UAV to setup base parameters
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    veh.parameters['SIM_SONAR_SCALE'] = 0.00001
    veh.parameters['RNGFND2_ORIENT'] = 0
    veh.parameters['RNGFND2_SCALING'] = 10
    veh.parameters['RNGFND2_PIN'] = 0
    veh.parameters['RNGFND2_TYPE'] = 1
    veh.parameters['RNGFND2_MAX_CM'] = 5000
    veh.parameters['RNGFND2_MIN_CM'] = 5000

    veh.parameters['SIM_BATT_VOLTAGE'] = 12.59
    veh.parameters['BATT_MONITOR'] = 4

    veh.parameters['TERRAIN_ENABLE'] = 0

    veh.parameters['ARMING_CHECK'] = 16384    # mission only
    veh.parameters['SIM_SPEEDUP'] = 10        # speed up SITL for rapid startup
    veh.parameters['FRAME_CLASS'] = 2         # I'm a hex
    veh.close()          # close vehicle instance after setting base parameters

    # Stop and relaunch SITL to enable distance sensor and battery monitor
    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    # Connect again
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)
    # wait until armable
    while not veh.is_armable:
        sleep(0.5)

    veh.parameters['SIM_SPEEDUP'] = 5    # set sim speed for rapid take-off
    veh.arm()                            # arm veh
    veh.mode = VehicleMode('GUIDED')     # set veh mode to GUIDED
    veh.wait_simple_takeoff(10)          # take-off to 10m AGL
    veh.close()                          # close vehicle after guided takeoff

    yield sitl
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()


@pytest.fixture
def copter_sitl_auto_to():
    """
    Returns a copter SITL instance for use in test cases
    Status: starting auto takeoff(to 20m AGL), 13 waypoints
    Mode: Auto
    """

    # Launch a SITL using local copy of Copter 4
    sitl = SITL(sitl_filename, instance=0)
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    # Connect to UAV to setup base parameters
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    veh.parameters['SIM_SONAR_SCALE'] = 0.00001
    veh.parameters['RNGFND2_ORIENT'] = 0
    veh.parameters['RNGFND2_SCALING'] = 10
    veh.parameters['RNGFND2_PIN'] = 0
    veh.parameters['RNGFND2_TYPE'] = 1
    veh.parameters['RNGFND2_MAX_CM'] = 5000
    veh.parameters['RNGFND2_MIN_CM'] = 5000

    veh.parameters['SIM_BATT_VOLTAGE'] = 12.59
    veh.parameters['BATT_MONITOR'] = 4

    veh.parameters['TERRAIN_ENABLE'] = 0

    veh.parameters['ARMING_CHECK'] = 16384    # mission only
    veh.parameters['SIM_SPEEDUP'] = 10        # speed up SITL for rapid startup
    veh.parameters['FRAME_CLASS'] = 2         # I'm a hex
    veh.close()          # close vehicle instance after setting base parameters

    # Stop and relaunch SITL to enable distance sensor and battery monitor
    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    # Wait for sitl to be ready before passing to test cases
    sitl.block_until_ready()

    # Connect again
    veh = connect(sitl.connection_string(),
                  wait_ready=True,
                  vehicle_class=DroneTreeVehicle)

    # upload an arbitrary mission
    mission = MAVWPLoader()
    filename = join(abspath(sys.path[0]), 'executable_mission.txt')
    mission.load(filename)
    # bit of a hack: fix the sequence number
    for w in mission.wpoints:
        w.seq = w.seq+1
    cmds = veh.commands
    cmds.clear()
    wplist = mission.wpoints[:]
    # add a dummy command on the front
    # just the first WP used as a placeholder
    # upload seems to skip the first WP
    cmds.add(wplist[0])
    for wp in wplist:
        cmds.add(wp)
    cmds.upload()

    # wait until armable
    while not veh.is_armable:
        sleep(0.5)

    veh.parameters['SIM_SPEEDUP'] = 5  # set sim speed for rapid take-off
    sleep(0.5)                        # wait for parameter change to take place
    veh.arm()                         # arm vehicle

    # Perform an auto take-off
    veh.mode = VehicleMode('AUTO')       # change mode to auto
    veh.channels.overrides['3'] = 1700   # raise throttle to confirm T/O
    # wait for take-off
    while veh.location.global_relative_frame.alt < 0.5:
        sleep(0.5)
    veh.close()              # close vehicle instance after auto takeoff

    yield sitl
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()
