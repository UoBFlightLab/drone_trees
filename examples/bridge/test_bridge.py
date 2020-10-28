# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 12:36:33 2020

@author: aeagr
"""

import platform
from time import sleep
from dronekit import connect, VehicleMode
from dronekit_sitl import SITL
from drone_trees.control_automaton import ControlAutomaton
from drone_trees.drone_tree_vehicle import DroneTreeVehicle
from fly_bridge import behaviour_tree


def start_sitl():
    """Launch a SITL using local copy of Copter 4,
    then set up the simulator for a rangefinder.
    Only works for Windows or Linux.  Including
    binary in the project is ugly, but only used
    for testing."""
    if platform.system()=='Linux':
        sitl_path='sitl/linux/arducopter'
    elif platform.system()=='Windows':
        sitl_path='sitl/windows/ArduCopter.exe'
    else:
        sitl_path=''
    sitl = SITL(sitl_path)
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

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

    veh.parameters['ARMING_CHECK'] = 16384 # mission only
    veh.parameters['FRAME_CLASS'] = 2 # I'm a hex

    veh.close()

    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    sitl.block_until_ready()

    return sitl


def test_sitl():
    """Test that the SITL launch function start_sitl works
    and that we can use it to access distance_sensors attributes"""
    sitl = start_sitl()
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    for ii in range(20):
        sleep(1)
        print(ii)
        print(veh.distance_sensors[1])
        print(veh.battery)
        if ii == 4:
            veh.parameters['SIM_BATT_VOLTAGE'] = 12.11
        rng_cm = (ii+10)*10.
        print(rng_cm)
        veh.parameters['SIM_SONAR_SCALE'] = 0.0238/rng_cm
    veh.close()
    sitl.stop()


def test_nominal(copter4=True):
    """Test nominal case: should complete whole mission"""
    ca = ControlAutomaton(behaviour_tree)

    if copter4:
        # start own sitl and get CA to connect to it
        # will be copter 4
        sitl = start_sitl()
        ca.startup(override_args=[sitl.connection_string()])
    else:
        # run CA with 'sitl' argument - it'll run its own through dronekit_sitl
        # will be copter 3.3
        sitl = None
        ca.startup(override_args=['sitl'])

    for ii in range(100):
        ca.tick()
        sleep(1)
        if ca.vehicle.is_armable:
            break
    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.5
    assert ca.vehicle.is_armable
    ca.vehicle.arm()
    for ii in range(3):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    # switch to auto and lift the throttle to trigger auto takeoff
    ca.vehicle.mode = VehicleMode('AUTO')
    ca.vehicle.channels.overrides['3'] = 1700
    # log as we visit each waypoint
    wp_log = {2: False,
              4: False,
              7: False,
              10: False,
              13: False,
              16: False,
              18: False,
              20: False}
    for ii in range(200):
        ca.tick()
        if ca.vehicle.commands.next in wp_log.keys():
            wp_log[ca.vehicle.commands.next] = True
        print(wp_log)
        if ca.finished():
            break
        sleep(1)
    # should be back on the ground at HOME
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    # should have visited everywhere
    for wpn in wp_log.keys():
        assert wp_log[wpn], 'Missed WP {}'.format(wpn)
    # close down ControlAutomaton
    ca.cleanup()
    if sitl:
        sitl.stop()
    print("Test passed")


def test_lowbat(copter4=True):
    """
    Test low battery case: Verify low battery safety behaviour. The vehicle is
    held at waypoint 2 in until battery level falls under 36% triggering
    a jump to safti response.
    """
    ca = ControlAutomaton(behaviour_tree)

    if copter4:
        # start own sitl and get CA to connect to it
        # will be copter 4
        sitl = start_sitl()
        ca.startup(override_args=[sitl.connection_string()])
    else:
        # run CA with 'sitl' argument - it'll run its own through dronekit_sitl
        # will be copter 3.3
        sitl = None
        ca.startup(override_args=['sitl'])
    # speed up simulation until vehicle is armable
    ca.vehicle.parameters['SIM_SPEEDUP'] = 5
    for ii in range(100):
        ca.tick()
        sleep(1)
        if ca.vehicle.is_armable:
            break
    # resume to normal speed
    ca.vehicle.parameters['SIM_SPEEDUP'] = 1

    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.5
    assert ca.vehicle.is_armable
    ca.vehicle.arm()
    for ii in range(3):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    # switch to auto and lift the throttle to trigger auto takeoff
    ca.vehicle.mode = VehicleMode('AUTO')
    ca.vehicle.channels.overrides['3'] = 1700
    # log as we visit each waypoint
    wp_log = {2: False,
              4: False,
              7: False,
              10: False,
              13: False,
              16: False,
              18: False,
              20: False}
    while not ca.finished():
        ca.tick()
        if ca.vehicle.commands.next in wp_log.keys():
            wp_log[ca.vehicle.commands.next] = True
        # hold vehicle at waypoint 2 as long as battery level is above 36%
        if ca.vehicle.commands.next >= 2 and ca.vehicle.battery.level > 36:
            ca.vehicle.commands.next = 2

        print(wp_log)
        sleep(1)
    # should skip waypoints 10 and 13
    assert not wp_log[13], 'Mission was not skipped'
    # should visit SAFTI
    assert wp_log[16], 'SAFTI was not visited'
    # should visit final waypoint
    assert wp_log[20], 'Mission was not completed'
    # should be back on the ground at HOME
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    # close down ControlAutomaton
    ca.cleanup()
    if sitl:
        sitl.stop()
    print("Test passed")


def test_avoidance():
    """
    Test collision avoidance case: Verify collision avoidance safety behaviour.
    The lidar reading at waypoint 7 is reduced below the safety limit of 2m
    triggering a jump to safti response.
    """
    ca = ControlAutomaton(behaviour_tree)

    # start own sitl and get CA to connect to it
    # will be copter 4 - distance sensor is not supported in 3.3
    sitl = start_sitl()
    ca.startup(override_args=[sitl.connection_string()])

    # speed up simulation until vehicle is armable
    ca.vehicle.parameters['SIM_SPEEDUP'] = 5
    for ii in range(100):
        ca.tick()
        sleep(1)
        if ca.vehicle.is_armable:
            break
    # resume to normal speed
    ca.vehicle.parameters['SIM_SPEEDUP'] = 1
    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.5
    assert ca.vehicle.is_armable
    ca.vehicle.arm()
    for ii in range(3):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    # switch to auto and lift the throttle to trigger auto takeoff
    ca.vehicle.mode = VehicleMode('AUTO')
    ca.vehicle.channels.overrides['3'] = 1700
    # log as we visit each waypoint
    wp_log = {2: False,
              4: False,
              7: False,
              10: False,
              13: False,
              16: False,
              18: False,
              20: False}
    for ii in range(200):
        ca.tick()
        if ca.vehicle.commands.next in wp_log.keys():
            wp_log[ca.vehicle.commands.next] = True
        # Simulate distance sensor reading
        # 20m at WP 7
        if ca.vehicle.commands.next == 7:
            ca.vehicle.parameters['RNGFND2_SCALING'] = 4

        # 1m injected at TENNA to trigger global collision avoidance check
        if ca.vehicle.commands.next == 10:
            ca.vehicle.parameters['RNGFND2_SCALING'] = 0.2

        # 5m at WP 13
        if ca.vehicle.commands.next == 13:
            ca.vehicle.parameters['RNGFND2_SCALING'] = 1

        # 50m for the remainder of the mission
        if ca.vehicle.commands.next > 13:
            ca.vehicle.parameters['RNGFND2_SCALING'] = 10
        print(wp_log)
        if ca.finished():
            break
        sleep(1)
    # should skip waypoints 10 and 13
    assert not wp_log[13], 'Mission was not skipped'
    # should visit SAFTI
    assert wp_log[16], 'SAFTI was not visited'
    # should visit final waypoint
    assert wp_log[20], 'Mission was not completed'
    # should be back on the ground at HOME
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    # close down ControlAutomaton
    ca.cleanup()
    if sitl:
        sitl.stop()
    print("Test passed")

if __name__ == '__main__':
    test_nominal()
    #test_nominal(copter4=False)
    #test_lowbat()
    #test_avoidance()
    #test_sitl()
