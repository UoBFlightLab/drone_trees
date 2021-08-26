# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 12:36:33 2020

@author: aeagr
"""

from fly_square import behaviour_tree
from drone_trees.control_automaton import ControlAutomaton
from drone_trees.sitl import start_sitl
from dronekit import VehicleMode
from time import sleep

def disp_from_home(veh):
    dlat = veh.location.global_relative_frame.lat \
                - veh.home_location.lat
    dlon = veh.location.global_relative_frame.lon \
                - veh.home_location.lon
    return (dlat,dlon)

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

    for ii in range(10):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    ca.vehicle.mode = VehicleMode('GUIDED')
    # wait for armable - can take a while
    for ii in range(100):
        ca.tick()
        sleep(1)
        if ca.vehicle.is_armable:
            break
    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    ca.vehicle.arm()
    for ii in range(200):
        ca.tick()
        if ca.finished():
            break
        sleep(1)
    # should be back on the ground in RTL at HOME
    dlat,dlon = disp_from_home(ca.vehicle)
    assert abs(dlat) < 1e-5
    assert abs(dlon) < 1e-5
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    assert ca.vehicle.mode.name == 'RTL'
    ca.cleanup()
    print("Test passed")

if __name__=='__main__':
    test_nominal()