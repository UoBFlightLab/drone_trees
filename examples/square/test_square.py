# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 12:36:33 2020

@author: aeagr
"""

from fly_square import behaviour_tree
from drone_trees.control_automaton import ControlAutomaton
from dronekit import VehicleMode
from time import sleep

def disp_from_home(veh):
    dlat = veh.location.global_relative_frame.lat \
                - veh.home_location.lat
    dlon = veh.location.global_relative_frame.lon \
                - veh.home_location.lon
    return (dlat,dlon)

def test_nominal():
    ca = ControlAutomaton(behaviour_tree)
    ca.startup(force_sitl=True)
    for ii in range(10):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    ca.vehicle.mode = VehicleMode('GUIDED')   
    for ii in range(10):
        ca.tick()
        sleep(1)
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