# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 12:36:33 2020

@author: aeagr
"""

from fly_bridge import behaviour_tree
from drone_trees.control_automaton import ControlAutomaton
from dronekit import VehicleMode
from time import sleep

def test_nominal():
    ca = ControlAutomaton(behaviour_tree)
    ca.startup(force_sitl=True)
    for ii in range(10):
        ca.tick()
        sleep(1)
    # should still be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    ca.vehicle.arm()
    for ii in range(3):
        ca.tick()
        sleep(1)
    # should be on the ground
    assert ca.vehicle.location.global_relative_frame.alt < 0.2
    # switch to auto and lift the throttle to trigger auto takeoff
    ca.vehicle.mode = VehicleMode('AUTO')
    ca.vehicle.channels.overrides['3']=1700
    for ii in range(200):
        ca.tick()
        if ca.finished():
            break
        sleep(1)
    # should be back on the ground at HOME
    assert ca.vehicle.location.global_relative_frame.alt < 0.3
    ca.cleanup()
    print("Test passed")

if __name__=='__main__':
    test_nominal()