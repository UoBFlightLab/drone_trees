# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 11:52:28 2020

@author: aeagr
"""

from drone_trees import leaf_nodes as lf
from drone_trees import flight_idioms as im
from drone_trees.mission_handler import MissionHandler
from drone_trees.control_automaton import ControlAutomaton
# from py_trees.composites import Sequence
# from py_trees.decorators import FailureIsRunning, Inverter, OneShot

def behaviour_tree(vehicle):
    mh = MissionHandler('executable_mission.txt')
        
    preflight_behaviours = [mh.upload_mission(vehicle),
                            lf.CheckGPS(vehicle, 3),
                            lf.CheckEKF(vehicle),
                            lf.CheckMode(vehicle, "AUTO"),
                            lf.IsArmed(vehicle)]
    
    safety_low_battery = im.safety_module(name="Low Battery",
                                          check=lf.BatteryLevelAbove(vehicle, 30),
                                          fallback=mh.go_safti(vehicle))
    
    safety_avoidance = im.safety_module(name="Collision avoidance", 
                                        check=lf.CheckDistance(vehicle, 2, 2.),
                                        fallback=mh.go_safti(vehicle))

    safety_ekf = im.safety_module(name="Collision avoidance", 
                                        check=lf.CheckEKF(vehicle),
                                        fallback=mh.go_safti(vehicle))
    
    leg_3_5 = im.leg_handler(vehicle, 3, 5)
    leg_5_7 = im.leg_handler(vehicle, 5, 7)
    leg_7_9 = im.leg_handler(vehicle, 7, 9)
    leg_9_11 = im.leg_handler(vehicle, 9, 11)
    
    bt = im.flight_manager(vehicle,
                           preflight=preflight_behaviours,
                           safety=[safety_ekf,
                                   safety_low_battery],
                           legs=[leg_3_5,
                                 leg_5_7,
                                 leg_7_9,
                                 leg_9_11])
    
    return(bt)

app = ControlAutomaton(behaviour_tree)
    
if __name__ == "__main__":
    app.main()