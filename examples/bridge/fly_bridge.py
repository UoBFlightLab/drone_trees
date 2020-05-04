# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 11:52:28 2020

@author: aeagr
"""

from drone_trees import leaf_nodes as lf
from drone_trees.mission_handler import MissionHandler
from drone_trees.control_automaton import ControlAutomaton
from py_trees.composites import Sequence
from py_trees.decorators import FailureIsRunning, Inverter, OneShot

def pre_flight(vehicle):
    bt_pf = FailureIsRunning(Sequence(name="Pre-flight",
                                      children=[lf.CheckMode(vehicle, "AUTO"),
                                                lf.IsArmed(vehicle)]))
    return(bt_pf)

def behaviour_tree(vehicle):
    mh = MissionHandler('executable_mission.txt')
    
    bt_mission = Sequence(name="Mission TX", 
                          children=[mh.upload_mission(vehicle),
                                    #mh.verify_mission(vehicle),
                                    ])
    
    bt_preflight = FailureIsRunning(Sequence(name="Pre-flight",
                                             children=[lf.CheckMode(vehicle, "AUTO"),
                                                       lf.IsArmed(vehicle)]))
    
    bt = OneShot(Sequence(name="Flight",
                  children=[bt_mission,
                            bt_preflight,
                            FailureIsRunning(lf.AltLocalAbove(vehicle, 18)),
                            FailureIsRunning(Inverter(lf.AltLocalAbove(vehicle, 0.3)))
                            ]))
    return(bt)

app = ControlAutomaton(behaviour_tree)
    
if __name__ == "__main__":
    app.main()