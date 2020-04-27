# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 11:52:28 2020

@author: aeagr
"""

from drone_trees import leaf_nodes as lf
from drone_trees.control_automaton import ControlAutomaton
from py_trees.composites import Sequence
from py_trees.decorators import FailureIsRunning, Inverter, OneShot

# utility function for behaviour to move by offset and wait until almost stationary
def move_behaviour(vehicle, dNorth, dEast, dDown):    
    bt_move = Sequence(name="move",
                       children=[lf.MoveDrone(vehicle,dNorth, dEast, dDown),                                        
                       FailureIsRunning(Inverter(lf.LatSpeedUnder(vehicle,1.0))),
                       FailureIsRunning(lf.LatSpeedUnder(vehicle,0.1))])
    return(bt_move)

def pre_flight(vehicle):
    bt_pf = FailureIsRunning(Sequence(name="Pre-flight",
                                      children=[lf.CheckMode(vehicle, "GUIDED"),
                                                lf.IsArmed(vehicle)]))
    return(bt_pf)

def behaviour_tree(vehicle):
    bt = OneShot(Sequence(name="Flight",
                  children=[pre_flight(vehicle),
                            lf.SimpleTakeoff(vehicle, 20),
                            FailureIsRunning(lf.AltLocalAbove(vehicle, 18)),
                            move_behaviour(vehicle, 50, 0, 0),
                            move_behaviour(vehicle, 0, 50, 0),
                            move_behaviour(vehicle, -50, 0, 0),
                            move_behaviour(vehicle, 0, -50, 0),
                            lf.Land(vehicle),
                            FailureIsRunning(Inverter(lf.AltLocalAbove(vehicle, 0.3)))
                            ]))
    return(bt)

app = ControlAutomaton(behaviour_tree)
    
if __name__ == "__main__":
    app.main()