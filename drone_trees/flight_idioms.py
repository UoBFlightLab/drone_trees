# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 09:47:59 2020

@author: aeagr
"""

import py_trees
from drone_trees import leaf_nodes as lf

# def preflight_Module(
#         vehicle, 
#         name="Pre-flight Module",
#         safety_check=py_trees.behaviours.Dummy(name="Safety Check"),
#         fallback=py_trees.behaviours.Dummy(name="Fallback")
#         ):
#     bt = py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(name=name,
#                                                                            children=[safety_check,
#                                                                                      fallback]), name=name)
#     bt.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
#     return bt

# def precond_module(
#         va,
#         name="Precond Module",
#         safety_check=py_trees.behaviours.Dummy(name="Safety Check"),
#         mishap_voice="Precond Fail",
#         fallback=py_trees.behaviours.Dummy(name="Fallback"),
#         wait=False):

#     sq = py_trees.composites.Sequence(children=[lf.PlaySound(mishap_voice, va)])
#     if wait==True:
#         timer=py_trees.timers.Timer()
#         sq.add_children([timer, fallback])
#         bt = py_trees.composites.Selector(name=name,children=[safety_check, py_trees.decorators.Inverter(py_trees.decorators.OneShot(sq))])

#     else:
#         sq.add_child(fallback)
#         bt = py_trees.composites.Selector(name=name,children=[safety_check, py_trees.decorators.Inverter(py_trees.decorators.OneShot(sq))])
        
#     return bt

def safety_module(name="Safety Module",
                  check=py_trees.behaviours.Dummy(name="Safety Check"),
                  fallback=py_trees.behaviours.Dummy(name="Fallback")
                  ):
    bt = py_trees.composites.Selector(name=name,children=[check, fallback])
    return bt


def wait_take_off(vehicle):
    
    bt = py_trees.decorators.FailureIsRunning(lf.AltLocalAbove(vehicle, 0.1))
    bt.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    bt.name = "Wait for takeoff"
    return bt

# def wait_landing(vehicle):

#     sq = py_trees.composites.Sequence(name="landing Handler")
#     landed = py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(lf.AltLocalAbove(vehicle, 0.5)))
#     oneShot_playSound = py_trees.decorators.OneShot(lf.PlaySound("Landed", va))

#     sq.add_children([lf.CheckLanding(vehicle), landed, oneShot_playSound])

#     lnd = py_trees.decorators.FailureIsRunning(sq, name="Is landing\ncomplete?")

#     lnd.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

#     return lnd

def wait_for_wp_hold(vehicle,wpn):
    at_wp = py_trees.composites.Sequence(name=("Is vehicle at\n waypoint %i ?" % wpn),
                                         children=[lf.CheckCounterLessThan(vehicle, wpn+1),
                                                   py_trees.decorators.FailureIsRunning(lf.CheckCounter(vehicle, wpn)),
                                                   py_trees.decorators.FailureIsRunning(lf.LatSpeedUnder(vehicle, 0.2)),
                                                   lf.CheckCounter(vehicle, wpn)])
    at_wp.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return at_wp

def leg_handler(vehicle,wp1,wp2,preconds=[py_trees.behaviours.Dummy(name="Precondition 1")]):
    assert wp2>wp1
    wait_then_set_ctr = py_trees.composites.Sequence(name='Wait then Set CTR to {}'.format(wp2),
                                                     children = [py_trees.timers.Timer(duration=2.0),
                                                                 lf.SetCounter(vehicle, wp2)])
    wait_then_set_ctr.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    
    precond_priority = py_trees.composites.Selector(name="Preconds for {} to {}".format(wp1,wp2))
    for precond in preconds:
        precond_priority.add_child(py_trees.decorators.Inverter(precond))
    precond_priority.add_child(wait_then_set_ctr)
        
    lh = py_trees.composites.Sequence(name='Transition handler {} to {}'.format(wp1,wp2),
                                      children=[wait_for_wp_hold(vehicle, wp1),
                                                precond_priority])
    lh_dec = py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(lh))
    return lh_dec

def flight_manager(vehicle,preflight,safety,legs):
    safety_parallel = py_trees.composites.Parallel(name="Safety Monitor",
                                                   children=safety)
    invtr_plus_parallel = py_trees.decorators.Inverter(child=safety_parallel)
    
    leg_manager = py_trees.composites.Parallel(name="Leg Handlers",
                                               children=legs)
    
    mission = py_trees.composites.Selector(name="Mission Manager",
                                           children=[invtr_plus_parallel,
                                                     leg_manager])

    pf = py_trees.composites.Sequence(name="Preflight checks",
                                      children=[py_trees.decorators.FailureIsRunning(b) 
                                                for b in preflight])
    
    fm = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="FLight manager",
                                                                  children=[pf,
                                                                            wait_take_off(vehicle),
                                                                            mission]))
    return fm


class FlightManager:
    def __init__(self, vehicle, va, wp_count, safety_modules):
        super(FlightManager, self).__init__()
        self._vehicle = vehicle
        self._va = va
        self._wp_count = wp_count
        self._safety_modules = safety_modules
    
    def at_wp(self, wp_n):
        at_wp = py_trees.composites.Sequence(name=("Is vehicle at\n waypoint %i ?" % wp_n),
                                             children=[lf.CheckCounter(self._vehicle, wp_n),
                                                       py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(lf.LatSpeedUnder(self._vehicle,1.0))),
                                                       lf.CheckCounter(self._vehicle, wp_n),
                                                       py_trees.decorators.FailureIsRunning(lf.LatSpeedUnder(self._vehicle, 1)),
                                                       lf.CheckCounter(self._vehicle, wp_n),
                                                       lf.PlaySound(("Waypoint %i" % wp_n), self._va)])
        at_wp.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        return at_wp

    def leg_handler(self, wp_n, increment, precond_next_wp=None):

        wait_then_set_ctr = py_trees.composites.Sequence(name="Wait then Set CTR to %i"  % (wp_n+int(increment)))
        wait_then_set_ctr.add_children([py_trees.timers.Timer(duration=2.0), lf.SetCounter(self._vehicle, (wp_n+int(increment)))])
        wait_then_set_ctr.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

        if precond_next_wp!=None:
            
            precond_priority = py_trees.composites.Selector(name="WP {} Preconds".format(wp_n+int(increment)))
            for precond in precond_next_wp:
                precond_priority.add_child(py_trees.decorators.Inverter(precond))
            precond_priority.add_child(wait_then_set_ctr)
            wrap=precond_priority

        else:
            wrap=wait_then_set_ctr
    
        
        if increment == 1 and wp_n==2:
            lh_name = "Leg Handler {}".format(int(wp_n/2))
        else:
            lh_name = "Leg Handler {}".format(int((wp_n+1)/2))

        lh = py_trees.composites.Sequence(name=lh_name,
                                          children=[self.at_wp(wp_n),
                                          wrap])

        return lh

    def flight_manager_root(self):
        # Consruct Safety Branch
        fm = py_trees.composites.Selector(name="Flight Manager")
        safety_parallel = py_trees.composites.Parallel(name="Safety Monitor")
        # Add safety modules
        for sm in self._safety_modules:
            safety_parallel.add_child(sm)

        invtr_plus_parallel = py_trees.decorators.Inverter(child=safety_parallel)

        # Core precond
        
        clearance = {3: 45, 5: 15, 7: 8, 9: 4}

        # Construct Waypoint Branch
        wrap=[py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(self.leg_handler(2,1), name="F=R"))]
        wp_n = 3
        while wp_n < self._wp_count:
            if wp_n == self._wp_count-2:
                wrap.append(py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(self.at_wp(wp_n), name="F=R")))
            else:
                # Precond Construction
                rtk_check = precond_module(self._va, name="RTK Check", safety_check=lf.CheckGPS(self._vehicle, 5), mishap_voice="No RTK fix", fallback=go_SAFTI(self._vehicle, self._va, (self._wp_count-2)), wait=True)
                clearance_check = precond_module(self._va, name="Clearance > {}?".format(int(clearance[wp_n])), safety_check=lf.CheckObstacle(self._vehicle, clearance[wp_n]), mishap_voice="Clearance Fail", fallback=go_SAFTI(self._vehicle, self._va, (self._wp_count-2)))
                rtk_check.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
                clearance_check.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
                # Mission leg construction
                wrap.append(py_trees.decorators.OneShot(py_trees.decorators.FailureIsRunning(self.leg_handler(wp_n, 2, precond_next_wp=[rtk_check, clearance_check]), name="F=R")))
            wp_n+=2

        mission_handler = py_trees.composites.Parallel(name="Mission Handler", policy=py_trees.common.ParallelPolicy.SuccessOnSelected(children=[wrap[-1]]), children=wrap) 
        fm.add_children([invtr_plus_parallel, mission_handler])

        return fm

def fm_behaviour(vehicle, va, wp_count, safety_modules=[py_trees.behaviours.Dummy(name="Safety Module 1")]):

    fm = FlightManager(vehicle, va, wp_count, safety_modules)
    fmb = fm.flight_manager_root()
    return fmb
