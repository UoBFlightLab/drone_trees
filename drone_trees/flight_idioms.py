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

flight_idiom.py:

Provide a series of standard behaviour tree constructs to assist the
development of trees for drone control.  In the limit, designer can use
just these idioms and not use py_trees directly.  Use of standard constructs
offers some degree of safety assurance, but only if the connected parts
make sense: garbage in, garbage out.

"""
###############################################################################


import py_trees
from drone_trees import leaf_nodes as lf


def safety_module(name="Safety Module",
                  check=py_trees.behaviours.Dummy(name="Safety Check"),
                  fallback=py_trees.behaviours.Dummy(name="Fallback"),
                  oneshot=False
                  ):
    """
    A standard tree for use in the flight_manager idiom as a global safety
    check, i.e. condition that is monitored throughout the flight and must
    trigger a standard response.  Typically used for some eventuality that
    requires either a wait or an immediate abort, regardless of when or
    where it is encountered.

    See examples/fly_bridge.py

    Parameters
    ----------
    name : str, optional
        Label for the module in the behaviour tree.
        The default is "Safety Module".
    check : py_trees.behaviour.Behaviour, optional
        Behaviour representing the check of the safety condition.
            SUCCESS means all is well and no further action needed.
            FAILURE means all is not well and the intervention is triggered.
        The default is py_trees.behaviours.Dummy(name="Safety Check").
    fallback : py_trees.behaviour.Behaviour, optional
        The behaviour to enact the intervention.
            SUCCESS means intervention complete, with authority handed back.
            RUNNING means intervention still on-going.  Default is this, which
            will just block the rest of the tree actions until the condition
            clears.
            FAILURE would be worrying here and would stop the whole tree.
        The default is py_trees.behaviours.Dummy(name="Fallback").
    oneshot : bool, optional
            True encapsulates the fallback behaviour in a oneshot resulting
            in an unrepeatable response.
            False allows for a repeatable response.
        The default is False

    Returns
    -------
    node : py_trees.behaviour.Behaviour
        Root node of the generated tree.

    """
    if oneshot:
        node = py_trees.composites.Selector(
            name=name,
            children=[check, py_trees.decorators.OneShot(fallback)])
    else:
        node = py_trees.composites.Selector(
            name=name,
            children=[check, fallback])

    node.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT

    return node


def wait_for_auto_take_off(vehicle):
    """
    Behaviour to wait for take-off in AUTO mode.  Returns RUNNING until:
        Mode is AUTO
        Vehicle is armed
        Altitude above 0.1m from home
    Has no timeout.  Used in flight_manager idiom after preflight.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.behaviour.Behaviour
        Root node of the generated tree.

    """
    node = py_trees.decorators.FailureIsRunning(py_trees.composites.Sequence(
        name="Wait for AUTO takeoff",
        children=[lf.CheckMode(vehicle, "AUTO"),
                  lf.IsArmed(vehicle),
                  lf.AltLocalAbove(vehicle, 0.1)]))
    return node


def wait_for_landing(vehicle):
    """
    Behaviour to wait for landing in any mode.  Returns RUNNING until Vehicle
    disarms, as happens automatically on landing.
    Has no timeout.  Used in flight_manager idiom to wait for conclusion.

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    Returns
    -------
    node : py_trees.behaviour.Behaviour
        Root node of the generated tree.

    """
    node = py_trees.decorators.FailureIsRunning(
        py_trees.decorators.Inverter(lf.IsArmed(vehicle)))
    node.name = "Wait for landing"
    return node


def wait_for_wp_hold(vehicle, wpn):
    """
    Behaviour to wait for arrival at specified waypoint and hold steady:
        FAILURE: missed waypoint, as WP index already higher on entry
        RUNNING: waypoint index at or below required but still on the way
        SUCCESS: holding steady at the desired waypoint (measured by low
                                                         lateral speed)

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        The MAVLINK interface

    wpn : int
        The index of the waypoint in question.

    Returns
    -------
    node : py_trees.behaviour.Behaviour
        Root node of the generated tree.
    """
    # TODO think about logic here, as will not fail if this WP is skipped over
    # maybe need check = wp with fallback to check < wp
    at_wp = py_trees.composites.Sequence(
        name=(f"Is vehicle at\n waypoint {wpn} ?"),
        children=[lf.CheckCounterLessThan(vehicle, wpn+1),
                  lf.WaitForWaypoint(vehicle, wpn),
                  py_trees.decorators.FailureIsRunning(
                      lf.LatSpeedUnder(vehicle, 0.2))])
    at_wp.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
    return at_wp


def leg_handler(vehicle, wp1, wp2, preconds=[]):
    """
    Manage procession to next leg of a mission.

    Advance waypoint counter to WP2 if vehicle steady at WP1 and preconditions
    met.  Typically used if WP1 has a delay,followed by a JMP and then WP2.

    Unlike safety_module conditions, these checks are only applied at certain
    mission stages, and are used where specific tigheter requirements apply
    to certain phases of the mission.  Examples include RTK functionality
    and rangefinder clearances for proximity operations.

    Designed for use in the flight_manager idiom.
    See examples/fly_bridge.py

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        Provides the MAVLINK interface
    wp1 : int
        Drone must be steady at this waypoint before commencing next leg.
    wp2 : int
        This waypoint is the first in the next leg.
    preconds : list of py_trees.behaviour.Behaviour
        Each of these conditions must return SUCCESS before advancement to
        next leg.  RUNNING will block the advancement.

    Returns
    -------
    lh_dec : py_trees.behaviour.Behaviour
        The root node of the generated tree.  When ticked:
            SUCCESS means this jump has been made.
            FAILURE means this jump cannot be made.
            RUNNING means this jump may yet be made, but not now.

    """
    assert wp2 > wp1
    wait_then_set_ctr = py_trees.composites.Sequence(
        name=f'Wait then Set CTR to {wp2}',
        children=[py_trees.timers.Timer(duration=2.0),
                  lf.SetCounter(vehicle, wp2)])
    wait_then_set_ctr.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    if not preconds:
        lh = py_trees.composites.Sequence(
            name=f'Transition handler {wp1} to {wp2}',
            children=[wait_for_wp_hold(vehicle, wp1),
                      wait_then_set_ctr])
    else:
        precond_priority = py_trees.composites.Selector(
            name=f"Preconds for {wp1} to {wp2}")
        for precond in preconds:
            precond_priority.add_child(py_trees.decorators.Inverter(precond))
        precond_priority.add_child(wait_then_set_ctr)

        lh = py_trees.composites.Sequence(
            name=f'Transition handler {wp1} to {wp2}',
            children=[wait_for_wp_hold(vehicle, wp1),
                      precond_priority])

    lh_dec = py_trees.decorators.OneShot(
        py_trees.decorators.FailureIsRunning(lh))

    lh.blackbox_level = py_trees.common.BlackBoxLevel.COMPONENT
    return lh_dec


def flight_manager(vehicle, preflight, safety, legs):
    """
    Flight manager integrates preflight, safety checks and leg handlers to
    provide a standard behaviour tree root for drone control.

    See examples/fly_bridge.py

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        Provides MAVLINK interface
    preflight : list of py_trees.behaviour.Behaviour
        The preflight checks, in order.  All must return success before takeoff
        permitted.  Preflight waits indefinitely if they return otherwise.
    safety : list of py_trees.behaviour.Behaviour
        The safety modules, each produced using the safety_module idiom.
    legs : list of py_trees.behaviour.Behaviour
        The leg handlers, each produced using the leg_handler idiom.

    Returns
    -------
    fm_root : py_trees.behaviour.Behaviour
        Root node of the entire flight manager tree.
        Typically returned by the behaviour tree builder function
        that itself is the argument to the ControlAutomaton

    """
    check_mode_auto = lf.CheckMode(vehicle, 'AUTO')

    invtr_plus_mdchk = py_trees.decorators.Inverter(check_mode_auto)

    safety_parallel = py_trees.composites.Parallel(
        name="Safety Monitor",
        children=safety,
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    invtr_plus_parallel = py_trees.decorators.Inverter(child=safety_parallel)

    leg_manager = py_trees.composites.Parallel(
        name="Leg Handlers",
        children=legs,
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[legs[-1]], synchronise=False))

    mission = py_trees.composites.Selector(name="Mission Manager",
                                           children=[invtr_plus_mdchk,
                                                     invtr_plus_parallel,
                                                     leg_manager])

    pre_flight = py_trees.composites.Sequence(
        name="Preflight checks",
        children=[py_trees.decorators.FailureIsRunning(b) for b in preflight])

    fm_root = py_trees.decorators.OneShot(py_trees.composites.Sequence(
        name="FLight Manager",
        children=[pre_flight,
                  wait_for_auto_take_off(vehicle),
                  mission,
                  wait_for_landing(vehicle)]))
    return fm_root
