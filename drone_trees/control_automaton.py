# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 12:20:27 2020

@author: aeagr

Provides the ControlAutomaton class for drone_trees

"""

import sys
import socket
import time
import dronekit_sitl
from dronekit import connect
from py_trees.trees import BehaviourTree
from py_trees.visitors import DebugVisitor, SnapshotVisitor
from py_trees.display import render_dot_tree, unicode_tree
from py_trees.common import Status
from drone_trees.drone_tree_vehicle import DroneTreeVehicle

class ControlAutomaton:
    """
    Top-level app class providing functionality for running and testing
    behaviour trees for drone control via MAVLINK.  Includes:
        - optional SITL simulation
        - optional visualisation of behaviour tree
        - connection to MAVLINK
    """
    def __init__(self, bt_func, sitl_lat=51.454531, sitl_lon=-2.629158):
        """
        Construct an app embodying a given behaviour tree.

        Parameters
        ----------
        bt_func : function
            Should take a dronekit.Vehicle as an argument and return the root
            node of a behaviour tree, as a py_trees.behaviour.Behaviour object.
            Typically built using nodes from leaf_nodes and stitched together
            using py_trees.composites and py_trees.decorators or using the
            flight_idioms.

        sitl_lat : float, optional
            Latitude for home location if app launched in SITL mode.
            The default is 51.454531.
        sitl_lon : float, optional
            Longitude for home location if app launched in SITL mode.
            The default is -2.629158.

        Returns
        -------
        The ControlAutomaton

        """
        super(ControlAutomaton, self).__init__()
        self._bt_func = bt_func
        self._bt = None
        self._snapshot_visitor = None
        self._app_name = "App name"
        self._sitl = None
        self._sitl_lat = sitl_lat
        self._sitl_lon = sitl_lon
        self._connection_string = None
        self.vehicle = None
        self._loop_should_exit = False
        self._max_ticks = 1000

    def __del__(self):
        self.cleanup()

    def startup(self, override_args=None):
        """
        Interpret command line arguments, render the tree (if --render)
        and connect to the vehicle if necessary (i.e. not in render mode) 
        
        Parameters:
            
            override_args : list of str
                override command line arguments from function call
                e.g. for use in testing (see test_bridge.py)
                
        """
        self._app_name = sys.argv[0]
        if override_args:
            my_args = [self._app_name]
            my_args = my_args + override_args
        else:
            my_args = sys.argv[:]
            
        if len(my_args) != 2:
            print("""Usage:
                  
  {0} sitl
    run with built-in SITL simulator
  {0} render
    just render the behaviour tree
  {0} <connection string>
    connect as prescribed and fly the mission""".format((self._app_name)))
        elif my_args[1] == 'sitl':
            self._sitl = dronekit_sitl.start_default(lat=self._sitl_lat,
                                                     lon=self._sitl_lon)
            self._connection_string = self._sitl.connection_string()
            print("Using SITL via {}".format(self._connection_string))
            self.connect()
        elif my_args[1] == 'render':
            print("Rendering only")
            self.render()
        else:
            self._connection_string = my_args[1]
            print("Attempting to connect via {}".format(self._connection_string))
            self.connect()

    def render(self, file_name=None):
        """
        Print the behaviour tree graphically for visualization.  Produces
        a .dot file (Graphviz), a .png and a .svg (images)

        Parameters
        ----------
        file_name : str, optional
            File name stub for graphics files produced.
            The default is the name of the executed script.

        Returns
        -------
        None.

        """
        if file_name is None:
            file_name = self._app_name
        render_dot_tree(self._bt_func(self.vehicle), name=file_name)

    def connect(self):
        """
        Connect to the MAVLINK interface, either to own SITL process or to
        a specified external connection provided via the command line.

        Returns
        -------
        None.

        """
        try:
            self.vehicle = connect(self._connection_string, wait_ready=True, \
                                   vehicle_class=DroneTreeVehicle)
        except socket.error as e:
            print(e)
            return
        self._bt = BehaviourTree(self._bt_func(self.vehicle))
        self._snapshot_visitor = SnapshotVisitor()
        self._bt.visitors.append(self._snapshot_visitor)
        self._bt.visitors.append(DebugVisitor())

    def finished(self):
        """
        Check if behaviour tree has completed.

        Returns
        -------
        bool
            True if root node has status 'SUCCESS' or 'FAILURE'

        """
        if self._bt.root.status == Status.SUCCESS:
            return True
        if self._bt.root.status == Status.FAILURE:
            return True
        return False

    def tick(self):
        """
        Execute one tick of the behaviour tree, printing the tree in ASCII
        form, plus some simple status information.

        Returns
        -------
        None.

        """
        print("******** {} *********".format(self._bt.count))
        self._bt.tick()
        # print BT
        if self._snapshot_visitor:
            print(unicode_tree(self._bt.root,
                               visited=self._snapshot_visitor.visited,
                               previously_visited=self._snapshot_visitor.visited))
            print("+++++++++++++++++++++")
        print(self.vehicle.battery)
        print('Mode: {}'.format(self.vehicle.mode.name))
        print('Altitude: {}'.format(self.vehicle.location.global_relative_frame.alt))
        print('Connected to {}'.format(self._connection_string))
        print('Next waypoint is {}'.format(self.vehicle.commands.next))
        # exit after timeout or completion
        if self.finished():
            print("**** Flight completed in {} steps".format(self._bt.count))
            self._loop_should_exit = True
        if self._bt.count > self._max_ticks:
            print("**** Exiting after {} steps".format(self._bt.count))
            self._loop_should_exit = True
        else:
            print("******** {} *********".format(self._bt.count-1))

    def cleanup(self):
        """
        Close connection to the MAVLINK stream and, if a SITL simulator was
        launched, kill its process.  Use at end of each test, pass or fail.

        Returns
        -------
        None.

        """
        if self.vehicle:
            print("Disconnecting from vehicle on {}".format(self._connection_string))
            self.vehicle.close()
        if self._sitl:
            print("Shutting down SITL instance")
            self._sitl.stop()

    def main(self):
        """
        Execute the controller application.  Process arguments, connect,
        run tree until completion or timeout, and tidy up.

        NOTE: exit due to keyboard interrupt is trapped and cleanup() called.
        Other exceptions may leave hanging threads, connections or SITL
        processes.  Cleanup is called in destructor to minimize damage.

        Returns
        -------
        None.

        """
        self.startup()
        if self.vehicle:
            try:
                while not self._loop_should_exit:
                    self.tick()
                    time.sleep(1)
            except KeyboardInterrupt:
                self.cleanup()
            self.cleanup()
            