# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 12:20:27 2020

@author: aeagr
"""

import sys
import dronekit_sitl
from dronekit import connect
import socket
from py_trees.trees import BehaviourTree
from py_trees.visitors import DebugVisitor,SnapshotVisitor
from py_trees.display import render_dot_tree, unicode_tree
from py_trees.common import Status
import time

class ControlAutomaton:
    
    def __init__(self, bt_func):
        super(ControlAutomaton, self).__init__()
        self._bt_func = bt_func
        self._bt = None
        self._app_name = "App name"
        self._sitl = None
        self._connection_string = None
        self.vehicle = None
        self._loop_should_exit = False
        self._max_ticks = 1000
    
    def startup(self,force_sitl=False):
        """
        Interpret command line arguments and connect to the vehicle 
        if necessary (i.e. not in render mode)
        """
        self._app_name = sys.argv[0]
        if force_sitl:
            self._sitl = dronekit_sitl.start_default()
            self._connection_string = self._sitl.connection_string()
            print("Using SITL via {}".format(self._connection_string))
            self.connect()
        if len(sys.argv)!=2:
            print("""Usage:
                  
  {0} sitl
    run with built-in SITL simulator
  {0} render
    just render the behaviour tree
  {0} <connection string>
    connect as prescribed and fly the mission""".format((self._app_name)))
        elif sys.argv[1]=='sitl':
            self._sitl = dronekit_sitl.start_default()
            self._connection_string = self._sitl.connection_string()
            print("Using SITL via {}".format(self._connection_string))
            self.connect()
        elif sys.argv[1]=='render':
            print("Rendering only")
            self.render()
        else:
            self._connection_string = sys.argv[1]
            print("Attempting to connect via {}".format(self._connection_string))
            self.connect()

    def render(self):
        render_dot_tree(self._bt_func(self.vehicle), name=self._app_name)
                    
    def connect(self):
        try:
            self.vehicle = connect(self._connection_string, wait_ready=True)
        except socket.error as e:
            print(e)
            return
        self._bt = BehaviourTree(self._bt_func(self.vehicle))
        self._snapshot_visitor = SnapshotVisitor()
        self._bt.visitors.append(self._snapshot_visitor)
        self._bt.visitors.append(DebugVisitor())
        
    def finished(self):
        if self._bt.root.status==Status.SUCCESS:
            return True
        else:
            return False
        
    def tick(self):
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
        # exit after timeout or completion
        if self.finished():
            print("**** Flight completed in {} steps".format(self._bt.count))
            self._loop_should_exit = True
        if self._bt.count>self._max_ticks:
            print("**** Exiting after {} steps".format(self._bt.count))
            self._loop_should_exit = True
        else:
            print("******** {} *********".format(self._bt.count-1))

    def cleanup(self):
        if self.vehicle:
            print("Disconnecting from vehicle on {}".format(self._connection_string))
            self.vehicle.close()
        if self._sitl:
            print("Shutting down SITL instance")
            self._sitl.stop()
        
    def main(self):
        self.startup()
        if self.vehicle:
            try:
                while not self._loop_should_exit:
                    self.tick()
                    time.sleep(1)
            except:
                print(sys.exc_info()[0])
            self.cleanup()