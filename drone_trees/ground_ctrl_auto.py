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
import time

class GroundControlAutomaton:
    
    def __init__(self, bt_func):
        super(GroundControlAutomaton, self).__init__()
        self._bt_func = bt_func
        self._bt = None
        self._app_name = "App name"
        self._sitl = None
        self._connection_string = None
        self._vehicle = None
        self._loop_should_exit = False
    
    def main(self):
        self._app_name = sys.argv[0]
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
            self.run()
        elif sys.argv[1]=='render':
            print("Rendering only")
            self.render()
        else:
            self._connection_string = sys.argv[1]
            print("Attempting to connect via {}".format(self._connection_string))
            self.run()
        
    def render(self):
        render_dot_tree(self._bt_func(self._vehicle), name=self._app_name)
        
    def run(self):
        try:
            self._vehicle = connect(self._connection_string, wait_ready=True)
        except socket.error as e:
            print(e)
            return
        self._bt = BehaviourTree(self._bt_func(self._vehicle))
        self._snapshot_visitor = SnapshotVisitor()
        self._bt.visitors.append(self._snapshot_visitor)
        self._bt.visitors.append(DebugVisitor())

        try:
            while not self._loop_should_exit:
                print("******** {} *********".format(self._bt.count))
                print(self._vehicle.battery)
                print(self._vehicle.mode.name)
                print(self._vehicle.location.global_frame.alt)
                print("+++++++++++++++++++++")
                self._bt.tick()
                # print it
                if self._snapshot_visitor:
                    print(unicode_tree(self._bt.root,
                                       visited=self._snapshot_visitor.visited,
                                       previously_visited=self._snapshot_visitor.visited))
                # now the tree bit
                if self._bt.count>1000:
                    print("Flight completed")
                    self._loop_should_exit = True
                else:
                    print("******** {} *********".format(self._bt.count))
                # pause
                time.sleep(1)
        except:
            print(sys.exc_info()[0])
        
        if self._vehicle:
            print("Disconnecting from vehicle on {}".format(self._connection_string))
            self._vehicle.close()
        if self._sitl:
            print("Shutting down SITL instance")
            self._sitl.stop()