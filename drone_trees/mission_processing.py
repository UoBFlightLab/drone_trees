# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 09:45:52 2020

@author: aeagr
"""

from pymavlink.mavwp import MAVWPLoader
from drone_trees.leaf_nodes import SetCounter

class MissionUtility:
    
    def __init__(self,filename):
        super(MissionUtility, self).__init__()
        self._wp = MAVWPLoader()
        self._num_wps = self._wp.load(filename)
        print("Loaded {} waypoints".format(self._num_wps))
        assert self._wp.wpoints[0].command==22, 'First waypoint should be TakeOff (22)'
        assert self._wp.wpoints[-1].command==21, 'Last waypoint should be Landing (21)'
        jump_wps = [wp for wp in self._wp.wpoints if wp.command==177]
        assert len(jump_wps)>0, 'Need at least one jump to identify SAFTI'
        safti_set = set([wp.param1 for wp in jump_wps])
        assert len(safti_set)==1, 'Must have unique SAFTI'
        self._safti_num = min(safti_set)
        
    def go_safti(self,vehicle):
        return(SetCounter(vehicle, self._safti_num))
    
    def upload_mission(self,vehicle):
        pass # TODO
        