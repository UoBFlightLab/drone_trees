# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 09:45:52 2020

@author: aeagr
"""

from pymavlink.mavwp import MAVWPLoader
from drone_trees.leaf_nodes import SetCounter, MissionUpload, MissionVerify

class MissionHandler:
    
    def __init__(self,filename):
        super(MissionHandler, self).__init__()
        self._safti_num = None
        self._wp = MAVWPLoader()
        self._num_wps = self._wp.load(filename)
        self.validate_mission()
        print("Loaded {} waypoints".format(self._num_wps))
        
    def validate_mission(self):
        assert self._wp.wpoints[0].command==22, 'First waypoint should be TakeOff (22)'
        assert self._wp.wpoints[-1].command==21, 'Last waypoint should be Landing (21)'
        jump_wps = [wp for wp in self._wp.wpoints if wp.command==177]
        assert len(jump_wps)>0, 'Need at least one jump to identify SAFTI'
        safti_set = set([wp.param1 for wp in jump_wps])
        assert len(safti_set)==1, 'Must have unique SAFTI'
        self._safti_num = min(safti_set)
        
    def go_safti(self,vehicle):
        if self._safti_num is None:
            raise(Exception("Mission has no SAFTI waypoint"))
        else:
            return(SetCounter(vehicle, self._safti_num))
    
    def upload_mission(self,vehicle):
        return(MissionUpload(vehicle,self._wp.wpoints))    
        
    def verify_mission(self,vehicle):
        return(MissionVerify(vehicle,self._wp.wpoints))    
        