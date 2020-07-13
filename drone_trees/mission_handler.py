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

mission_handler.py:

Provides behaviour tree functionality for mission handling, including mission
import, validation, upload to drone, and SAFTI waypoint functions.

"""
###############################################################################

from pymavlink.mavwp import MAVWPLoader
from drone_trees.leaf_nodes import SetCounter, MissionUpload  # , MissionVerify


class MissionHandler:
    """Provides behaviour tree functionality for mission handling, including
    mission import, validation, upload to drone, and SAFTI waypoint functions.
    """
    def __init__(self, filename):
        """
        Construct a MissionHandler

        Parameters
        ----------
        filename : str
            The name of the waypoint file.  This file should use MAVLINK
            waypoint file format conventions, e.g. from MissionPlanner or
            mavproxy.
        """
        super(MissionHandler, self).__init__()
        self._safti_num = None
        self._wp = MAVWPLoader()
        self._num_wps = self._wp.load(filename)
        # bit of a hack: fix the sequence number
        for w in self._wp.wpoints:
            w.seq = w.seq+1
        self.validate_mission()
        print("Loaded {} waypoints".format(self._num_wps))

    def print(self):
        """
        Print the loaded mission in simple text form.

        Returns
        -------
        None.

        """
        for w in self._wp.wpoints:
            print(w.seq, w.command,
                  w.param1, w.param2, w.param3, w.param4,
                  w.x, w.y, w.z)

    def validate_mission(self):
        """
        Validate the mission.  Raises AssertionError if validation checks fail.
        Checks are:
            Is first WP a takeoff?
            Is last WP a landing?

        Returns
        -------
        None.

        """
        assert self._wp.wpoints[0].command == 22, 'First waypoint should be \
            TakeOff (22)'
        assert self._wp.wpoints[-1].command == 21, 'Last waypoint should be \
            Landing (21)'

    def get_safti(self):
        """
        Identify and store the SAFTI waypoint index.  Identified as the target
        of all the Jump commands in the mission.  Raises AssertionError if
        SAFTI cannot be identified, e.g. if there are no jumps or if they go
        to different places.

        Returns
        -------
        None.

        """
        jump_wps = [wp for wp in self._wp.wpoints if wp.command == 177]
        assert len(jump_wps) > 0, 'Need at least one jump to identify SAFTI'
        # TODO slicker set comprehension of the item below?
        safti_set = set([wp.param1 for wp in jump_wps])
        assert len(safti_set) == 1, 'Must have unique SAFTI'
        self._safti_num = min(safti_set)

    def go_safti(self, vehicle):
        """
        Behaviour to jump to SAFTI.  Raises AssertionError if no SAFTI
        can be identified.

        Parameters
        ----------
        vehicle : dronekit.Vehicle
            Interface to MAVLINK

        Returns
        -------
        node : leaf_nodes.SetCounter
            Behaviour tree node to advance WP counter to SAFTI

        """
        if self._safti_num is None:
            self.get_safti()
        return SetCounter(vehicle, self._safti_num)

    def upload_mission(self, vehicle):
        """
        Behaviour tree node to upload the mission to the drone.

        Parameters
        ----------
        vehicle : dronekit.Vehicle
            Interface to MAVLINK

        Returns
        -------
        node : leaf_nodes.MissionUpload
            Behaviour tree node to upload the mission.

        """
        return MissionUpload(vehicle, self._wp.wpoints[:])
