# -*- coding: utf-8 -*-
"""
Created on Fri Apr 24 09:45:52 2020

@author: aeagr
"""

from dronekit import Command

class MissionUtility:
    
    def __init__(self, vehicle):
        super(MissionUtility, self).__init__()
        self._vehicle = vehicle
        self._cmds = self._vehicle.commands
        self._executable_mission_filename = 'executable_mission.txt'
        

    def readmission(self, aFileName):
        """
        Load a mission from a file into a list. The mission definition is in the Waypoint file
        format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

        This function is used by upload_mission().
        """
        print("\nReading mission from file: %s" % aFileName)
        missionlist=[]
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i==0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray=line.split('\t')
                    ln_index=int(linearray[0])
                    ln_currentwp=int(linearray[1])
                    ln_frame=int(linearray[2])
                    ln_command=int(linearray[3])
                    ln_param1=float(linearray[4])
                    ln_param2=float(linearray[5])
                    ln_param3=float(linearray[6])
                    ln_param4=float(linearray[7])
                    ln_param5=float(linearray[8])
                    ln_param6=float(linearray[9])
                    ln_param7=float(linearray[10])
                    ln_autocontinue=int(linearray[11].strip())
                    cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def gen_exe_mission(self, aFileName, safti_wp, delay_s):
        # Read the mission file
        missionlist = self.readmission(aFileName)

        input_wp_len = len(missionlist)
        new_safti = float(safti_wp) + float(input_wp_len-5) 
        j = 3
        while j <= input_wp_len:
            cmd = Command( 0, 0, 0, 3, 177, 0, 1, new_safti, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            missionlist.insert(j, cmd)
            j+=2

        output='QGC WPL 110\n'
        for cmd in missionlist:
            cmd.param1=delay_s if cmd.command==16 else cmd.param1
            cmd.seq=missionlist.index(cmd)+1
            commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
            output+=commandline
        with open(self._executable_mission_filename, 'w') as file_:
            print(" Write mission to file")
            file_.write(output)
        
        # Count number of WP
        wp_count = len(missionlist) 

        return wp_count


    def upload_mission(self):
        """
        Upload a mission from a file. 
        """
        #Read mission from file
        missionlist = self.readmission(self._executable_mission_filename)
        print("\nUpload mission from a file: %s" % self._executable_mission_filename)
        #Clear existing mission from vehicle
        print(' Clear mission')
        self._cmds.clear()
        #Add new mission to vehicle
        for command in missionlist:
            command.seq=missionlist.index(command)+1
            self._cmds.add(command)
        print('Upload mission')
        self._cmds.upload()