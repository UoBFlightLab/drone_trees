#!/usr/bin/env python

import py_trees
import math
import os
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import pyttsx3
import queue
from queue import Queue
import threading
import time

class VoiceAssistant(threading.Thread):
    def __init__(self):
        super(VoiceAssistant, self).__init__(daemon=True)
        self._engine = pyttsx3.init()
        self._q = Queue()
        self._loop_should_exit = False
    
    def kill(self):
        self._loop_should_exit = True
        print('----------Kill-----------')

    def add_say(self, msg):
        self._q.put(msg)
        print("[add_say] Adding: \"" + msg + "\"")

    def run(self):
        while not self._loop_should_exit:
            try:
                self._engine.say(self._q.get(False))
                self._engine.startLoop(False)
                self._engine.iterate()
                time.sleep(1)
                self._engine.endLoop()
                self._q.task_done()
            except queue.Empty as e:
                pass

class MissionUtility:
    
    def __init__(self, vehicle):
        super(MissionUtility, self).__init__()
        self._vehicle = vehicle
        self._cmds = self._vehicle.commands
        self._executable_mission_filename = os.path.join('mission', 'executable_mission.txt')
        

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


class MissionUpload(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mu):
        super(MissionUpload, self).__init__('Mission Upload')
        self._vehicle = vehicle
        self._mu = mu
        
    def update(self):
        self._mu.upload_mission()
        return py_trees.common.Status.SUCCESS

class log:
    def __init__(self, root, path):
        super(log, self).__init__()
        self._root = root
        self._path = path
        
        # Make HTML file for the html log
        self._bt_log_path = os.path.join(self._path, 'BT')
        self._filename = os.path.join(self._bt_log_path, 'bt_log.html')
        self._f = open(self._filename, 'w')
        self._f.write('<html><head><title>Foo</title><body>')

    def logging(self, iteration, dot=False):
        self._f.write("<p>******************** %i ********************</p>" % iteration)
        self._f.write(py_trees.display.xhtml_tree(self._root, show_status=True))
        if dot:
            py_trees.display.render_dot_tree(self._root, name='tick_%i' % iteration, target_directory=self._bt_log_path)

    def terminate(self):
        self._f.write("</body></html>")
        self._f.close()

class SetParam(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, param_name, new_value):
        # use name of mode as label for behaviour
        super(SetParam, self).__init__("%s=%i" % (param_name, new_value))
        self._vehicle = vehicle
        self._param_name = param_name
        self._new_value = new_value

    def update(self):
        self._vehicle.parameters[self._param_name]=self._new_value
        return py_trees.common.Status.SUCCESS


class ChangeMode(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(ChangeMode, self).__init__(mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode = VehicleMode(mode_name)

    def update(self):
        self._vehicle.mode = self._mode
        return py_trees.common.Status.SUCCESS

class CheckGPS(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, fixType):
        super(CheckGPS, self).__init__("GPS Status Check > %i?" % fixType)
        self._vehicle = vehicle
        self._fixType = fixType

    def update(self):
        if self._vehicle.gps_0.fix_type > self._fixType:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckObstacle(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, clearance):
        super(CheckObstacle, self).__init__("Clearance > %i ?" % clearance)
        self._vehicle = vehicle
        self._clearance = clearance
        self._distance = None

    def checkMavlink(self):
        @self._vehicle.on_message('DISTANCE_SENSOR')
        def listener(dkself, name, msg):
            self._distance = msg.current_distance

    def update(self):
        while self._distance==None:
            self.checkMavlink()
        if self._distance/100 > self._clearance:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
class CheckEKF(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckEKF, self).__init__("EKF Check")
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.ekf_ok:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckCounter(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(CheckCounter, self).__init__("Check Counter %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        if self._wpn == self._vehicle.commands.next:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckCounterLessThan(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(CheckCounterLessThan, self).__init__("Check Counter < %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        if self._vehicle.commands.next < self._wpn:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class CheckLanding(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckLanding, self).__init__("Check Landing")
        self._vehicle = vehicle
        self._cmds = self._vehicle.commands
        self._cmds.download()

    def update(self):
        if self._cmds.count > 1 and self._cmds.next == self._cmds.count:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class SetCounter(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(SetCounter, self).__init__("Set WP Counter to %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        self._vehicle.commands.next = self._wpn
        return py_trees.common.Status.SUCCESS

class CheckMode(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckMode, self).__init__("Is mode %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        if self._vehicle.mode.name == self._mode_name:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class IsArmable(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(IsArmable, self).__init__('is_armable')
        self._vehicle = vehicle
        
    def update(self):
        if self._vehicle.is_armable:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

        
class ArmDrone(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(ArmDrone, self).__init__('Arm')
        self._vehicle = vehicle
        
    def update(self):
        self._vehicle.armed=True
        return py_trees.common.Status.SUCCESS
    

class SimpleTakeoff(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(SimpleTakeoff, self).__init__("Take off %i" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        print("TAKEOFF")
        self._vehicle.simple_takeoff(self._altitude)
        return py_trees.common.Status.SUCCESS


class PlaySound(py_trees.behaviour.Behaviour):

    def __init__(self, msg, voiceAst, returnFailure=None):
        super(PlaySound, self).__init__("Play sound %s" % msg)
        self._msg = msg
        self._voiceAst = voiceAst
        self._returnFailure = returnFailure

    def update(self):
        print("[WarningSound::update] Adding: \"" + self._msg + "\"")
        # Adding message to the Voice Assistant queue
        self._voiceAst.add_say(self._msg)

        if self._returnFailure:
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

class AltGlobalAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(AltGlobalAbove, self).__init__("Over %f (global)" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        if self._vehicle.location.global_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class AltLocalAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(AltLocalAbove, self).__init__("Over %f (local)" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        if self._vehicle.location.global_relative_frame.alt >= self._altitude:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class BatteryLevelAbove(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, level):
        super(BatteryLevelAbove, self).__init__(f"Battery over {level}% ?")
        self._vehicle = vehicle
        self._level = level
        
    def update(self):
        if self._vehicle.battery.level >= self._level:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation
    
class MoveDrone(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, dNorth, dEast, dDown):
        super(MoveDrone, self).__init__("Move (%f,%f,%f) NED" % (dNorth, dEast, dDown))
        self._vehicle = vehicle
        self._dNorth = dNorth
        self._dEast = dEast
        self._dDown = dDown    

    def update(self):
        current_loc = self._vehicle.location.global_frame
        target_loc = get_location_metres(current_loc, self._dNorth, self._dEast)
        target_loc.alt = target_loc.alt - self._dDown
        self._vehicle.simple_goto(target_loc)
        return py_trees.common.Status.SUCCESS



class LatSpeedUnder(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, max_speed):
        super(LatSpeedUnder, self).__init__("Lat speed < %f" % max_speed)
        self._vehicle = vehicle
        self._max_speed = max_speed

    def update(self):
        (vx,vy,vz) = self._vehicle.velocity
        current_speed = math.sqrt(vx*vx + vy*vy)
        if current_speed < self._max_speed:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

def preflight_Module(
        vehicle, 
        va,
        name="Pre-flight Module",
        safety_check=py_trees.behaviours.Dummy(name="Safety Check"),
        fallback=py_trees.behaviours.Dummy(name="Fallback")
        ):
    bt = py_trees.decorators.FailureIsRunning(py_trees.composites.Selector(name=name,children=[safety_check, fallback]), name=name)
    bt.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    return bt

def go_SAFTI(vehicle, va, safti_wp_n):
    bt = py_trees.composites.Sequence(name="Go SAFTI",
                                      children=[CheckCounterLessThan(vehicle, safti_wp_n),
                                                SetCounter(vehicle, safti_wp_n)])
    bt.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    return bt

def precond_module(
        va,
        name="Precond Module",
        safety_check=py_trees.behaviours.Dummy(name="Safety Check"),
        mishap_voice="Precond Fail",
        fallback=py_trees.behaviours.Dummy(name="Fallback"),
        wait=False):

    sq = py_trees.composites.Sequence(children=[PlaySound(mishap_voice, va)])
    if wait==True:
        timer=py_trees.timers.Timer()
        sq.add_children([timer, fallback])
        bt = py_trees.composites.Selector(name=name,children=[safety_check, py_trees.decorators.Inverter(py_trees.decorators.OneShot(sq))])

    else:
        sq.add_child(fallback)
        bt = py_trees.composites.Selector(name=name,children=[safety_check, py_trees.decorators.Inverter(py_trees.decorators.OneShot(sq))])
        
    return bt

def safety_module(
        va,
        name="Safety Module",
        safety_check=py_trees.behaviours.Dummy(name="Safety Check"),
        mishap_tts="Safety Fail",
        fallback=py_trees.behaviours.Dummy(name="Fallback")
        ):
    sq = py_trees.composites.Sequence(children=[PlaySound(mishap_tts, va), fallback])
    oneShot_sq = py_trees.decorators.OneShot(sq)
    bt = py_trees.composites.Selector(name=name,children=[safety_check, oneShot_sq])

    return bt

def take_off(vehicle, va):
    
    sq = py_trees.composites.Sequence(name="Is take-off\ncomplete?")
    finished_tko = py_trees.decorators.FailureIsRunning(CheckCounter(vehicle, 2))
    oneShot_playSound = py_trees.decorators.OneShot(PlaySound("Take-off completed", va))
    sq.add_children([CheckCounterLessThan(vehicle, 2), finished_tko, oneShot_playSound])

    sq.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    return sq


def landing(vehicle, va):

    sq = py_trees.composites.Sequence(name="landing Handler")
    landed = py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(AltLocalAbove(vehicle, 0.5)))
    oneShot_playSound = py_trees.decorators.OneShot(PlaySound("Landed", va))

    sq.add_children([CheckLanding(vehicle), landed, oneShot_playSound])

    lnd = py_trees.decorators.FailureIsRunning(sq, name="Is landing\ncomplete?")

    lnd.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL

    return lnd

class FlightManager:
    def __init__(self, vehicle, va, wp_count, safety_modules):
        super(FlightManager, self).__init__()
        self._vehicle = vehicle
        self._va = va
        self._wp_count = wp_count
        self._safety_modules = safety_modules
    
    def at_wp(self, wp_n):
        at_wp = py_trees.composites.Sequence(name=("Is vehicle at\n waypoint %i ?" % wp_n),
                                             children=[CheckCounter(self._vehicle, wp_n),
                                                       py_trees.decorators.FailureIsRunning(py_trees.decorators.Inverter(LatSpeedUnder(self._vehicle,1.0))),
                                                       CheckCounter(self._vehicle, wp_n),
                                                       py_trees.decorators.FailureIsRunning(LatSpeedUnder(self._vehicle, 1)),
                                                       CheckCounter(self._vehicle, wp_n),
                                                       PlaySound(("Waypoint %i" % wp_n), self._va)])
        at_wp.blackbox_level = py_trees.common.BlackBoxLevel.DETAIL
        return at_wp

    def leg_handler(self, wp_n, increment, precond_next_wp=None):

        wait_then_set_ctr = py_trees.composites.Sequence(name="Wait then Set CTR to %i"  % (wp_n+int(increment)))
        wait_then_set_ctr.add_children([py_trees.timers.Timer(duration=2.0), SetCounter(self._vehicle, (wp_n+int(increment)))])
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
                rtk_check = precond_module(self._va, name="RTK Check", safety_check=CheckGPS(self._vehicle, 5), mishap_voice="No RTK fix", fallback=go_SAFTI(self._vehicle, self._va, (self._wp_count-2)), wait=True)
                clearance_check = precond_module(self._va, name="Clearance > {}?".format(int(clearance[wp_n])), safety_check=CheckObstacle(self._vehicle, clearance[wp_n]), mishap_voice="Clearance Fail", fallback=go_SAFTI(self._vehicle, self._va, (self._wp_count-2)))
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
    
