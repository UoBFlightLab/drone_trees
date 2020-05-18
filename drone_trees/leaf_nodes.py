import py_trees
import math
import os
from dronekit import VehicleMode, LocationGlobal, LocationGlobalRelative

class MissionUpload(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wplist):
        super(MissionUpload, self).__init__('Mission Upload')
        self._vehicle = vehicle
        self._wplist = wplist
        
    def update(self):
        # only permitted to upload if unarmed
        if self._vehicle.armed:
            return py_trees.common.Status.FAILURE
        else:
            cmds = self._vehicle.commands
            cmds.clear()
            # add a dummy command on the front
            # just the first WP used as a placeholder
            # upload seems to skip the first WP
            cmds.add(self._wplist[0])
            for wp in self._wplist:
                cmds.add(wp)
            cmds.upload()
            self.feedback_message = 'Uploaded {} WPs'.format(cmds.count)
            return py_trees.common.Status.SUCCESS

# class MissionVerify(py_trees.behaviour.Behaviour):

#     def __init__(self, vehicle, wplist):
#         super(MissionVerify, self).__init__('Mission Verify')
#         self._vehicle = vehicle
#         self._wplist = wplist[:]

#     def initialise(self):
#         self._vehicle.commands.clear()
#         self._vehicle.commands.download()
        
#     def update(self):
#         if self._vehicle.commands.count < len(self._wplist):
#             self.feedback_message = 'Got {} of {} WPs'.format(self._vehicle.commands.count,
#                                                              len(self._wplist))
#             return py_trees.common.Status.RUNNING
#         else:
#             cmd_list = [cmd for cmd in self._vehicle.commands]
#             if cmd_list[1:]==self._wplist[:]:
#                 return py_trees.common.Status.SUCCESS
#             else:
#                 return py_trees.common.Status.FAILURE
                

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


class Land(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(Land, self).__init__()
        self._vehicle = vehicle
        self._mode = VehicleMode('RTL')

    def update(self):
        self._vehicle.mode = self._mode
        return py_trees.common.Status.SUCCESS


class CheckGPS(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, fixType):
        super(CheckGPS, self).__init__("GPS Status Check > %i?" % fixType)
        self._vehicle = vehicle
        self._fixType = fixType

    def update(self):
        gps_fix = self._vehicle.gps_0.fix_type
        if gps_fix > self._fixType:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No,only {}'.format(gps_fix)
            return py_trees.common.Status.FAILURE


# class CheckObstacle(py_trees.behaviour.Behaviour):

#     def __init__(self, vehicle, clearance):
#         super(CheckObstacle, self).__init__("Clearance > %i ?" % clearance)
#         self._vehicle = vehicle
#         self._clearance = clearance
#         self._distance = None

#     def checkMavlink(self):
#         @self._vehicle.on_message('DISTANCE_SENSOR')
#         def listener(dkself, name, msg):
#             self._distance = msg.current_distance

#     def update(self):
#         while self._distance==None:
#             self.checkMavlink()
#         if self._distance/100 > self._clearance:
#             return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.FAILURE
        
        
class CheckDistance(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, sensor_id, clearance):
        super(CheckDistance, self).__init__("Distance {} > {} ?".format(sensor_id,clearance))
        self._vehicle = vehicle
        self._sensor_id = sensor_id
        self._clearance = clearance

    def update(self):
        current_dist = self._vehicle.distance_sensors[self._sensor_id].current_distance
        if current_dist==None:
            self.feedback_message = 'Nothing from sensor {}'.format(self._sensor_id)
            return py_trees.common.Status.FAILURE
        elif current_dist/100. > self._clearance:
            self.feedback_message = 'Yes, it''s {}'.format(current_dist/100.)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}'.format(current_dist/100.)
            return py_trees.common.Status.FAILURE
        
class CheckEKF(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckEKF, self).__init__("EKF healthy?")
        self._vehicle = vehicle

    def update(self):
        if self._vehicle.ekf_ok:
            self.feedback_message = 'Healthy'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'Bad EKF'
            return py_trees.common.Status.FAILURE

class CheckCounter(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(CheckCounter, self).__init__("Is Counter {}?".format(wpn))
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        next_wp = self._vehicle.commands.next
        if self._wpn == next_wp:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}'.format(next_wp)
            return py_trees.common.Status.FAILURE

class CheckCounterLessThan(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, wpn):
        super(CheckCounterLessThan, self).__init__("Check Counter < %i" % wpn)
        self._vehicle = vehicle
        self._wpn = wpn

    def update(self):
        next_wp = self._vehicle.commands.next
        if next_wp < self._wpn:
            self.feedback_message = 'Yes, it''s {}.'.format(next_wp)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}.'.format(next_wp)
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
        super(SetCounter, self).__init__("Set WP Counter to %i" % int(wpn))
        self._vehicle = vehicle
        self._wpn = int(wpn)

    def update(self):
        cur_wpn = self._vehicle.commands.next 
        if cur_wpn < self._wpn:
            self.feedback_message = 'Advancing WP counter from {} to {}'.format(cur_wpn,self._wpn)
            print(self.feedback_message)
            self._vehicle.commands.next = self._wpn
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'Cannot move WP counter backwards from {} to {}'.format(cur_wpn,self._wpn)
            return py_trees.common.Status.FAILURE
            

class CheckMode(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckMode, self).__init__("Is mode %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        current_mode = self._vehicle.mode.name
        if current_mode == self._mode_name:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, it''s {}'.format(current_mode)
            return py_trees.common.Status.FAILURE


class CheckModeNot(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, mode_name):
        # use name of mode as label for behaviour
        super(CheckModeNot, self).__init__("Is mode NOT %s ?" % mode_name)
        # in time may want to delay connection
        # and use setup method instead
        self._vehicle = vehicle
        self._mode_name = mode_name

    def update(self):
        current_mode = self._vehicle.mode.name
        if current_mode == self._mode_name:
            self.feedback_message = 'Fail: mode is {}'.format(current_mode)
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = 'OK, it''s {}'.format(current_mode)
            return py_trees.common.Status.SUCCESS
        

class IsArmable(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(IsArmable, self).__init__('Can drone be armed?')
        self._vehicle = vehicle
        
    def update(self):
        if self._vehicle.is_armable:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No'
            return py_trees.common.Status.FAILURE

        
class IsArmed(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(IsArmed, self).__init__('Is drone armed?')
        self._vehicle = vehicle
        
    def update(self):
        if self._vehicle.armed:
            self.feedback_message = 'Yes'
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No'
            return py_trees.common.Status.FAILURE
        
        
# class ArmDrone(py_trees.behaviour.Behaviour):

#     def __init__(self, vehicle):
#         super(ArmDrone, self).__init__('Arm')
#         self._vehicle = vehicle
        
#     def update(self):
#         self._vehicle.armed=True
#         return py_trees.common.Status.SUCCESS
    

class SimpleTakeoff(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, altitude):
        super(SimpleTakeoff, self).__init__("Take off %i" % altitude)
        self._vehicle = vehicle
        self._altitude = altitude
        
    def update(self):
        self._vehicle.simple_takeoff(self._altitude)
        self.feedback_message = 'Takeoff and hold {}'.format(self._altitude)
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
        if self._voiceAst:
            self._voiceAst.add_say(self._msg)
        else:
            print("Muted")
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
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180.))
    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180./math.pi)
    newlon = original_location.lon + (dLon * 180./math.pi)
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
        super(LatSpeedUnder, self).__init__("Lat speed < {}?".format(max_speed))
        self._vehicle = vehicle
        self._max_speed = max_speed

    def update(self):
        (vx,vy,vz) = self._vehicle.velocity
        current_speed = math.sqrt(vx*vx + vy*vy)
        if current_speed < self._max_speed:
            self.feedback_message = 'Yes, speed is {}'.format(current_speed)
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = 'No, speed is {}'.format(current_speed)
            return py_trees.common.Status.FAILURE
        
        
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

