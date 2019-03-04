# Import DroneKit-Python
from dronekit import connect, VehicleMode
from pymavlink.mavutil import mavlink

# Connect to the Vehicle.
connection_string = 'tcp:127.0.0.1:5762'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Get some vehicle attributes (state)
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable
print " Location: ", vehicle.location.global_frame

#Create a message listener for all messages.
#@vehicle.on_message('*')
@vehicle.on_message('MISSION_ACK')
def listener(self, name, message):
    print 'message: %s' % message
    
#vehicle.commands.next=8

#vehicle.mode = VehicleMode("LAND")

print 'Downloading mission'
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
for mi in cmds:
    print mi.x,mi.y,mi.z

print 'Sending waypoint modification'
idx = 5 # note this is one-based for MAVlink
lat = cmds[idx-1].x
lon = cmds[idx-1].y
alt = cmds[idx-1].z
delay = 300
vehicle.message_factory.mission_write_partial_list_send(0, 0,
                                                       idx, # start index
                                                       idx) # end index
frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
vehicle.message_factory.mission_item_send(0, 0, idx, frame,
                    mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    delay, 0, 0, 0,
                    lat+1e-4, lon+1e-4, alt+5)

print 'Downloading mission'
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
for mi in cmds:
    print mi.x,mi.y,mi.z

#m = input('Press ENTER to stop')
    
# Close vehicle object before exiting script
vehicle.close()

#msg = vehicle.message_factory.command_long_encode(
#            0, 0,  # target system, target component
#            mavlink.MAV_CMD_MISSION_START,  # command
#            0,  # confirmation
#            1,  # param 1: first_item
#            6,  # param 2: last_item
#            0, 0, 0, 0, 0)  # params 3-7
# send command to vehicle
#vehicle.send_mavlink(msg)

#        self.send_mavlink(self.message_factory.command_long_encode(
#            0, 0,  # target system, target component
#            mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
#            0,  # confirmation
#            2,  # param 1: 1 to use current position, 2 to use the entered values.
#            0, 0, 0,  # params 2-4
#            pos.lat, pos.lon, pos.alt))
