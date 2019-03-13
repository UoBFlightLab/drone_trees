from pymavlink.dialects.v10 import common as mavlink
from pymavlink import mavutil

con = mavutil.mavlink_connection('tcp:localhost:5762')

#print con.waypoint_current()

con.waypoint_set_current_send(3)
