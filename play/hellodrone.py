# Import DroneKit-Python
from dronekit import connect, VehicleMode
import time

# Connect to the Vehicle.
connection_string = 'tcp:127.0.0.1:5760'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# set the frame class http://ardupilot.org/copter/docs/parameters.html
vehicle.parameters['FRAME_CLASS']=1 # quad
vehicle.parameters['FRAME_TYPE']=0 # plus

# disable vehicle arming checks to prevent accel hangup
vehicle.parameters['ARMING_CHECK']=0

# set to always land if RC lost
# http://ardupilot.org/copter/docs/parameters.html
vehicle.parameters['FS_THR_ENABLE']=3

# mode to guided so can arm and take off
vehicle.mode = VehicleMode('GUIDED')

# prepare to print altitude reports
@vehicle.on_attribute('location')
def location_callback(self, attr_name, value):
    print("Altitude : %s" % value.global_relative_frame.alt)

# wait for armable
for ii in range(300):
    if vehicle.is_armable:
        break
    time.sleep(1)
    
print('Arming')
vehicle.armed=True
print('Taking off to alt 20m')
vehicle.simple_takeoff(20)

# wait for a bit
time.sleep(40)

# now throw something at it... 
# http://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
print('Simulating RC link loss')
vehicle.parameters['SIM_RC_FAIL']=1

# wait for a bit longer and see what happens
time.sleep(60)

# Close vehicle object before exiting script
vehicle.close()
