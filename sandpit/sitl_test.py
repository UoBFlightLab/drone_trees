from dronekit import connect
import dronekit_sitl
import time

connection_string = 'tcp:127.0.0.1:14550'

use_own_sitl = False
if use_own_sitl:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

vehicle = connect(connection_string,wait_ready=True)
print('CONNECTED')

vehicle.parameters['SIM_SONAR_SCALE'] = 0.001
vehicle.parameters['RNGFND2_TYPE'] = 1
vehicle.parameters['RNGFND2_SCALING'] = 10
vehicle.parameters['RNGFND2_PIN'] = 0
vehicle.parameters['RNGFND2_MAX_CM'] = 5000
vehicle.parameters['RNGFND2_MIN_CM'] = 20
vehicle.parameters['RNGFND2_ ORIENT'] = 0

@vehicle.on_message('HEARTBEAT')
def heartbeat_callback(self, name, msg):
    print('HB')

@vehicle.on_message('DISTANCE_SENSOR')
def distance_callback(self, name, msg):
    print('Range is {}'.format(msg.current_distance))

@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
    print('RF: ',self.rangefinder.distance)

while True:
    print(vehicle.mode)
    print(vehicle.rangefinder.distance)
    time.sleep(1)
    
#sitl.stop() # terminates SITL