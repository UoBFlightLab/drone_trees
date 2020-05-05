from dronekit import connect
import dronekit_sitl
import time
from drone_trees.drone_tree_vehicle import DroneTreeVehicle

connection_string = 'tcp:127.0.0.1:14550'

use_own_sitl = True
if use_own_sitl:
    sitl = dronekit_sitl.SITL(path='./sitl/apm.exe')
    sitl.launch(initial_args=['--home=51.454531,-2.629158,584,353'])
    connection_string = sitl.connection_string()

print('Connecting to {}'.format(connection_string))
vehicle = connect(connection_string,wait_ready=True,vehicle_class=DroneTreeVehicle)
print('CONNECTED')

# vehicle.parameters['SIM_SONAR_SCALE'] = 0.001
# vehicle.parameters['RNGFND2_TYPE'] = 1
# vehicle.parameters['RNGFND2_SCALING'] = 10
# vehicle.parameters['RNGFND2_PIN'] = 0
# vehicle.parameters['RNGFND2_MAX_CM'] = 5000
# vehicle.parameters['RNGFND2_MIN_CM'] = 20


# time.sleep(1)

# vehicle.reboot()

while True:
    print(vehicle.mode)
    print(vehicle.distance_sensors[1])
    time.sleep(1)
    
# USE vehicle.close() in console to stop callbacks    
#sitl.stop() # terminates SITL