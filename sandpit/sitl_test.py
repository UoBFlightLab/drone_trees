from dronekit import connect
import dronekit_sitl
import time
from cascade_vehicle import CascadeVehicle

connection_string = 'tcp:127.0.0.1:14550'

use_own_sitl = True
if use_own_sitl:
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

vehicle = connect(connection_string,wait_ready=True,vehicle_class=CascadeVehicle)
print('CONNECTED')

while True:
    print(vehicle.mode)
    print(vehicle.distance_sensors[1])
    time.sleep(1)
    
# USE vehicle.close() in console to stop callbacks    
#sitl.stop() # terminates SITL