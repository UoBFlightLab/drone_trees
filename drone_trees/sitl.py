import platform
from time import sleep
from dronekit_sitl import SITL
from dronekit import connect, VehicleMode
from drone_trees.drone_tree_vehicle import DroneTreeVehicle

def start_sitl():
    """Launch a SITL using local copy of Copter 4,
    then set up the simulator for a rangefinder.
    Only works for Windows or Linux.  Including
    binary in the project is ugly, but only used
    for testing."""
    if platform.system()=='Linux':
        sitl_path='../sitl/linux/arducopter'
    elif platform.system()=='Windows':
        sitl_path='../sitl/windows/ArduCopter.exe'
    else:
        sitl_path=''
    sitl = SITL(sitl_path)
    sitl.launch(['--home=51.454531,-2.629158,589,353'])

    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    veh.parameters['SIM_SONAR_SCALE'] = 0.00001
    veh.parameters['RNGFND2_ORIENT'] = 0
    veh.parameters['RNGFND2_SCALING'] = 10
    veh.parameters['RNGFND2_PIN'] = 0
    veh.parameters['RNGFND2_TYPE'] = 1
    veh.parameters['RNGFND2_MAX_CM'] = 5000
    veh.parameters['RNGFND2_MIN_CM'] = 5000

    veh.parameters['SIM_BATT_VOLTAGE'] = 12.59
    veh.parameters['BATT_MONITOR'] = 4

    veh.parameters['TERRAIN_ENABLE'] = 0

    veh.parameters['ARMING_CHECK'] = 16384 # mission only
    veh.parameters['FRAME_CLASS'] = 2 # I'm a hex

    veh.close()

    sitl.stop()
    sitl.launch(['--home=51.454531,-2.629158,589,353'],
                use_saved_data=True,
                verbose=True)
    sitl.block_until_ready()

    return sitl


def test_sitl():
    """Test that the SITL launch function start_sitl works
    and that we can use it to access distance_sensors attributes"""
    sitl = start_sitl()
    veh = connect(sitl.connection_string(), vehicle_class=DroneTreeVehicle)

    for ii in range(20):
        sleep(1)
        print(ii)
        print(veh.distance_sensors[1])
        print(veh.battery)
        print(veh.mode.name)
        # check we can change battery levels
        if ii == 4:
            veh.parameters['SIM_BATT_VOLTAGE'] = 12.11
        # test fiddling lidar range
        rng_cm = (ii+10)*10.
        print(rng_cm)
        veh.parameters['SIM_SONAR_SCALE'] = 0.0238/rng_cm
        # test we can change mode
        if ii == 8:
            veh.mode = VehicleMode('GUIDED')
    veh.close()
    sitl.stop()

if __name__ == '__main__':
    test_sitl()