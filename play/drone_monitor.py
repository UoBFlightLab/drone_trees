# Import DroneKit-Python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math

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

    return targetlocation;

# Connect to the Vehicle.
connection_string = 'tcp:127.0.0.1:5760'
print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# and every second for five minutes, print stuff
for ii in range(300):
    print("******* %i ********" % ii)
    if ii==60:
        vehicle.mode = VehicleMode('RTL')
    print(vehicle.armed)
    print(vehicle.battery)
    print(vehicle.mode.name)
    print(vehicle.location.global_frame.alt)
    print(vehicle.velocity)
    print(vehicle.rangefinder.distance)
    print(vehicle.location.global_frame)
    # experiment with offsets
    target = get_location_metres(vehicle.location.global_frame,10,10)
    print(target)
    time.sleep(1)

# Close vehicle object before exiting script
vehicle.close()
