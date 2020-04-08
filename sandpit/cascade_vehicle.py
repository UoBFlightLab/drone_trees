#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
cascade_vehicle.py:

Custom Vehicle subclass to add DISTANCE_SENSOR data.

Follows example at https://github.com/dronekit/dronekit-python/tree/master/examples/create_attribute
Documented at https://dronekit-python.readthedocs.io/en/stable/examples/create_attribute.html

"""

from dronekit import Vehicle


class DistanceSensor(object):
    """
    Distance sensor readings e.g. from LIDAR or SONAR
    
    The message definition is here: https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
    
    :param time_boot_ms:     Timestamp (milliseconds since system boot). 
    :param current_distance: sensor reading in cm
    :param id:               which sensor is it?
    """
    def __init__(self, time_boot_ms=None, current_distance=None, sensor_id=None):
        """
        DISTANCE_SENSOR object constructor.
        """
        self.time_boot_ms = time_boot_ms
        self.current_distance = current_distance
        self.id = sensor_id
        
    def __str__(self):
        """
        String representation used to print the RawIMU object. 
        """
        return "DISTANCE_SENSOR: time_boot_ms={},current_distance={},id={}".format(self.time_boot_ms,
                                                                                   self.current_distance,
                                                                                   self.id)

   
class CascadeVehicle(Vehicle):
    """
    Custom vehicle subclass created for the CASCADE project
    
    Customizations:
        - includes distance sensor attributes
    """
    def __init__(self, *args):
        super(CascadeVehicle, self).__init__(*args)

        # Create an array of distance sensors
        max_distance_sensors = 8
        self._distance_sensors = [DistanceSensor() for ii in range(max_distance_sensors)]

        # Create a message listener using the decorator.   
        @self.on_message('DISTANCE_SENSOR')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.
            
            The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object 
            and notifies observers.
            """
            self._distance_sensors[message.id].time_boot_ms = message.time_boot_ms
            self._distance_sensors[message.id].current_distance = message.current_distance
            self._distance_sensors[message.id].id = message.id
            
            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('distance_sensors', self._distance_sensors) 

    @property
    def distance_sensors(self):
        """
        Access the distance sensors of the drone

        Returns
        -------
        List of DistanceSensor objects
            v.distance_sensors[n] is the last data from sensor with id=n

        """
        return self._distance_sensors