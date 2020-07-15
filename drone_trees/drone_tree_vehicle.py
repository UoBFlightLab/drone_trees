# -*- coding: utf-8 -*-
###############################################################################
# License: MIT License
#    https://raw.githubusercontent.com/UoBFlightLab/drone_trees/master/LICENSE
###############################################################################
# Author: Hirad Goudarzi
# Role: PhD Candidate
# Organisation: University of Bristol
# Version: 2.0.0
# Email: hirad.goudarzi@bristol.ac.uk
###############################################################################
"""

drone_tree_vehicle.py:

Custom Vehicle subclass to add DISTANCE_SENSOR data.

Follows example at https://github.com/dronekit/dronekit-python/tree/master/examples/create_attribute
Documented at https://dronekit-python.readthedocs.io/en/stable/examples/create_attribute.html

"""
###############################################################################

from dronekit import Vehicle


class DistanceSensor():
    """
    Distance sensor readings e.g. from LIDAR or SONAR

    The message definition is here:
    https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR

    :param time_boot_ms:     Timestamp (milliseconds since system boot).
    :param current_distance: sensor reading in cm
    :param id:               which sensor is it?
    """
    def __init__(self, time_boot_ms=None,
                 current_distance=None,
                 sensor_id=None):
        """
        DISTANCE_SENSOR object constructor.
        """
        self.time_boot_ms = time_boot_ms
        self.current_distance = current_distance
        self.id = sensor_id

    def __str__(self):
        """
        String representation used to print the Distance Sensor measurements.
        """
        return (f"DISTANCE_SENSOR: time_boot_ms={self.time_boot_ms},"
                f"current_distance={self.current_distance},id={self.id}")


class StatusText():
    """
    Status text message. These messages are printed in yellow in the COMM
    console of QGroundControl.

    The message definition is here:
    https://mavlink.io/en/messages/common.html#STATUSTEXT

    :text:      Status text message, without null termination character
    :severity: 	Severity of status. Relies on the definitions within RFC-5424.
    """
    def __init__(self, text=None, severity=None):
        """
        STATUSTEXT object constructor.
        """
        self.text = text
        self.severity = severity

    def __str__(self):
        """
        String representation used to print the Status Text.
        """
        return f"STATUSTEXT: text={self.text},severity={self.severity}"


class DroneTreeVehicle(Vehicle):
    """
    Custom vehicle subclass created for the drone_trees project.

    Requires copter 4.04 (note that dronekit_sitl latest is 3.3, which does not
                          support DISTANCE_SENSOR messages)

    Customizations:
        - includes distance sensor attributes
    """
    def __init__(self, *args):
        super(DroneTreeVehicle, self).__init__(*args)

        # Create an array of distance sensors
        max_distance_sensors = 8
        self._distance_sensors = [DistanceSensor() for ii in range(max_distance_sensors)]

        # Status text message
        self._statustext = StatusText()

        # Acknowledgment message during waypoint handling
        # https://mavlink.io/en/messages/common.html#MISSION_ACK
        self._mission_ack_type = None

        # Updates when a mission item is completed
        self._mission_item_reached = None

        # Create a message listener using the decorator.
        @self.on_message('DISTANCE_SENSOR')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string
            specified in the decorator, passing the vehicle, message name, and
            the message.

            The listener writes the message to the (newly attached)
            ``vehicle.raw_imu`` object and notifies observers.
            """
            self._distance_sensors[message.id].time_boot_ms = message.time_boot_ms
            self._distance_sensors[message.id].current_distance = message.current_distance
            self._distance_sensors[message.id].id = message.id

            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('distance_sensors',
                                            self._distance_sensors)

        @self.on_message('STATUSTEXT')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string
            specified in the decorator, passing the vehicle, message name, and
            the message.
            """
            self._statustext.text = message.text
            self._statustext.severity = message.severity

            # Notify all observers of new message (with new value)
            self.notify_attribute_listeners('statustext', self._statustext)

        @self.on_message('MISSION_ACK')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string
            specified in the decorator, passing the vehicle, message name, and
            the message.
            """
            self._mission_ack_type = message.type

            # Notify all observers of new message (with new value)
            self.notify_attribute_listeners('mission_ack_type',
                                            self._mission_ack_type)

        @self.on_message('MISSION_ITEM_REACHED')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string
            specified in the decorator, passing the vehicle, message name, and
            the message.
            """
            self._mission_item_reached = message.seq

            # Notify all observers of new message (with new value)
            self.notify_attribute_listeners('mission_item_reached',
                                            self._mission_item_reached)

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

    @property
    def statustext(self):
        """
        Access the status text of the drone

        Returns
        -------
        status text object

        """
        return self._statustext

    @property
    def mission_ack_type(self):
        """
        Access Mission Acknowledgment type during mission handling
        type number defined in the link below:
        https://mavlink.io/en/messages/common.html#MAV_MISSION_RESULT

        Returns
        -------
        int describing the mission ack according to the link above

        """
        return self._mission_ack_type

    @property
    def mission_item_reached(self):
        """
        Access the sequence number of the last reached mission item. In an AUTO
        mission this number represents the index of the reached waypoint
        command and it updates when a mission command item is completed

        The message definition is here:
        https://mavlink.io/en/messages/common.html#MISSION_ITEM_REACHED

        Returns
        -------
        int sequence number of the last reached mission item

        """
        return self._mission_item_reached
