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


class RCChannels():
    """
    RC input channels

    The message definition is here:
    https://mavlink.io/en/messages/common.html#RC_CHANNELS

    :time_boot_ms:  Timestamp (time since system boot).
    :chancount:     Total number of RC channels being received. This can be
                    larger than 18, indicating that more channels are available
                    but not given in this message. This value should be 0 when
                    no RC channels are available.
    : chan_raw:     A dictionary containing values of RC channels 1 to 18
    """
    def __init__(self, time_boot_ms=None, chancount=None,
                 chan_raw=dict.fromkeys(range(1, 18, 1))):
        """
        RC_CHANNELS object constructor.
        """
        self.time_boot_ms = time_boot_ms
        self.chancount = chancount
        self.chan_raw = chan_raw

    def __str__(self):
        """
        String representation used to print the RC input.
        """
        return (f"RC_CHANNELS : time_boot_ms={self.time_boot_ms}"
                f",chancount={self.chancount}\n{self.chan_raw}")


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

        # RC inputs
        self._rcin = RCChannels()

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

        @self.on_message('RC_CHANNELS')
        def listener(self, name, m):
            """
            The listener is called for messages that contain the string
            specified in the decorator, passing the vehicle, message name, and
            the message.
            """
            self._rcin.time_boot_ms = m.time_boot_ms
            self._rcin.chancount = m.chancount
            self._rcin.chan_raw = {
                1: m.chan1_raw,
                2: m.chan2_raw,
                3: m.chan3_raw,
                4: m.chan4_raw,
                5: m.chan5_raw,
                6: m.chan6_raw,
                7: m.chan7_raw,
                8: m.chan8_raw,
                9: m.chan9_raw,
                10: m.chan10_raw,
                11: m.chan11_raw,
                12: m.chan12_raw,
                13: m.chan13_raw,
                14: m.chan14_raw,
                15: m.chan15_raw,
                16: m.chan16_raw,
                17: m.chan17_raw,
                18: m.chan18_raw
            }
            # Notify all observers of new message (with new value)
            self.notify_attribute_listeners('rcin', self._rcin)

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
    def rc_channels(self):
        """
        Access the rc input channels

        Returns
        -------
        RCChannels object

        """
        return self._rcin

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
