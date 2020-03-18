import time
import socket
import sys
from dronekit import connect
import py_trees



# Connect to the Vehicle.
connection_string = 'udp:127.0.0.1:14551'

print("Connecting to vehicle on: %s" % (connection_string,))
try:
    vehicle = connect(connection_string, wait_ready=True)
except socket.error as e:
    print(e)
    # proceed just with a blank object so I can render the tree
    vehicle=None


x=None
"""
@vehicle.on_message('STATUSTEXT')
def my_method(self, name, msg):
    global x
    x = msg
    print(x)
"""
"""
class Bollocks(object):
    def __init__(self, val):
        self._val = val
        self._anotherval = None

    @property
    def val(self):
        return self._val

    @property
    def anotherval(self):
        if not self._anotherval:
            self._anotherval = 2 * self.val
        return self._anotherval

    def __sub__(other):
        return other.val - self.val
    
print(Bollocks(1) - Bollocks(2))

"""     


class CheckRC(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckRC, self).__init__("RC1")
        self._vehicle = vehicle
        self._rc = None

    def mavlink_check(self):
        @self._vehicle.on_message('RC_CHANNELS')
        def listener(dkself, name, msg):
            self._rc = msg.chan1_raw


    def update(self):
        while self._rc == None:
            self.mavlink_check()

        if self._rc > 100:
            print(self._rc)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class CheckGPSGlitch(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckGPSGlitch, self).__init__("Glitch")
        self._vehicle = vehicle
        self._severity = None
        self._text = None

    def mavlinkCheck(self):
        @self._vehicle.on_message('STATUSTEXT')
        def listener(dkself, name, msg):
            self._severity = msg.severity
            self._text = msg.text


    def update(self):
        self.mavlinkCheck()
        if self._severity == None:
            return py_trees.common.Status.SUCCESS
        elif self._severity < 3 and self._text == "GPS Glitch":
            print(self._severity)
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS





root=py_trees.composites.Sequence()
root.add_children([py_trees.timers.Timer(duration=1), CheckGPSGlitch(vehicle)])

behaviour_tree = py_trees.trees.BehaviourTree(root)
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)


for ii in range(50):
    print("******* %i ********" % ii)
    print(vehicle.battery)
    print(vehicle.mode.name)
    print(vehicle.location.global_frame.alt)
    print(vehicle.velocity)
    # now the tree bit
    print("+++++++++++++++++++++")
    behaviour_tree.tick()
    unicode_tree = py_trees.display.unicode_tree(behaviour_tree.root,
                                            visited=snapshot_visitor.visited,
                                            previously_visited=snapshot_visitor.visited)
    print(unicode_tree)
    # pause
    time.sleep(1)


print(x)