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


class CheckRCLink(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckRCLink, self).__init__("Check RC Link?")
        self._vehicle = vehicle
        self._text = None
        self._failure = False
        self._success = True

    def checkMavlink(self):
        @self._vehicle.on_message('STATUSTEXT')
        def listener(dkself, name, msg):
            self._text = msg.text
    
    def checkFailureCond(self):
        if self._text == "Radio Failsafe - Continuing Auto Mode" or self._text == "PreArm: Radio failsafe on" or self._text == "Radio Failsafe" or self._text == "Radio Failsafe - Disarming":
            self._failure=True
            self._success=False

    def checkSuccessCond(self):
        if self._text == "Radio Failsafe Cleared":
            self._failure=False
            self._success=True

    def update(self):
        self.checkMavlink()
        self.checkFailureCond()
        self.checkSuccessCond()
        if self._failure:
            return py_trees.common.Status.FAILURE
        elif self._success:
            return py_trees.common.Status.SUCCESS


class CheckGPSGlitch(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle):
        super(CheckGPSGlitch, self).__init__("Check GPS Glitch?")
        self._vehicle = vehicle
        self._text = None
        self._failure = False
        self._success = True

    def checkMavlink(self):
        @self._vehicle.on_message('STATUSTEXT')
        def listener(dkself, name, msg):
            self._text = msg.text

    def checkFailureCond(self):
        if self._text == "GPS Glitch":
            self._failure=True
            self._success=False

    def checkSuccessCond(self):
        if self._text == "GPS Glitch cleared":
            self._failure=False
            self._success=True

    def update(self):
        self.checkMavlink()
        self.checkFailureCond()
        self.checkSuccessCond()
        if self._failure:
            return py_trees.common.Status.FAILURE
        elif self._success:
            return py_trees.common.Status.SUCCESS





root=py_trees.composites.Parallel()
root.add_children([CheckRCLink(vehicle), CheckGPSGlitch(vehicle)])

behaviour_tree = py_trees.trees.BehaviourTree(root)
snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)


for ii in range(1000):
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



#print(x)