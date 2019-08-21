import py_trees
import time
import socket
#from drone_trees import *
from battery_caller import battery_caller
from dronekit import connect
from pymavlink import mavutil
import pyttsx3
from queue import Queue
import threading
"""
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)


class mavlinkTest(py_trees.behaviour.Behaviour):

    def __init__(self, vehicle, m):
        # use name of mode as label for behaviour
        super(mavlinkTest, self).__init__(m)
        self._vehicle = vehicle
        self._m = m
        self._alt = None
        @vehicle.on_message(m)
        def listener(self, name, m):
            self._alt = m.alt

    def update(self):
        print("+++++++++++++++++++++")
        print(self._alt)
        print("+++++++++++++++++++++")
        return py_trees.common.Status.SUCCESS


root = mavlinkTest(vehicle, 'GLOBAL_POSITION_INT')

# tree
behaviour_tree = py_trees.trees.BehaviourTree(root)

snapshot_visitor = py_trees.visitors.SnapshotVisitor()
behaviour_tree.visitors.append(snapshot_visitor)

ascii_tree = py_trees.display.ascii_tree(behaviour_tree.root, visited=snapshot_visitor.visited, previously_visited=snapshot_visitor.visited)
print(ascii_tree)


x=None
@vehicle.on_message('GLOBAL_POSITION_INT')
def listener(self, name, m, x):
    x=m.alt
    print(x)


class mavlinkTest(object):

    def __init__(self, vehicle):
        # use name of mode as label for behaviour
        super(mavlinkTest, self).__init__()
        self._vehicle = vehicle
        self._alt = None
        @vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(self, name, m):
            print(m.alt)

a = mavlinkTest(vehicle)



class mavlinkTest(object):
    def __init__(self, vehicle):
        super(mavlinkTest, self).__init__()
        self._vehicle = vehicle
        self._alt = None
            
        @vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(self, name, ms):
            self._alt = ms.alt
        print(self._alt)

a = mavlinkTest(vehicle)
print(a._alt)

vehicle.close()
"""


class VoiceAssistant(threading.Thread):
    def __init__(self):
        super(VoiceAssistant, self).__init__()
        self.engine = pyttsx3.init()
        self.q = Queue()
        self.daemon = True

    def add_say(self, msg):
        self.q.put(msg)

    def run(self):
        while True:
            self.engine.say(self.q.get())
            self.engine.startLoop(False)
            self.engine.iterate()
            time.sleep(1)
            #self.engine.runAndWait()
            self.engine.endLoop()
            self.q.task_done()


if __name__ == '__main__':
    va = VoiceAssistant()
    va.start()
    for i in range(0, 3):
        va.add_say('Sally sells seashells by the seashore.')
    print("now we want to exit...")
    va.q.join() # ends the loop when queue is empty