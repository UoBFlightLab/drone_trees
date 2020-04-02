import py_trees
import time
import socket
import sys
from drone_trees import *
from dronekit import connect
import pyttsx3
from queue import Queue
import threading
import signal

connection_string = 'udp:127.0.0.1:14551'

class GroundControlAutomation:
    def __init__(self, vehicle):
        super(GroundControlAutomation, self).__init__()
        # Vehicle instance
        self.vehicle=vehicle
        # Initialising the Voice Assistant
        self.va = VoiceAssistant()
        # Starting Voice Assistant Thread
        self.va.start()

        # Generate executable mission file
        self.missionUtility = MissionUtility(self.vehicle)
        input_mission_filename = os.path.join('mission', 'input_mission.txt')
        self.wp_count = self.missionUtility.gen_exe_mission(input_mission_filename, 7, 10)
        self.SAFTI = self.wp_count-2 # SAFTI waypoint number

        self._loop_should_exit = False
    
    def cleanup(self):
        self._loop_should_exit = True
        self.va.kill()
        #self.va.join()
        #self.logger.terminate()
        self.vehicle.close()

    def behaviourTree(self):
        
        input_mission_filename = os.path.join('mission', 'executable_mission.txt')
        # pre-flight

        preflight_GPS_Check = preflight_Module(self.vehicle, self.va, 
                                                name="Preflight GPS Check",
                                                safety_check=CheckGPS(self.vehicle, 4),
                                                fallback=PlaySound('No RTK', self.va, returnFailure=True))

        preflight_EKF_Check = preflight_Module(self.vehicle, self.va, 
                                                name="Preflight EKF Check",
                                                safety_check=CheckEKF(self.vehicle),
                                                fallback=PlaySound('Bad EKF', self.va, returnFailure=True))

        preflight = py_trees.composites.Sequence(name="Pre-flight",
                                                 children=[preflight_GPS_Check,
                                                           preflight_EKF_Check,
                                                           MissionUpload(self.vehicle, self.missionUtility)])
        # Flight Manager

        safety_low_battery = safety_module(self.va, name="Low Battery",
                                   safety_check=BatteryLevelAbove(self.vehicle, 30),
                                   mishap_tts="Low battery", 
                                   fallback=go_SAFTI(self.vehicle, self.va, self.SAFTI))

        safety_obstacle_check = safety_module(self.va, name="Obstacle Check", 
                                      safety_check=CheckObstacle(self.vehicle, 2),
                                      mishap_tts="Obstacle ahead",
                                      fallback=go_SAFTI(self.vehicle, self.va, self.SAFTI))


        flight_manager = fm_behaviour(self.vehicle, self.va, self.wp_count, safety_modules=[safety_low_battery, safety_obstacle_check])

        # Root
        root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="OPS",
                                                                        children=[preflight,
                                                                                  take_off(self.vehicle, self.va),
                                                                                  flight_manager,
                                                                                  landing(self.vehicle, self.va)]))
        # piccies
        py_trees.display.render_dot_tree(root, name='Sim_Demo')

        return root

    def run(self, root):
        #self.logger = log(root, sys.argv[1])

        # tree
        behaviour_tree = py_trees.trees.BehaviourTree(root)
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        behaviour_tree.visitors.append(snapshot_visitor)

        while not self._loop_should_exit:
            print("******** {} *********".format(behaviour_tree.count))
            print(self.vehicle.battery)
            print(self.vehicle.mode.name)
            print(self.vehicle.location.global_frame.alt)
            # now the tree bit
            print("+++++++++++++++++++++")
            behaviour_tree.tick()
            unicode_tree = py_trees.display.unicode_tree(behaviour_tree.root,
                                                    visited=snapshot_visitor.visited,
                                                    previously_visited=snapshot_visitor.visited)
            print(unicode_tree)
            # log
            #self.logger.logging(behaviour_tree.count, dot=True)
            # Check for successful landing before exiting the loop
            if behaviour_tree.root.children[0].children[-1].status == py_trees.common.Status.SUCCESS:
                self._loop_should_exit = True
            # pause
            time.sleep(1)
        
        self.cleanup()

def cleanup():
    sys.exit()

def handler(signum, frame):
    print('Signal handler called with signal', signum)
    cleanup()

# Register signal handler
signal.signal(signal.SIGINT, handler)

def main():
    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    try:
        vehicle = connect(connection_string, wait_ready=True)
    except socket.error as e:
        print(e)
        # proceed just with a blank object so I can render the tree
        vehicle=None
    
    gca = GroundControlAutomation(vehicle)

    # Build behaviour tree
    root = gca.behaviourTree()

    # Run GCA
    gca.run(root)

    vehicle.close()
    cleanup()


main()