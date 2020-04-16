import py_trees
import time
import socket
import drone_trees as dt
from dronekit import connect
from distance_sensor_vehicle import DistanceSensorVehicle
import dronekit_sitl
import sys
#from voice_assistant import VoiceAssistant

connection_string = 'tcp:127.0.0.1:14550'

input_mission_filename = 'input_mission.txt'

class GroundControlAutomation:
    def __init__(self, vehicle, voice_asst = None):
        super(GroundControlAutomation, self).__init__()
        # Vehicle instance
        self.vehicle=vehicle
        # Initialising the Voice Assistant
        self.va = voice_asst
        # Starting Voice Assistant Thread
        if self.va:
            self.va.start()
        # Generate executable mission file
        self.missionUtility = dt.MissionUtility(self.vehicle)
        self.wp_count = self.missionUtility.gen_exe_mission(input_mission_filename, 7, 10)
        self.SAFTI = self.wp_count-2 # SAFTI waypoint number

        self._loop_should_exit = False
    
    def cleanup(self):
        self._loop_should_exit = True
        if self.va:
            self.va.kill()
        #self.va.join()
        #self.logger.terminate()
        self.vehicle.close()

    def behaviourTree(self):
        
        # pre-flight

        preflight_GPS_Check = dt.preflight_Module(self.vehicle, self.va, 
                                                name="Preflight GPS Check",
                                                safety_check=dt.CheckGPS(self.vehicle, 4),
                                                fallback=dt.PlaySound('No RTK', self.va, returnFailure=True))

        preflight_EKF_Check = dt.preflight_Module(self.vehicle, self.va, 
                                                name="Preflight EKF Check",
                                                safety_check=dt.CheckEKF(self.vehicle),
                                                fallback=dt.PlaySound('Bad EKF', self.va, returnFailure=True))

        preflight = py_trees.composites.Sequence(name="Pre-flight",
                                                 children=[preflight_GPS_Check,
                                                           preflight_EKF_Check,
                                                           dt.MissionUpload(self.vehicle, self.missionUtility)])
        # Flight Manager

        safety_low_battery = dt.safety_module(self.va, name="Low Battery",
                                   safety_check=dt.BatteryLevelAbove(self.vehicle, 30),
                                   mishap_tts="Low battery", 
                                   fallback=dt.go_SAFTI(self.vehicle, self.va, self.SAFTI))

        safety_obstacle_check = dt.safety_module(self.va, name="Obstacle Check", 
                                      safety_check=dt.CheckObstacle(self.vehicle, 2),
                                      mishap_tts="Obstacle ahead",
                                      fallback=dt.go_SAFTI(self.vehicle, self.va, self.SAFTI))


        flight_manager = dt.fm_behaviour(self.vehicle, self.va, self.wp_count, safety_modules=[safety_low_battery, safety_obstacle_check])

        # Root
        root = py_trees.decorators.OneShot(py_trees.composites.Sequence(name="OPS",
                                                                        children=[preflight,
                                                                                  dt.take_off(self.vehicle, self.va),
                                                                                  flight_manager,
                                                                                  dt.landing(self.vehicle, self.va)]))
        # piccies
        #py_trees.display.render_dot_tree(root, name='Sim_Demo')

        return root

    def run(self, root):
        #self.logger = log(root, sys.argv[1])

        # tree
        behaviour_tree = py_trees.trees.BehaviourTree(root)
        
        #snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        #behaviour_tree.visitors.append(snapshot_visitor)

        behaviour_tree.visitors.append(py_trees.visitors.DebugVisitor())

        while not self._loop_should_exit:
            print("******** {} *********".format(behaviour_tree.count))
            print(self.vehicle.battery)
            print(self.vehicle.mode.name)
            print(self.vehicle.location.global_frame.alt)
            # now the tree bit
            print("+++++++++++++++++++++")
            behaviour_tree.tick()
            # unicode_tree = py_trees.display.unicode_tree(behaviour_tree.root,
            #                                        visited=snapshot_visitor.visited,
            #                                        previously_visited=snapshot_visitor.visited)
            #print(unicode_tree)
            # log
            #self.logger.logging(behaviour_tree.count, dot=True)
            # Check for successful landing before exiting the loop
            if behaviour_tree.root.children[0].children[-1].status == py_trees.common.Status.SUCCESS:
                print("Flight completed")
                self._loop_should_exit = True
            else:
                print("******** {} *********".format(behaviour_tree.count))
           
            # pause
            time.sleep(1)
        
        self.cleanup()

def main():
    
    if len(sys.argv)>1:
        sitl = None
        connection_string = sys.argv[1]
        print("Attempting to connect via {}".format(connection_string))
    else:
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        print("Using SITL via {}".format(connection_string))

    try:
        vehicle = connect(connection_string, wait_ready=True, vehicle_class=DistanceSensorVehicle)
    except socket.error as e:
        print(e)
        # proceed just with a blank object so I can render the tree
        vehicle=None
    
    gca = GroundControlAutomation(vehicle)
    #gca = GroundControlAutomation(vehicle,VoiceAssistant())

    # Build behaviour tree
    root = gca.behaviourTree()

    # Run GCA
    gca.run(root)

    vehicle.close()
    if sitl:
        sitl.stop()

if __name__ == "__main__":
    main()