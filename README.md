# drone_trees
Like all good cartoons, this library brings together two good stories:
* [DroneKit Python](https://github.com/dronekit/dronekit-python) offering easy Python interfacing to a MAVlink-controlled drone
* [py_trees](https://github.com/splintered-reality/py_trees) implementing behaviour trees in Python

The work was inspired by [Increasing Modularity of UAV Control Systems using Computer Game Behavior Trees](https://arc.aiaa.org/doi/pdf/10.2514/6.2012-4458) by Petter Ogren, and follows on from our experience using behaviour trees in the [Venturer](https://www.venturer-cars.com/) driverless car project at BRL.  Behaviour trees make it really easy to mess around with mission scripting in a modular way.  If you need to insert some extra checking or stages in an operation, you can do so without touching the parts you've made already.

## Installation
drone_trees is compatible with python 3.7 and it needs the following dependencies:
* `dronekit-python` version 2.9.1 or higher
* `py-trees` version 2.0.0 or higher
* Optional `pyttsx3` V2.87 for voice feedback. If using windows `pypiwin32` is also needed.

To install:
* On Linux, `pip install py_trees dronekit dronekit_sitl` (recommended in a virtual environment) and then local install of drone_trees
* On Windows, in the root folder of this repository, `conda env create drone_trees.yml` followed by `conda develop .` 

## Examples

### Fly a square

This example uses a hand-crafted behaviour tree to control a drone in GUIDED mode through a simple flight round a square.

To simulate the whole thing automatically, run the `test_square.py` script.  To see more, connect your favourite ground station software to the extra port on the SITL, usually `tcp:127.0.0.1:5762` but check the output as the port number increments by ten each run.

If you'd rather manage the sim yourself or even fly the thing (!), run `python fly_square.py <conn>` where `<conn>` is the connection to the mavlink stream.  For example, if you run a SITL through Mission Planner, hit Ctrl+F, then the Mavlink button, and enable 'TCP host' with write access.  Then, if you agreed to the defaults, run `python fly_sqare.py tcp:127.0.0.1:14550`.  You'll need to put the drone in 'GUIDED' and arm it through the ground station as the behaviour tree will never arm or change mode.

![Behaviour tree](https://raw.githubusercontent.com/UoBFlightLab/drone_trees/master/examples/square/fly_square.py.png)

### Fly a waypoint mission

This example is based on the CASCADE BRIDGE case study.  Use the `test_bridge.py` script to run it all automatically, or use `python fly_bridge.py <conn>` as described above to connect to a drone, ground control software or simulator.

This example uses the 'flight_idioms' tools to construct a behaviour tree in a standard form, avoiding the need for the user to work directly with the `py_trees` library.  The drone performs an extensive pre-flight, then waits for the pilot to arm it, put it in AUTO mode and initiate take-off by throttling up.  (These latter three activities are automated in the `test_bridge.py` script.)  Then for the rest of the mission:

* the drone will instantly return home, via a SAFTI point 50m from the bridge, if the battery drops below 30% or the EKF goes unhealthy at any time

* each mission phase depends on a specific battery level

![Behaviour tree](https://raw.githubusercontent.com/UoBFlightLab/drone_trees/master/examples/bridge/cusersaeagronedrive_-_university_of_bristoldocumentspythondrone_treesexamplesbridgefly_bridge.py.png)
