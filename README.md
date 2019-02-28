# drone_trees
Like all good cartoons, this library brings together two good stories:
* [DroneKit Python](http://python.dronekit.io/) offering easy Python interfacing to a MAVlink-controlled drone
* [py_trees](https://github.com/splintered-reality/py_trees) implementing behaviour trees in Python
The work was inspired by [Increasing Modularity of UAV Control Systems using Computer Game Behavior Trees](https://arc.aiaa.org/doi/pdf/10.2514/6.2012-4458) by Petter Ogren, and folows on from our experience using behaviour trees in the [Venturer](https://www.venturer-cars.com/) driverless car project at BRL.  Behaviour trees make it really easy to mess around with mission scripting in a modular way.  If you need to insert some extra checking or stages in an operation, you can do so without touching the parts you've made already.
## Example
This has all been tested on Windows running everything under Cygwin, although the Python stuff should work just as well on any Python environment.  You'll need ardupilot, py_trees and DroneKit installed.
1. Run an [Arducopter software-in-the-loop simulator](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).  You don't need Mission Planner or MAVproxy although you can run those too if you like to.  The `sim_bridge.sh` script will put you on Clifton Suspension Bridge but it should be OK anyway, maybe give-or-take some altitude variations.
2. Run `python simple_flight.py` and you should see the mission tick through, with the drone taking off, doing a couple of moves, and then landing again.
