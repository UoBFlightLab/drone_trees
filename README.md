# drone_trees
Like all good cartoons, this library brings together two good stories:
* [DroneKit Python](http://python.dronekit.io/) offering easy Python interfacing to a MAVlink-controlled drone
* [py_trees](https://github.com/splintered-reality/py_trees) implementing behaviour trees in Python
The work was inspired by [Increasing Modularity of UAV Control Systems using Computer Game Behavior Trees](https://arc.aiaa.org/doi/pdf/10.2514/6.2012-4458) by Petter Ogren, and folows on from our experience using behaviour trees in the [Venturer](https://www.venturer-cars.com/) driverless car project at BRL.  Behaviour trees make it really easy to mess around with mission scripting in a modular way.  If you need to insert some extra checking or stages in an operation, you can do so without touching the parts you've made already.
## Example
This has all been tested on Windows running everything under Cygwin, although the Python stuff should work just as well on any Python environment.  You'll need ardupilot, py_trees and DroneKit installed.
1. Run an [Arducopter software-in-the-loop simulator](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html).  You don't need Mission Planner or MAVproxy although you can run those too if you like to.  The `sim_bridge.sh` script will put you on Clifton Suspension Bridge but it should be OK anyway, maybe give-or-take some altitude variations.
2. Run `python simple_flight.py` and you should see the mission tick through, with the drone taking off, doing a couple of moves, and then landing again.
## About the Example
![Behaviour tree](https://raw.githubusercontent.com/arthurrichards77/drone_trees/master/example.png)
Ogren's paper, referenced above, gives a really good introduction to how trees work, so I won't repeat that all here.  In summary:
* the orange boxes are Sequences, that work through each child in turn until it returns "Success" before moving on to the next child.
* the "OneShot" node at the top latches success if its gets it - otherwise the "Simple Flight" sequence would start again from scratch once "land" returned success.  It will literally take straight off again otherwise.
* the "FailureIsRunning" nodes convert their child's output as it sounds.  The effect is to make the sequence wait for something to happen rather than give up if it doesn't happen immediately, e.g. wait for the UAV to climb above 600m
## TO DO
* Make conditions to cover more drone attributes, e.g. last heartbeats, position tolerances, obstacle ranges, battery
* Include more robust contingency checking, e.g. detection of erroneous movement, excess speed or unexpected mode change
* Include low battery auto-land fail-safe.  This will exercise the Selector parent node that can prioritise actions rather than just follow a sequence.
