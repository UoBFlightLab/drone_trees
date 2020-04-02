# drone_trees
Like all good cartoons, this library brings together two good stories:
* [DroneKit Python](https://github.com/dronekit/dronekit-python) offering easy Python interfacing to a MAVlink-controlled drone
* [py_trees](https://github.com/splintered-reality/py_trees) implementing behaviour trees in Python

The work was inspired by [Increasing Modularity of UAV Control Systems using Computer Game Behavior Trees](https://arc.aiaa.org/doi/pdf/10.2514/6.2012-4458) by Petter Ogren, and follows on from our experience using behaviour trees in the [Venturer](https://www.venturer-cars.com/) driverless car project at BRL.  Behaviour trees make it really easy to mess around with mission scripting in a modular way.  If you need to insert some extra checking or stages in an operation, you can do so without touching the parts you've made already.

## Installation
drone_trees is compatible with python 3.6 and it needs the following dependencies:
* `dronekit-python` version 2.9.1 or higher. To install simply `pip install dronekit`.
* `py-trees` version 2.0.0 or higher along with `graphviz` (for viewing trees)
* `pyttsx3` V2.87 for voice feedback. If using windows `pypiwin32` is also needed.

## Example

This has all been tested on Windows WSL, although the Python stuff should work just as well on any Python environment.  You'll need ardupilot, py_trees, dronkit and pyttsx3 installed.

1. Set up [Arducopter software-in-the-loop simulator](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) according to its documentation.
2. Open `sim_demo.sh` script and change `ARDUPILOT_PATH` to your ardupilot directory.
3. Run `sim_demo.sh`. The script will lunch SITL with the correct parameters, handles the take-off and run the behaviour tree demo. The script will put you on Clifton Suspension Bridge but it should be OK anyway, maybe give-or-take some altitude variations.
4. Alternatively, you can start your own ardupilot SITL with udp port of 14551 such as `sim_vehicle.py --map --console  -l 51.4545219,-2.6291727,584,270 --out 127.0.0.1:14550 --out 127.0.0.1:14551`.  Once the SITL is running `Arm` and take-off the drone in `Guided` mode then run `sim_demo.py`. Finally change mode to `Auto`.

## About the Example
![Behaviour tree](https://raw.githubusercontent.com/hiradg/drone_trees/dev-che/example.png)
Ogren's paper, referenced above, gives a really good introduction to how trees work, so I won't repeat that all here.  In summary:
* the boxes are Sequences, that work through each child in turn until it returns "Success" before moving on to the next child.
* the octagons are selectors, they try each child in order, if it returns "Failure" they move to the next child.
* The paralellograms are parrallels, they check all their children simultaneously and they can have a defined success criteria.
* the "OneShot" node at the top latches success if its gets it - otherwise the "Simple Flight" sequence would start again from scratch once "land" returned success.  It will literally take straight off again otherwise.
* the "F=R" nodes convert their child's "Failure" response to "Success".  The effect is to make the sequence wait for something to happen rather than give up if it doesn't happen immediately, e.g. wait for the UAV to reach certain waypoint.