#!/bin/bash

# Path to ardupilot directory
ARDUPILOT_PATH=/home/hirad/ardupilot

# Make a test results directory if one doesn't exist
mkdir -p -- Test_Results

# Make a time stamped folder for the results
simDate=$(date +%Y%m%d_%H%M%S)

# Start arducopter SITL
#$ARDUPILOT_PATH/build/sitl/bin/arducopter -w --model hexa --defaults=Test_Results/$simDate
#$ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa -w -N -A "--sim-port-out=tcp:127.0.0.1:5762" --no-mavproxy --add-param-file $(pwd)/sim_param --use-dir=Test_Results/$simDate --mavproxy-args='--daemon' &
#$ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa -w -N --add-param-file $(pwd)/sim_param --use-dir=Test_Results/$simDate --mavproxy-args='--daemon' --map --console  -l 51.423393,-2.671576,584,270 --out 127.0.0.1:14550 --out 127.0.0.1:14551 &
$ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa -w -N -l 51.423393,-2.671576,584,270 --add-param-file $(pwd)/sim_param --use-dir=Test_Results/$simDate --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552 --out 127.0.0.1:14553 --map --console
#sim_veh_pid=$!
