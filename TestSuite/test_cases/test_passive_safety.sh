#!/bin/bash

# Path to ardupilot directory
ARDUPILOT_PATH=/home/hirad/ardupilot

# Get the testcase to run
#TESTCASE_PATH=$1
filename=test_case_01.py
test_case="passive_safety"

cd ../
# Make a test results directory if one doesn't exist
mkdir -p -- Test_Results
cd Test_Results

# Make a test-case directory if one doesn't exist
mkdir -p -- $test_case
cd $test_case

# Make a time stamped folder for the results
simDate=$(date +%Y%m%d_%H%M%S)
mkdir $simDate
cd $simDate
mkdir BT
cd ../../../

# Start arducopter SITL
gnome-terminal -- $ARDUPILOT_PATH/Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa -w -N -l 51.4235405,-2.6711033,584,270 --add-param-file $(pwd)/sim_param --use-dir=Test_Results/$test_case/$simDate --out 127.0.0.1:14550 --out 127.0.0.1:14551 --out 127.0.0.1:14552 --out 127.0.0.1:14553 --map --console

# Arm and take-off when armable
python3 sim_takeoff.py &

cd test_cases

# Inject test-case condition  
python3 $filename &

cd ../../

# Run BT
timeout 1.5m python3 sim_demo.py TestSuite/Test_Results/$test_case/$simDate