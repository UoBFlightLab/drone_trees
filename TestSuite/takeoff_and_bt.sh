#!/bin/bash

python3 sim_takeoff.py &

cd test_cases

#python3 test_case_02_proximity.py &

cd ../../

python3 sim_demo.py