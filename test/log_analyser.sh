#!/bin/bash

LOGPATH=$1
cd $LOGPATH/logs

types=(CMD GPS RFND BAT)
for i in $(find . -name '*.BIN' -exec echo {} \;); do
 for t in ${types[@]}; do 
  mavlogdump.py --types=$t --format=csv $i > "${i/'.BIN'/"$t.csv"}"
 done
done