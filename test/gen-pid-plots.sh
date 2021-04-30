#!/bin/sh

set -u
set -e

python3 test.py
echo "setpoint,adjust,rps" >log.csv
grep "RPS: " log.txt |
  awk '{print $4 "," $5 "," $6}' \
  >>log.csv

python3 plot-pid.py log.csv
