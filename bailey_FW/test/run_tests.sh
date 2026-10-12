#!/bin/sh
# Builds the firmware simulator for both motor types and runs every scenario.
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Istubs"
$CXX $FLAGS -DMOTOR_TYPE=1 -o sim_dc sim.cpp
$CXX $FLAGS -DMOTOR_TYPE=2 -o sim_stepper sim.cpp

fail=0
for sim in sim_dc sim_stepper; do
  for s in balance push drive fall autotune commands eeprom motortest; do
    printf '== %s ' "$sim"
    ./$sim $s || fail=1
  done
done
[ $fail -eq 0 ] && echo "ALL PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
