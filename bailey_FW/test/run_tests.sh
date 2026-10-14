#!/bin/sh
# Builds the firmware simulator for both motor types and runs every scenario.
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
FLAGS="-std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Istubs"
$CXX $FLAGS -DMOTOR_TYPE=1 -o sim_dc sim.cpp
$CXX $FLAGS -DMOTOR_TYPE=2 -o sim_stepper sim.cpp
# The ESP32-S3 build: same firmware with its board settings (500 Hz loop, ESP32 EEPROM/I2C calls).
$CXX $FLAGS -DMOTOR_TYPE=2 -DARDUINO_ARCH_ESP32 -o sim_esp32 sim.cpp

fail=0
for sim in sim_dc sim_stepper sim_esp32; do
  for s in balance push drive fall autotune scaling disturbance kalman lqrsolver limits commands eeprom motortest; do
    printf '== %s ' "$sim"
    ./$sim $s || fail=1
  done
done
# MPC (ESP32-S3 only) through the main scenarios.
for s in balance push drive fall autotune disturbance; do
  printf '== sim_esp32 (MPC) '
  SIM_CONTROLLER=2 ./sim_esp32 $s || fail=1
done
# Steppers default to LQR; keep the PID controller covered too.
for s in balance push drive fall autotune; do
  printf '== sim_stepper (PID) '
  SIM_CONTROLLER=0 ./sim_stepper $s || fail=1
done
[ $fail -eq 0 ] && echo "ALL PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
