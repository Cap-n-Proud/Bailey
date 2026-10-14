#!/bin/sh
# Builds the firmware simulator and runs every scenario, with each controller.
# Needs g++ and Eigen (see build.sh).
set -e
cd "$(dirname "$0")"
./build.sh

fail=0
for s in balance push drive fall autotune scaling disturbance kalman lqrsolver limits commands eeprom watchdog motortest; do
  printf '== sim '
  ./sim $s || fail=1
done
# MPC through the main scenarios.
for s in balance push drive fall autotune disturbance; do
  printf '== sim (MPC) '
  SIM_CONTROLLER=2 ./sim $s || fail=1
done
# LQR is the default; keep the PID controller covered too.
for s in balance push drive fall autotune; do
  printf '== sim (PID) '
  SIM_CONTROLLER=0 ./sim $s || fail=1
done
[ $fail -eq 0 ] && echo "ALL PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
