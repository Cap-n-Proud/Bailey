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

# The same firmware on MuJoCo physics (a 3D robot with real wheel contact),
# if MuJoCo is installed (python3 -m pip install mujoco). LQR and MPC; the
# PID controller is the fallback and is only checked on the simulator above.
if ./build_mujoco.sh >/dev/null 2>&1; then
  for c in 1 2; do
    for s in balance push drive fall disturbance kalman limits scaling; do
      printf '== sim_mujoco (controller %s) ' "$c"
      SIM_CONTROLLER=$c ./sim_mujoco $s || fail=1
    done
    for L in 0.31 0.25 0.45; do
      printf '== sim_mujoco (controller %s, L %s) ' "$c" "$L"
      SIM_CONTROLLER=$c AT_L=$L ./sim_mujoco autotune || fail=1
    done
  done
else
  echo "== sim_mujoco skipped: MuJoCo not installed (python3 -m pip install mujoco)"
fi
[ $fail -eq 0 ] && echo "ALL PASSED" || { echo "SOME TESTS FAILED"; exit 1; }
