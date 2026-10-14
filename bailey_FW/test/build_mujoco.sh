#!/bin/sh
# Builds the simulator with MuJoCo physics (./sim_mujoco, see mujoco_plant.h).
# Needs g++, Eigen (see build.sh) and MuJoCo's C library, which the Python
# package includes:  python3 -m pip install mujoco   (or set MUJOCO=<dir with include/ and libmujoco.so*>)
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
EIGEN=${EIGEN:-/usr/include/eigen3}
PY=${PYTHON:-python3}
MUJOCO=${MUJOCO:-$($PY -c 'import mujoco, os; print(os.path.dirname(mujoco.__file__))')}
LIB=$(ls "$MUJOCO"/libmujoco.so* | head -n 1)
for f in admm tiny_api rho_benchmark; do
  [ -f tinympc_$f.o ] || $CXX -std=c++17 -O2 -w -Istubs -isystem "$EIGEN" -c ../src/tinympc/$f.cpp -o tinympc_$f.o
done
$CXX -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Istubs -isystem "$EIGEN" -isystem "$MUJOCO/include" -DSIM_MUJOCO \
  -o sim_mujoco sim.cpp tinympc_admm.o tinympc_tiny_api.o tinympc_rho_benchmark.o "$LIB" -Wl,-rpath,"$MUJOCO"
echo "built sim_mujoco (MuJoCo from $MUJOCO)"
