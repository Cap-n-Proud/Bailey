#!/bin/sh
# Builds the firmware simulator (./sim). Needs g++ and Eigen
# (Debian/Ubuntu: apt install libeigen3-dev; or set EIGEN=<dir with Eigen/>).
set -e
cd "$(dirname "$0")"
CXX=${CXX:-g++}
EIGEN=${EIGEN:-/usr/include/eigen3}
# TinyMPC (vendored, third party) without our warning flags.
for f in admm tiny_api rho_benchmark; do
  $CXX -std=c++17 -O2 -w -Istubs -isystem "$EIGEN" -c ../src/tinympc/$f.cpp -o tinympc_$f.o
done
$CXX -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -Istubs -isystem "$EIGEN" \
  -o sim sim.cpp tinympc_admm.o tinympc_tiny_api.o tinympc_rho_benchmark.o
