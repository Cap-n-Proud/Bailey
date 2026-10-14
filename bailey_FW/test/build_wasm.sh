#!/usr/bin/env bash
# Builds the simulator for the interactive web page (web/index.html):
# the firmware (with Eigen and TinyMPC) and the robot model, as WebAssembly.
# Needs clang, wasm-ld, a WASI sysroot and Eigen (Ubuntu/Debian: apt install
# wasi-libc libc++-18-dev-wasm32 libc++abi-18-dev-wasm32 libclang-rt-18-dev-wasm32 libeigen3-dev).
#
#   ./build_wasm.sh && cd web && python3 -m http.server   # then open http://localhost:8000
set -euo pipefail
cd "$(dirname "$0")"
CXX=${CXX:-clang++}
EIGEN=${EIGEN:-/usr/include/eigen3}
$CXX --target=wasm32-wasi -std=c++17 -O2 -fno-exceptions -w -Istubs -isystem $EIGEN \
  -mexec-model=reactor -Wl,--export=sim_reset,--export=sim_run,--export=sim_input,--export=sim_send,--export=sim_output,--export=sim_get,--export=sim_set -Wl,--strip-all \
  -o web/bailey_sim.wasm wasm.cpp ../src/tinympc/admm.cpp ../src/tinympc/tiny_api.cpp ../src/tinympc/rho_benchmark.cpp
ls -l web/bailey_sim.wasm
