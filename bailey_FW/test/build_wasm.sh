#!/usr/bin/env bash
# Builds the simulator for the interactive web page (web/index.html):
# the ESP32-S3 stepper firmware and the robot model, as WebAssembly.
# Needs clang, wasm-ld and a WASI sysroot (Ubuntu/Debian: apt install
# wasi-libc libc++-18-dev-wasm32 libc++abi-18-dev-wasm32 libclang-rt-18-dev-wasm32).
#
#   ./build_wasm.sh && cd web && python3 -m http.server   # then open http://localhost:8000
set -euo pipefail
cd "$(dirname "$0")"
CXX=${CXX:-clang++}
$CXX --target=wasm32-wasi -std=c++17 -O2 -fno-exceptions -Istubs \
  -DMOTOR_TYPE=2 -DARDUINO_ARCH_ESP32 \
  -mexec-model=reactor -Wl,--export=sim_reset,--export=sim_run,--export=sim_input,--export=sim_send,--export=sim_output,--export=sim_get,--export=sim_set -Wl,--strip-all \
  -o web/bailey_sim.wasm wasm.cpp
ls -l web/bailey_sim.wasm
