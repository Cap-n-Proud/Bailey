TinyMPC (vendored)
==================

Model predictive control solver for microcontrollers (ADMM on top of a cached
Riccati solution), from https://github.com/TinyMPC/TinyMPC (MIT licence, see
LICENSE), commit 023f36bd5b27267e1a96ed47a774d120b5707f13 (2 Sep 2026).
Used by ../../mpc.ino.

Only the solver sources are copied (src/tinympc: admm, tiny_api, types,
error, rho_benchmark; not codegen). Changes, all marked "Bailey:":

* types.hpp: `tinytype` is `float` (the ESP32-S3 FPU is single precision),
  and Eigen comes from the Arduino "Eigen" library (`<ArduinoEigenDense.h>`).
* admm.cpp: removed the unconditional "Solver converged" print.

To update: copy the same files from a newer TinyMPC and reapply these changes.
