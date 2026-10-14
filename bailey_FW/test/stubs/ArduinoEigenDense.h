// Simulator: Eigen from the system (Debian/Ubuntu: apt install libeigen3-dev)
// instead of the Arduino "Eigen" library, which is the same Eigen 3.4 with
// one change: a template parameter named F renamed so Arduino's F() macro
// doesn't break it. Hiding the macro while including does the same here.
#pragma once
#pragma push_macro("F")
#undef F
#include <Eigen/Dense>
#pragma pop_macro("F")
