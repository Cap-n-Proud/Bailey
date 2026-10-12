#pragma once
// Simulated FastAccelStepper: speed ramps towards the target with the set
// acceleration, like the real library. advance() is called by the simulator.
#include <stdint.h>
#include <math.h>
class FastAccelStepper {
 public:
  double speed = 0;          // steps/s, signed
  double target = 0;
  double accel = 1000;
  uint32_t maxHz = 0;
  bool enabled = false;
  void setDirectionPin(uint8_t, bool = true, uint16_t = 0) {}
  void setEnablePin(uint8_t, bool = true) {}
  void setAutoEnable(bool) {}
  bool enableOutputs() { enabled = true; return true; }
  bool disableOutputs() { enabled = false; return true; }
  int8_t setSpeedInHz(uint32_t hz) { maxHz = hz; return 0; }
  int8_t setAcceleration(int32_t a) { accel = a; return 0; }
  int8_t runForward() { target = maxHz; return 0; }
  int8_t runBackward() { target = -(double)maxHz; return 0; }
  void stopMove() { target = 0; }
  void forceStop() { speed = target = 0; }
  int32_t getCurrentSpeedInMilliHz(bool = true) const { return (int32_t)lround(speed * 1000); }
  void advance(double dt) {
    double step = accel * dt;
    if (target > speed + step) speed += step;
    else if (target < speed - step) speed -= step;
    else speed = target;
    if (!enabled) speed = target = 0;
  }
};
class FastAccelStepperEngine {
  FastAccelStepper steppers[2];
  int used = 0;
 public:
  void init() {}
  FastAccelStepper *stepperConnectToPin(uint8_t) { return used < 2 ? &steppers[used++] : nullptr; }
};
