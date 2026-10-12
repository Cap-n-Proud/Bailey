#pragma once
// The simulator writes the orientation it wants the firmware to see here.
extern float simYpr[3];
class FreeIMU {
 public:
  void init() {}
  void getYawPitchRoll(float *ypr) { ypr[0] = simYpr[0]; ypr[1] = simYpr[1]; ypr[2] = simYpr[2]; }
};
