// Controller gains from the robot's physical model.
//
// With steppers the wheels follow the commanded speed exactly, so the robot is
// an inverted pendulum on a speed-controlled cart:
//
//   pitch'' = (g * pitch - wheel acceleration) / L
//
// where L is the effective pendulum length, (I + m l^2) / (m l). Two robots
// that differ only in L behave the same with time scaled by sqrt(L / g), so a
// set of gains tuned for one L carries over exactly to any other L:
//
//   angle Kp ~ sqrt(g L)   Ki ~ g   Kd ~ L
//   speed Kp ~ 1 / sqrt(g L)        Ki ~ 1 / L
//
// The reference gains below were tuned in the simulator (test/) for L = 0.2 m
// and checked over L = 0.12 - 0.3 m, IMU offsets and noise, so they are also
// the defaults before the robot has been measured. They are in physical units
// and converted to the firmware's % units using the wheel size and stepper
// settings, so changing microstepping or wheels doesn't invalidate them.

#define REF_LENGTH 0.20f        // m
#define REF_ANGLE_KP 5.076f     // (m/s) per rad
#define REF_ANGLE_KI 5.076f     // (m/s^2) per rad
#define REF_ANGLE_KD 0.550f     // m per rad
#define REF_SPEED_KP 0.3546f    // rad per (m/s)
#define REF_SPEED_KI 0.04728f   // rad per m
#define AGGRESSIVE_FACTOR 1.3f

// Ground speed at 100 % motor command.
float fullSpeedMps() {
  float turnsPerSec;
  turnsPerSec = cfg.stepperMaxSpeed / cfg.stepsPerRev;
  return turnsPerSec * (float)M_PI * cfg.wheelDiameter / 1000.0f;
}

void applyGainsForLength(float length) {
  float s = length / REF_LENGTH;
  float r = sqrt(s);
  float vFull = fullSpeedMps();

  // (m/s per rad) -> (% per deg)
  float toPercent = 100.0f / vFull / 57.29578f;
  cfg.anglePIDConKp = REF_ANGLE_KP * r * toPercent;
  cfg.anglePIDConKi = REF_ANGLE_KI * toPercent;
  cfg.anglePIDConKd = REF_ANGLE_KD * s * toPercent;
  cfg.anglePIDAggKp = cfg.anglePIDConKp * AGGRESSIVE_FACTOR;
  cfg.anglePIDAggKi = cfg.anglePIDConKi * AGGRESSIVE_FACTOR;
  cfg.anglePIDAggKd = cfg.anglePIDConKd * AGGRESSIVE_FACTOR;

  // (rad per m/s) -> (deg per %)
  float toDegPerPercent = 57.29578f * vFull / 100.0f;
  cfg.speedPIDKp = REF_SPEED_KP / r * toDegPerPercent;
  cfg.speedPIDKi = REF_SPEED_KI / s * toDegPerPercent;
  cfg.speedPIDKd = 0;

  cfg.pendulumLength = length;
}
