// Shared types and prototypes for the Bailey firmware.
// The Arduino IDE concatenates all .ino files of the sketch into one translation
// unit, so globals defined in bailey_FW.ino are visible everywhere; this header
// only declares the types and the functions used across files.
#ifndef BAILEY_H
#define BAILEY_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "config.h"

#define SETTINGS_MAGIC   0xBA17
#define SETTINGS_VERSION 3
#define SETTINGS_ADDRESS 32      // EEPROM address of the stored Settings
#define MAX_PARTICLES    12

// Everything that can be changed at runtime with "SCMD <name> <value>".
// Plain old data only, so it can be stored in EEPROM byte by byte.
struct Settings {
  uint16_t magic;
  uint8_t  version;

  // Angle loop: input is pitch (deg), output is motor command (% of full power).
  // Conservative gains near the setpoint, aggressive gains further away.
  float anglePIDConKp, anglePIDConKi, anglePIDConKd;
  float anglePIDAggKp, anglePIDAggKi, anglePIDAggKd;
  float TriggerAngleAggressive;   // deg of error where the aggressive gains kick in (0 = never)
  float anglePIDOutputLimit;      // % motor command
  float calibratedZeroAngle;      // deg, IMU reading when the robot is at its balance point
  float fallAngle;                // deg, beyond this the robot is considered fallen

  // Speed loop: input is wheel speed (% of full speed), output is the pitch setpoint (deg).
  float speedPIDKp, speedPIDKi, speedPIDKd;
  float speedPIDMoveFactor;       // speed gains are multiplied by this while driving
  float speedPIDOutputLimit;      // deg, maximum lean the speed loop may ask for
  float speedFilter;              // 0..1, low-pass factor for the measured wheel speed
  float speedRamp;                // %/s, how fast the speed setpoint may follow the throttle
  float speedFeedForward;         // speed setpoint added straight to the motor command (0..1)

  // Remote control
  float Maxsteer;                 // % motor command difference at full steer
  float Maxthrottle;              // % of full wheel speed at full throttle

  // Motors
  uint8_t motorLeftMinimumSpeed;  // DC: PWM below which the motor does not turn
  uint8_t motorRightMinimumSpeed;
  float encoderCountsPerSecFull;  // DC: encoder counts/s at full power (sets the speed scale)
  float stepperMaxSpeed;          // stepper: steps/s at 100 %
  float stepperMaxAccel;          // stepper: steps/s^2

  // Orientation of IMU, motors and encoders. Fix these instead of rewiring.
  uint8_t imuAxis;                // gyro axis the robot pitches around: 0 = X, 1 = Y, 2 = Z
  uint8_t imuInvert;              // 1 = pitch reads negative when leaning forward
  uint8_t motorLeftInvert, motorRightInvert;
  uint8_t encoderLeftInvert, encoderRightInvert;

  // Autotuner (particle swarm on the conservative angle gains)
  uint8_t numParticles;
  uint8_t maxIterations;
  uint16_t SPOConfigEval;         // ms each particle is evaluated
  uint16_t SPOSettle;             // ms to settle after changing gains, not scored
  float SPOspread;                // search +/- this fraction around the gains at start

  // Telemetry / debug
  uint8_t debug;                  // 1 = human-readable debug output instead of telemetry
  uint16_t debugSampleRate;       // ms
  uint16_t telemetryPeriod;       // ms

  uint16_t crc;
};

// PID with derivative on measurement, filtered derivative and clamped integral.
struct Pid {
  float kp, ki, kd;
  float outMin, outMax;
  float dFilter;                  // 0..1, 1 = no derivative filtering
  float integral;
  float prevInput;
  float dState;
  bool first;

  void reset() {
    integral = 0;
    dState = 0;
    first = true;
  }

  // inputRate: derivative of the input if it is measured directly (gyro);
  // otherwise it is computed from successive inputs.
  float compute(float setpoint, float input, float dt, float inputRate = NAN) {
    float error = setpoint - input;
    float dInput = 0;
    if (!isnan(inputRate))
      dInput = inputRate;
    else if (!first && dt > 0)
      dInput = (input - prevInput) / dt;
    first = false;
    prevInput = input;
    dState += dFilter * (dInput - dState);

    integral += ki * error * dt;
    if (integral > outMax) integral = outMax;
    if (integral < outMin) integral = outMin;

    float out = kp * error + integral - kd * dState;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    return out;
  }
};

enum RobotState {
  STATE_DISARMED = 0,  // motors switched off by the user
  STATE_WAITING  = 1,  // motors on, waiting for the robot to be held upright
  STATE_BALANCING = 2,
  STATE_FALLEN   = 3
};

// settings.ino
void loadDefaultSettings();
bool loadSettings();
void saveSettings();
void eraseSettings();

// imu.ino
void imuSetup();
void imuRead(float dt);

// motors.ino
void motorsSetup();
void motorsSet(float left, float right);
void motorsStop();
void motorsUpdateSpeed(float dt);

// bailey_FW.ino
void controlStep(float dt);
void setEvent(const char *event);

// comms.ino
void commsRead();
void handleLine(char *line);
void telemetryTX();
void debugTX();
void motorTestReport();
void remoteInit();
void PIDParamTX();
void SYSParamTX();
void printConfig();

// autotune.ino
void autotuneStart();
void autotuneStop(bool applyBest);
void autotuneStep(float dt, float angleError, float motorOutput);
void autotuneOnFall();

#endif
