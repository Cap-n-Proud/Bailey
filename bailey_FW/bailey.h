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
#define SETTINGS_VERSION 5
#define SETTINGS_ADDRESS 32      // EEPROM address of the stored Settings
#define EEPROM_BYTES     1024    // ESP32: size of the flash area emulating EEPROM
#define G_ACC 9.81f              // m/s^2

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
  float stepsPerRev;              // stepper: steps per wheel turn, including microstepping
  float encoderCountsPerRev;      // DC: encoder counts per wheel turn
  float wheelDiameter;            // mm
  // Orientation of IMU, motors and encoders. Fix these instead of rewiring.
  uint8_t imuAxis;                // gyro axis the robot pitches around: 0 = X, 1 = Y, 2 = Z
  uint8_t imuInvert;              // 1 = pitch reads negative when leaning forward
  uint8_t motorLeftInvert, motorRightInvert;
  uint8_t encoderLeftInvert, encoderRightInvert;

  // Robot model and autotuner
  float pendulumLength;           // m, effective pendulum length the gains were computed for
  float autotuneAmplitude;        // deg, wobble added to the pitch setpoint while measuring

  // Balance controller: 0 = PID cascade, 1 = LQR + disturbance observer (steppers only)
  uint8_t controller;
  // LQR weights: how far each quantity may stray (for a 0.2 m robot; scaled
  // with the pendulum length). Smaller = the controller fights harder for it.
  float lqrMaxPosition;           // m
  float lqrMaxSpeed;              // m/s
  float lqrMaxAngle;              // deg
  float lqrMaxRate;               // deg/s
  float lqrMaxAccel;              // m/s^2, how much effort is acceptable
  float lqrAccelLimit;            // m/s^2, hard limit on the commanded acceleration
  float dobBandwidth;             // rad/s, disturbance observer speed for a 0.2 m robot (0 = off)

  // Kalman filter for pitch and gyro bias (imu.ino)
  float kalmanQAngle;             // deg^2/s, how fast the true pitch can drift from the gyro integral
  float kalmanQBias;              // (deg/s)^2/s, how fast the gyro bias can change
  float kalmanR;                  // deg^2, accelerometer pitch noise while balancing

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

enum AutotunePhase { AT_IDLE, AT_BASELINE, AT_MEASURE, AT_VERIFY };

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
void autotuneAbort(const char *reason);
float autotuneExcitation();
void autotuneStep(float dt, float angleError);
void autotuneOnFall();
bool autotuneRunning();

// lqr.ino
typedef float Mat4[4][4];        // 4x4 matrix, row major (in the header: the Arduino IDE's
                                 // generated prototypes need it before the .ino files)
#define CONTROLLER_PID 0
#define CONTROLLER_LQR 1
#define CONTROLLER_MPC 2         // ESP32-S3 only (mpc.ino)
extern bool lqrWeightsChanged;
extern float balanceAngle;
bool lqrSolve(float L, float dt, const float qDiag[4], float r, float K[4], Mat4 *P = NULL);
bool lqrSolveReference();
void lqrMaintain(bool balancing);
bool lqrReady();
void LQRParamTX();
void lqrReset();
void lqrZeroShifted(float delta);
float lqrStep(float dt, float referenceSpeed, float extraPitch, float &angleError);

// mpc.ino (ESP32-S3)
float mpcAccel(const float x[4], float speed, float referenceSpeed);
void mpcStats(uint16_t &iterations, uint16_t &microseconds);

// gains.ino
float fullSpeedMps();
void applyGainsForLength(float length);

#endif
