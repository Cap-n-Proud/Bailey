// Bailey self-balancing robot firmware.
//
// Control structure (runs at a fixed 200 Hz, see CONTROL_PERIOD_US):
//
//   throttle -> speed setpoint -> [speed PID] -> pitch setpoint -> [angle PID] -> motors
//                                     ^ wheel speed (encoders)       ^ pitch (IMU)
//
// Sign convention used everywhere: pitch > 0 means leaning forward, a positive
// motor command drives the robot forward and a positive wheel speed means moving
// forward. Use the imu*/motor*/encoder*Invert settings to make the hardware match.

#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050.h>    // Electronic Cats MPU6050 (maintained i2cdevlib): https://github.com/ElectronicCats/mpu6050
#include "bailey.h"     // must come before the MOTOR_TYPE checks below
#if MOTOR_TYPE == MOTOR_DC
#include <Encoder.h>    // https://www.pjrc.com/teensy/td_libs_Encoder.html
#else
#include <FastAccelStepper.h>  // https://github.com/gin66/FastAccelStepper
#endif

Settings cfg;
Pid anglePID;
Pid speedPID;

// Sensors
float pitch = 0;              // balance angle relative to the calibrated zero (deg)
float pitchRaw = 0;           // balance angle as read from the IMU (deg)
float pitchRate = 0;          // deg/s, straight from the gyro
float yaw = 0, roll = 0;      // telemetry only (deg)
float leftSpeed = 0, rightSpeed = 0;   // % of full speed
float wheelSpeed = 0;         // filtered average, input of the speed PID
float groundSpeed = 0;        // m/s, unfiltered
float wheelAccel = 0;         // m/s^2, unfiltered

// Control
float angleSetpoint = 0;
float angleOutput = 0;        // % motor command from the angle PID
float speedSetpoint = 0;
float steerCmd = 0;           // remote input, -100..100
float throttleCmd = 0;        // remote input, -100..100
bool motorsEnabled = false;
RobotState state = STATE_DISARMED;
unsigned long uprightSince = 0;
unsigned long motorTestUntil = 0;
unsigned long lastDriveCmd = 0;   // millis() of the last Throttle/Steer command
float motorTestPower = 0;

// Statistics
float iste = 0;               // integral of squared angle error since the last telemetry line
unsigned long maxLoopUs = 0;
unsigned long lastControlUs = 0;
char lastEvent[32] = "Boot";

void setEvent(const char *event) {
  strncpy(lastEvent, event, sizeof(lastEvent) - 1);
  lastEvent[sizeof(lastEvent) - 1] = 0;
  Serial.print(F("EV,"));
  Serial.println(lastEvent);
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  if (!loadSettings()) {
    loadDefaultSettings();
    saveSettings();
  }

  motorsSetup();
  motorsStop();

  // The angle D term uses the gyro rate, which is clean; the speed loop's
  // derivative is computed from noisy wheel speed and filtered harder.
  anglePID.dFilter = 0.5f;
  speedPID.dFilter = 0.2f;
  anglePID.reset();
  speedPID.reset();

  Wire.begin();
  imuSetup();   // keep the robot still: the gyro is calibrated here

  randomSeed(analogRead(0));
  lastControlUs = micros();
  remoteInit();
  setEvent("Ready");
}

void loop() {
  commsRead();

  // Drive watchdog: the remote repeats Throttle/Steer while they are non-zero,
  // so if the Pi, the network or the browser goes away the robot stops instead
  // of driving off.
  if ((throttleCmd != 0 || steerCmd != 0) && millis() - lastDriveCmd > DRIVE_TIMEOUT_MS) {
    throttleCmd = steerCmd = 0;
    setEvent("Drive timeout");
  }

  unsigned long now = micros();
  if (now - lastControlUs >= CONTROL_PERIOD_US) {
    float dt = (now - lastControlUs) * 1e-6f;
    // If something blocked us for long, don't let one huge dt kick the integrators.
    if (dt > 0.05f) dt = 0.05f;
    lastControlUs = now;

    controlStep(dt);

    unsigned long took = micros() - now;
    if (took > maxLoopUs) maxLoopUs = took;
  }

  static unsigned long lastTx = 0;
  unsigned long period = cfg.debug ? cfg.debugSampleRate : cfg.telemetryPeriod;
  if (millis() - lastTx >= period) {
    lastTx = millis();
    if (cfg.debug)
      debugTX();
    else
      telemetryTX();
  }
  motorTestReport();
}

// LQR needs the steppers' exact acceleration; DC motors always use the PIDs.
static bool wantLqr() {
  return MOTOR_TYPE == MOTOR_STEPPER && cfg.controller == CONTROLLER_LQR;
}

// ...and a solved gain: if the Riccati solver ever failed, balance with the PIDs.
static bool useLqr() {
  return wantLqr() && lqrReady();
}

static void updateState() {
  unsigned long nowMs = millis();
  bool upright = fabs(pitch) < UPRIGHT_ANGLE;

  if (!motorsEnabled) {
    state = STATE_DISARMED;
    return;
  }

  switch (state) {
    case STATE_DISARMED:
    case STATE_FALLEN:
    case STATE_WAITING:
      if (nowMs < ARM_DELAY_MS || !upright) {
        uprightSince = 0;
        if (state == STATE_DISARMED) state = STATE_WAITING;
        break;
      }
      if (uprightSince == 0) uprightSince = nowMs;
      if (nowMs - uprightSince >= UPRIGHT_HOLD_MS) {
        anglePID.reset();
        speedPID.reset();
        if (useLqr()) lqrReset();
        uprightSince = 0;
        state = STATE_BALANCING;
        setEvent("Balancing");
      }
      break;

    case STATE_BALANCING:
      if (fabs(pitch) > cfg.fallAngle) {
        state = STATE_FALLEN;
        motorsStop();
        autotuneOnFall();
        setEvent("Fallen");
      }
      break;
  }
}

// Cascaded PIDs: the speed loop sets the pitch setpoint, the angle loop the motors.
static float pidStep(float dt, float &angleError) {
  float f = (throttleCmd != 0) ? cfg.speedPIDMoveFactor : 1.0f;
  speedPID.kp = f * cfg.speedPIDKp;
  speedPID.ki = f * cfg.speedPIDKi;
  speedPID.kd = f * cfg.speedPIDKd;
  speedPID.outMin = -cfg.speedPIDOutputLimit;
  speedPID.outMax = cfg.speedPIDOutputLimit;
  angleSetpoint = speedPID.compute(speedSetpoint, wheelSpeed, dt) + autotuneExcitation();

  // The PID drives (setpoint - pitch) to zero; when leaning forward (pitch >
  // setpoint) its output is negative, and the wheels must go forward.
  angleError = pitch - angleSetpoint;
  if (cfg.TriggerAngleAggressive > 0 && fabs(angleError) > cfg.TriggerAngleAggressive) {
    anglePID.kp = cfg.anglePIDAggKp;
    anglePID.ki = cfg.anglePIDAggKi;
    anglePID.kd = cfg.anglePIDAggKd;
  } else {
    anglePID.kp = cfg.anglePIDConKp;
    anglePID.ki = cfg.anglePIDConKi;
    anglePID.kd = cfg.anglePIDConKd;
  }
  anglePID.outMin = -cfg.anglePIDOutputLimit;
  anglePID.outMax = cfg.anglePIDOutputLimit;
  angleOutput = -anglePID.compute(angleSetpoint, pitch, dt, pitchRate);

  // Feed the speed setpoint forward: wheel speed in % roughly equals motor command
  // in %, so the PIDs only have to correct the difference instead of slowly
  // integrating up the whole cruise command.
  return angleOutput + cfg.speedFeedForward * speedSetpoint;
}

void controlStep(float dt) {
  imuRead(dt);
  motorsUpdateSpeed(dt);
  updateState();
  if (wantLqr()) lqrMaintain(state == STATE_BALANCING);

  if (state != STATE_BALANCING) {
    angleOutput = 0;
    angleSetpoint = 0;
    speedSetpoint = 0;
    if (state == STATE_DISARMED && millis() < motorTestUntil)
      motorsSet(motorTestPower, motorTestPower);
    else
      motorsStop();
    return;
  }

  // Speed setpoint from the throttle, ramped: a sudden jump makes the robot
  // overshoot into motor saturation.
  float target = throttleCmd / 100.0f * cfg.Maxthrottle;
  float step = cfg.speedRamp * dt;
  speedSetpoint += constrain(target - speedSetpoint, -step, step);

  float angleError;
  float drive;
  if (useLqr()) {
    drive = lqrStep(dt, speedSetpoint / 100.0f * fullSpeedMps(), autotuneExcitation(), angleError);
    angleOutput = drive;
    angleSetpoint = pitch - angleError;
  } else {
    drive = pidStep(dt, angleError);
  }

  float steer = steerCmd / 100.0f * cfg.Maxsteer;
  motorsSet(drive + steer, drive - steer);

  iste += angleError * angleError * dt;
  autotuneStep(dt, angleError);
}
