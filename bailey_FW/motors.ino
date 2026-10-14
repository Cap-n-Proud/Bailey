// Motor drivers and wheel speed measurement.
// motorsSet() takes a command in % (-100..100, positive = forward) per wheel.
// motorsUpdateSpeed() updates leftSpeed/rightSpeed (% of full speed) and the
// filtered wheelSpeed used by the speed PID.

// Shared by both motor types once leftSpeed/rightSpeed are known: the filtered
// speed for the speed PID, plus ground speed and acceleration in physical units
// for the IMU's acceleration compensation and the autotuner.
static float lastGroundSpeed = 0;

static void updateWheelMotion(float dt) {
  float avg = (leftSpeed + rightSpeed) / 2;
  wheelSpeed += cfg.speedFilter * (avg - wheelSpeed);
  groundSpeed = avg / 100.0f * fullSpeedMps();
  if (dt > 0) wheelAccel = (groundSpeed - lastGroundSpeed) / dt;
  lastGroundSpeed = groundSpeed;
}

static float clampPercent(float v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}


// FastAccelStepper generates the step pulses from a hardware timer and ramps
// the speed with a bounded acceleration, so the motors never get a speed jump
// they can't follow (which would make them skip steps).
FastAccelStepperEngine engine;
FastAccelStepper *stepperLeft = NULL;
FastAccelStepper *stepperRight = NULL;
static bool steppersEnabled = false;

void motorsSetup() {
  engine.init();
  stepperLeft = engine.stepperConnectToPin(PIN_LEFT_STEP);
  stepperRight = engine.stepperConnectToPin(PIN_RIGHT_STEP);
  if (!stepperLeft || !stepperRight) {
    out.println(F("E, stepper pins not usable by FastAccelStepper"));
    return;
  }
  stepperLeft->setDirectionPin(PIN_LEFT_DIR);
  stepperRight->setDirectionPin(PIN_RIGHT_DIR);
  stepperLeft->setEnablePin(PIN_STEPPER_EN);
  stepperRight->setEnablePin(PIN_STEPPER_EN);
  stepperLeft->setAutoEnable(false);
  stepperRight->setAutoEnable(false);
}

static void driveStepper(FastAccelStepper *s, float percent, uint8_t invert) {
  if (!s) return;
  if (invert) percent = -percent;
  float hz = clampPercent(percent) / 100.0f * cfg.stepperMaxSpeed;
  s->setAcceleration((int32_t)cfg.stepperMaxAccel);
  if (fabs(hz) < 1.0f) {
    s->stopMove();                      // ramps down to standstill
    return;
  }
  s->setSpeedInHz((uint32_t)fabs(hz));
  // Reversing is handled by the library: it decelerates through zero first.
  if (hz > 0) s->runForward();
  else s->runBackward();
}

void motorsSet(float left, float right) {
  if (!steppersEnabled && stepperLeft) {
    stepperLeft->enableOutputs();
    stepperRight->enableOutputs();
    steppersEnabled = true;
  }
  driveStepper(stepperLeft, left, cfg.motorLeftInvert);
  driveStepper(stepperRight, right, cfg.motorRightInvert);
}

void motorsStop() {
  if (!stepperLeft) return;
  stepperLeft->forceStop();
  stepperRight->forceStop();
  if (steppersEnabled) {
    stepperLeft->disableOutputs();      // motors free, drivers cool
    stepperRight->disableOutputs();
    steppersEnabled = false;
  }
}

static float stepperSpeedPercent(FastAccelStepper *s, uint8_t invert) {
  if (!s) return 0;
  float hz = s->getCurrentSpeedInMilliHz() / 1000.0f;
  if (invert) hz = -hz;
  return hz / cfg.stepperMaxSpeed * 100.0f;
}

void motorsUpdateSpeed(float dt) {
  // Steppers don't slip (unless they stall), so the actual step rate is the speed.
  leftSpeed = stepperSpeedPercent(stepperLeft, cfg.motorLeftInvert);
  rightSpeed = stepperSpeedPercent(stepperRight, cfg.motorRightInvert);
  updateWheelMotion(dt);
}

