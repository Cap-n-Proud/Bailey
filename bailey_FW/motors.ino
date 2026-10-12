// Motor drivers and wheel speed measurement.
// motorsSet() takes a command in % (-100..100, positive = forward) per wheel.
// motorsUpdateSpeed() updates leftSpeed/rightSpeed (% of full speed) and the
// filtered wheelSpeed used by the speed PID.

static float clampPercent(float v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

#if MOTOR_TYPE == MOTOR_DC

Encoder encoderLeft(PIN_ENC_LEFT_A, PIN_ENC_LEFT_B);
Encoder encoderRight(PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B);
long lastLeftCount = 0, lastRightCount = 0;

void motorsSetup() {
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
}

static void driveMotor(uint8_t pwmPin, uint8_t in1, uint8_t in2, float percent, uint8_t minPwm) {
  percent = clampPercent(percent);
  if (fabs(percent) < 0.5f) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    return;
  }
  // Map 0..100 % onto minPwm..255 so small commands still overcome friction.
  int pwm = minPwm + (int)(fabs(percent) / 100.0f * (255 - minPwm) + 0.5f);
  if (pwm > 255) pwm = 255;
  digitalWrite(in1, percent > 0 ? HIGH : LOW);
  digitalWrite(in2, percent > 0 ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void motorsSet(float left, float right) {
  driveMotor(PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2,
             cfg.motorLeftInvert ? -left : left, cfg.motorLeftMinimumSpeed);
  driveMotor(PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2,
             cfg.motorRightInvert ? -right : right, cfg.motorRightMinimumSpeed);
}

void motorsStop() {
  motorsSet(0, 0);
}

void motorsUpdateSpeed(float dt) {
  long l = encoderLeft.read();
  long r = encoderRight.read();
  long dl = l - lastLeftCount;
  long dr = r - lastRightCount;
  lastLeftCount = l;
  lastRightCount = r;
  if (dt <= 0) return;

  float scale = 100.0f / (cfg.encoderCountsPerSecFull * dt);
  leftSpeed = (cfg.encoderLeftInvert ? -dl : dl) * scale;
  rightSpeed = (cfg.encoderRightInvert ? -dr : dr) * scale;
  wheelSpeed += cfg.speedFilter * ((leftSpeed + rightSpeed) / 2 - wheelSpeed);
}

#elif MOTOR_TYPE == MOTOR_STEPPER

// Step pulses come from a Timer2 interrupt every STEP_TICK_US. Each motor has a
// phase accumulator: its rate is added every tick and a step is issued when it
// overflows 2^16, giving any step rate up to one step every two ticks.
volatile int32_t stepRateLeft = 0, stepRateRight = 0;   // steps per tick * 65536, signed
volatile uint32_t phaseLeft = 0, phaseRight = 0;
float stepSpeedLeft = 0, stepSpeedRight = 0;            // current steps/s after ramping

#if defined(__AVR__)
// digitalWrite() is too slow for a 20 kHz interrupt, so write the port directly.
volatile uint8_t *stepPortLeft, *stepPortRight;
uint8_t stepMaskLeft, stepMaskRight;

ISR(TIMER2_COMPA_vect) {
  // Pulses started on the previous tick end here (>= 50 us high).
  *stepPortLeft &= ~stepMaskLeft;
  *stepPortRight &= ~stepMaskRight;

  int32_t rl = stepRateLeft;
  if (rl != 0) {
    phaseLeft += (uint32_t)(rl < 0 ? -rl : rl);
    if (phaseLeft >= 65536UL) {
      phaseLeft -= 65536UL;
      *stepPortLeft |= stepMaskLeft;
    }
  }
  int32_t rr = stepRateRight;
  if (rr != 0) {
    phaseRight += (uint32_t)(rr < 0 ? -rr : rr);
    if (phaseRight >= 65536UL) {
      phaseRight -= 65536UL;
      *stepPortRight |= stepMaskRight;
    }
  }
}
#endif

void motorsSetup() {
  pinMode(PIN_LEFT_STEP, OUTPUT);
  pinMode(PIN_LEFT_DIR, OUTPUT);
  pinMode(PIN_RIGHT_STEP, OUTPUT);
  pinMode(PIN_RIGHT_DIR, OUTPUT);
  pinMode(PIN_STEPPER_EN, OUTPUT);
  digitalWrite(PIN_STEPPER_EN, HIGH);   // drivers off

#if defined(__AVR__)
  stepPortLeft = portOutputRegister(digitalPinToPort(PIN_LEFT_STEP));
  stepMaskLeft = digitalPinToBitMask(PIN_LEFT_STEP);
  stepPortRight = portOutputRegister(digitalPinToPort(PIN_RIGHT_STEP));
  stepMaskRight = digitalPinToBitMask(PIN_RIGHT_STEP);
  noInterrupts();
  TCCR2A = _BV(WGM21);                  // CTC mode
  TCCR2B = _BV(CS21);                   // prescaler 8 -> 2 MHz
  OCR2A = (F_CPU / 8 / 1000000UL) * STEP_TICK_US - 1;
  TIMSK2 = _BV(OCIE2A);
  interrupts();
#endif
}

static void setStepRate(float stepsPerSec, uint8_t dirPin, uint8_t invert, volatile int32_t &rate) {
  bool forward = stepsPerSec >= 0;
  if (invert) forward = !forward;
  digitalWrite(dirPin, forward ? HIGH : LOW);
  int32_t r = (int32_t)(fabs(stepsPerSec) * STEP_TICK_US * 1e-6f * 65536.0f);
  if (r > 32768) r = 32768;             // at most one step every two ticks
  noInterrupts();
  rate = r;
  interrupts();
}

static float ramp(float current, float target, float maxDelta) {
  if (target > current + maxDelta) return current + maxDelta;
  if (target < current - maxDelta) return current - maxDelta;
  return target;
}

static float lastStepDt = CONTROL_PERIOD_US * 1e-6f;

void motorsSet(float left, float right) {
  digitalWrite(PIN_STEPPER_EN, LOW);
  float maxDelta = cfg.stepperMaxAccel * lastStepDt;
  stepSpeedLeft = ramp(stepSpeedLeft, clampPercent(left) / 100.0f * cfg.stepperMaxSpeed, maxDelta);
  stepSpeedRight = ramp(stepSpeedRight, clampPercent(right) / 100.0f * cfg.stepperMaxSpeed, maxDelta);
  setStepRate(stepSpeedLeft, PIN_LEFT_DIR, cfg.motorLeftInvert, stepRateLeft);
  setStepRate(stepSpeedRight, PIN_RIGHT_DIR, cfg.motorRightInvert, stepRateRight);
}

void motorsStop() {
  stepSpeedLeft = stepSpeedRight = 0;
  setStepRate(0, PIN_LEFT_DIR, 0, stepRateLeft);
  setStepRate(0, PIN_RIGHT_DIR, 0, stepRateRight);
  digitalWrite(PIN_STEPPER_EN, HIGH);   // drivers off, motors free
}

void motorsUpdateSpeed(float dt) {
  // Steppers don't slip (unless they stall), so the commanded speed is the speed.
  if (dt > 0) lastStepDt = dt;
  leftSpeed = stepSpeedLeft / cfg.stepperMaxSpeed * 100.0f;
  rightSpeed = stepSpeedRight / cfg.stepperMaxSpeed * 100.0f;
  wheelSpeed += cfg.speedFilter * ((leftSpeed + rightSpeed) / 2 - wheelSpeed);
}

#endif
