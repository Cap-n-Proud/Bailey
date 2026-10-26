// Autotuner: measures the robot while it balances and computes its gains.
//
// The robot is an inverted pendulum driven by its wheels' acceleration (lqr.ino):
//
//   pitch'' = (g / L) * (pitchMeasured - offset) - (kappa / L) * wheelAcceleration
//
// L (how fast it falls), kappa (how hard the wheels tip it) and the IMU offset
// are the unknowns, fitted by least squares on  pitch'' = p1 * pitch + p2 + p3 * accel:
//   L = g / p1,  kappa = -p3 * L,  offset = -p2 / p1
// kappa is 1 for a cart-pole and a bit more for a real two-wheeler (the wheels
// turn with the body and the motors push back on it), so a kappa far from that
// means the measurement went wrong (wheels slipping, steps skipped).
// Gains then follow from L and kappa (gains.ino, lqr.ino) and the offset
// becomes the new zero.
//
// "SCMD AUTOTUNE 1" while balancing, throttle and steer at 0. About 33 s:
//   1. BASELINE  6 s  balance on the current gains, score the wobble
//   2. MEASURE  18 s  add a small wobble (autotuneAmplitude) to the pitch
//                     setpoint so there is motion to measure, fit the model
//   3. VERIFY    9 s  balance on the new gains and zero, score the wobble
//                     (after 4 s: a tall robot takes ~3 s to calm down after MEASURE)
// The result is kept only if the fit is consistent and the robot balances at
// least as well as before; otherwise everything is restored. "SCMD SaveCfg 1"
// stores it. Driving, steering, falling, Motors 0, AUTOTUNE 0 or a tilt error
// above ABORT_ERROR abort the run and restore the previous settings.
//
// Progress is reported as "AT,<phase>" lines and the outcome as
// "AT,RESULT,<accepted>,<L>,<kappa>,<offset>,<R^2>,<score before>,<score after>".


#define BASELINE_S   6.0f
#define MEASURE_S    18.0f
#define VERIFY_S     9.0f
#define SETTLE_S     1.0f       // start of each phase not scored / not fitted
#define VERIFY_SETTLE_S 4.0f    // ... longer after MEASURE's wobble
#define FIT_CUTOFF_HZ 3.0f      // same low-pass on every signal entering the fit
#define FIT_PITCH_TAU 5.0f      // s, see fitPitch
#define ABORT_ERROR  6.0f       // deg
#define MIN_LENGTH   0.05f      // m, plausible range of L
#define MAX_LENGTH   0.80f
#define MIN_COUPLING 0.85f      // plausible range of kappa (physically >= 1, plus fit noise)
#define MAX_COUPLING 2.5f
#define MIN_R2       0.6f       // fraction of pitch'' the model must explain
#define AT_ACCEL_DITHER 0.4f    // m/s^2, see autotuneAccelExcitation

static AutotunePhase phase = AT_IDLE;
static float phaseTime = 0;
static float rateSum = 0, gyroBias = 0;   // rad/s, residual gyro bias measured in BASELINE
static float score = 0, scoreTime = 0, baselineScore = 0;

// Settings changed by the autotuner, restored if the result is rejected.
struct SavedTuning {
  float conKp, conKi, conKd, aggKp, aggKi, aggKd, speedKp, speedKi, length, coupling, zero;
};
static SavedTuning saved;

// Two first-order low-passes in series, applied identically to every signal of
// the fit: a linear filter doesn't change the linear relation between them.
struct LowPass2 {
  float a, b;
  void reset(float v) { a = b = v; }
  float step(float x, float k) {
    a += k * (x - a);
    b += k * (a - b);
    return b;
  }
};
static LowPass2 fPitch, fAccel, fPitchAcc;
static float prevRate, prevSpeed, delayedAccel, fitPitch;
static bool havePrev;

// Running means and covariances (Welford) of x0 = pitch, x1 = accel, x2 = pitch''.
static float mean[3];
static float cov[6];            // 00 01 02 11 12 22, sums of products of deviations
static long samples;

static uint8_t covIndex(uint8_t i, uint8_t j) {
  if (i > j) { uint8_t t = i; i = j; j = t; }
  static const uint8_t idx[3][3] = { { 0, 1, 2 }, { 1, 3, 4 }, { 2, 4, 5 } };
  return idx[i][j];
}

static void fitReset() {
  memset(mean, 0, sizeof(mean));
  memset(cov, 0, sizeof(cov));
  samples = 0;
  havePrev = false;
}

static void fitAdd(const float x[3]) {
  float d[3];
  samples++;
  for (uint8_t i = 0; i < 3; i++) {
    d[i] = x[i] - mean[i];
    mean[i] += d[i] / samples;
  }
  for (uint8_t i = 0; i < 3; i++)
    for (uint8_t j = i; j < 3; j++)
      cov[covIndex(i, j)] += d[i] * (x[j] - mean[j]);
}

static void saveTuning() {
  saved.conKp = cfg.anglePIDConKp;
  saved.conKi = cfg.anglePIDConKi;
  saved.conKd = cfg.anglePIDConKd;
  saved.aggKp = cfg.anglePIDAggKp;
  saved.aggKi = cfg.anglePIDAggKi;
  saved.aggKd = cfg.anglePIDAggKd;
  saved.speedKp = cfg.speedPIDKp;
  saved.speedKi = cfg.speedPIDKi;
  saved.length = cfg.pendulumLength;
  saved.coupling = cfg.pendulumCoupling;
  saved.zero = cfg.calibratedZeroAngle;
}

// Move the zero without a bump: the speed loop's integral has been holding the
// pitch setpoint at the old offset, so shift it by the same amount.
static void setZero(float zero) {
  float delta = zero - cfg.calibratedZeroAngle;
  cfg.calibratedZeroAngle = zero;
  pitch -= delta;
  speedPID.integral -= delta;
  lqrZeroShifted(delta);
}

static void restoreTuning() {
  cfg.anglePIDConKp = saved.conKp;
  cfg.anglePIDConKi = saved.conKi;
  cfg.anglePIDConKd = saved.conKd;
  cfg.anglePIDAggKp = saved.aggKp;
  cfg.anglePIDAggKi = saved.aggKi;
  cfg.anglePIDAggKd = saved.aggKd;
  cfg.speedPIDKp = saved.speedKp;
  cfg.speedPIDKi = saved.speedKi;
  cfg.pendulumLength = saved.length;
  cfg.pendulumCoupling = saved.coupling;
  setZero(saved.zero);
}

static void startPhase(AutotunePhase p) {
  phase = p;
  phaseTime = 0;
  score = 0;
  scoreTime = 0;
  static const char *const names[] = { "IDLE", "BASELINE", "MEASURE", "VERIFY" };
  out.print(F("AT,"));
  out.println(names[p]);
}

bool autotuneRunning() {
  return phase != AT_IDLE;
}

void autotuneStart() {
  if (phase != AT_IDLE) return;
  if (state != STATE_BALANCING) {
    out.println(F("E, AUTOTUNE needs the robot balancing"));
    return;
  }
  if (throttleCmd != 0 || steerCmd != 0) {
    out.println(F("E, AUTOTUNE needs throttle and steer at 0"));
    return;
  }
  saveTuning();
  setEvent("Autotune ON");
  rateSum = 0;
  startPhase(AT_BASELINE);
}

void autotuneAbort(const char *reason) {
  if (phase == AT_IDLE) return;
  restoreTuning();
  phase = AT_IDLE;
  out.print(F("E, autotune aborted: "));
  out.println(reason);
  setEvent("Autotune aborted");
  PIDParamTX();
}

void autotuneOnFall() {
  autotuneAbort("fell");
}

// Wobble added to the pitch setpoint while measuring: two sines, so the fit
// sees both slow and fast motion. Peak about autotuneAmplitude.
float autotuneExcitation() {
  if (phase != AT_MEASURE) return 0;
  float t = phaseTime;
  float s = sin(2 * (float)M_PI * 0.6f * t) + 0.7f * sin(2 * (float)M_PI * 1.7f * t + 1.0f);
  return cfg.autotuneAmplitude * s / 1.7f;
}

// A second, independent wobble added straight to the wheels' acceleration
// (LQR and MPC), in m/s^2. Through the pitch setpoint alone the controller
// makes the acceleration follow the pitch, so the two are nearly collinear and
// L and kappa are hard to tell apart; this keeps them distinguishable.
float autotuneAccelExcitation() {
  if (phase != AT_MEASURE) return 0;
  float t = phaseTime;
  float s = sin(2 * (float)M_PI * 2.3f * t + 0.5f) + 0.8f * sin(2 * (float)M_PI * 3.9f * t + 2.0f);
  return AT_ACCEL_DITHER * s / 1.8f;
}

static void report(bool accepted, float L, float kappa, float offset, float r2, float after) {
  out.print(F("AT,RESULT,"));
  out.print(accepted ? 1 : 0);
  out.print(',');
  out.print(L, 3);
  out.print(',');
  out.print(kappa, 3);
  out.print(',');
  out.print(offset, 2);
  out.print(',');
  out.print(r2, 2);
  out.print(',');
  out.print(baselineScore, 4);
  out.print(',');
  out.println(after, 4);
}

// Last fit, kept for the report after verification.
static float fitL, fitKappa, fitOffset, fitR2;

static void finishMeasure() {
  // Least squares of pitch'' on (pitch, accel) with intercept, from the covariances.
  float n1 = samples > 1 ? samples - 1 : 1;
  float s00 = cov[0] / n1, s01 = cov[1] / n1, s02 = cov[2] / n1;
  float s11 = cov[3] / n1, s12 = cov[4] / n1, s22 = cov[5] / n1;
  float det = s00 * s11 - s01 * s01;
  float corr2 = (s00 > 0 && s11 > 0) ? s01 * s01 / (s00 * s11) : 1;

  fitL = fitKappa = fitOffset = fitR2 = 0;
  const char *problem = NULL;
  if (samples < 100 || det <= 0 || corr2 > 0.98f) {
    problem = "not enough motion to measure";
  } else {
    float p1 = (s02 * s11 - s12 * s01) / det;
    float p3 = (s00 * s12 - s01 * s02) / det;
    float p2 = mean[2] - p1 * mean[0] - p3 * mean[1];
    float residual = s22 - p1 * s02 - p3 * s12;
    fitR2 = s22 > 0 ? 1 - residual / s22 : 0;
    if (p1 <= 0 || p3 >= 0) {
      problem = "model signs wrong: check imuInvert and motor directions";
    } else {
      fitL = G_ACC / p1;
      fitKappa = -p3 * fitL;
      fitOffset = -p2 / p1 * 57.29578f;
      if (fitKappa < MIN_COUPLING || fitKappa > MAX_COUPLING)
        problem = "wheel coupling implausible: wheels slipping or skipping steps?";
      else if (fitL < MIN_LENGTH || fitL > MAX_LENGTH)
        problem = "length out of range";
      else if (fitR2 < MIN_R2)
        problem = "model doesn't fit the motion";
    }
  }

  if (problem) {
    report(false, fitL, fitKappa, fitOffset, fitR2, 0);
    autotuneAbort(problem);
    return;
  }

  cfg.pendulumCoupling = fitKappa;
  applyGainsForLength(fitL);
  setZero(fitOffset);
  PIDParamTX();
  startPhase(AT_VERIFY);
}

static void finishVerify() {
  float after = score / scoreTime;
  // Allow for run-to-run noise: reject only if clearly worse.
  bool better = after <= baselineScore * 1.5f + 0.01f;
  report(better, fitL, fitKappa, fitOffset, fitR2, after);
  if (!better) {
    autotuneAbort("new gains balance worse");
    return;
  }
  phase = AT_IDLE;
  setEvent("Autotune done");
  PIDParamTX();
}

void autotuneStep(float dt, float angleError) {
  if (phase == AT_IDLE) return;
  if (throttleCmd != 0 || steerCmd != 0) { autotuneAbort("driving"); return; }
  if (fabs(angleError) > ABORT_ERROR) { autotuneAbort("tilt error too large"); return; }

  phaseTime += dt;

  if (phase == AT_BASELINE || phase == AT_VERIFY) {
    if (phaseTime > (phase == AT_VERIFY ? VERIFY_SETTLE_S : SETTLE_S)) {
      score += angleError * angleError * dt;
      scoreTime += dt;
      // Balancing in place, the true pitch rate averages to zero, so the
      // average gyro reading is its bias.
      rateSum += pitchRate / 57.29578f * dt;
    }
    if (phase == AT_BASELINE && phaseTime >= BASELINE_S) {
      baselineScore = score / scoreTime;
      gyroBias = rateSum / scoreTime;
      fitReset();
      startPhase(AT_MEASURE);
    } else if (phase == AT_VERIFY && phaseTime >= VERIFY_S) {
      finishVerify();
    }
    return;
  }

  // AT_MEASURE
  float rate = pitchRate / 57.29578f - gyroBias;   // rad/s, from the gyro
  if (!havePrev) {
    prevRate = rate;
    prevSpeed = groundSpeed;
    delayedAccel = 0;
    fitPitch = pitchRaw / 57.29578f;
    fPitch.reset(fitPitch);
    fAccel.reset(0);
    fPitchAcc.reset(0);
    havePrev = true;
    return;
  }
  // Pitch for the fit: integrated gyro, pulled only very slowly (FIT_PITCH_TAU)
  // towards the balance filter's pitch. The balance filter trusts the
  // accelerometer within 0.5 s, and at the wobble frequencies the robot's own
  // acceleration leaks into it, which would bias the fit.
  fitPitch += rate * dt + dt / FIT_PITCH_TAU * (pitchRaw / 57.29578f - fitPitch);
  float thetaAcc = (rate - prevRate) / dt;
  // The MPU-6050's internal low-pass delays the gyro by ~5 ms; the step rate
  // isn't delayed, so delay the acceleration by one control step to match.
  float accel = delayedAccel;
  delayedAccel = (groundSpeed - prevSpeed) / dt;
  prevRate = rate;
  prevSpeed = groundSpeed;
  float theta = fitPitch;

  float w = 2 * (float)M_PI * FIT_CUTOFF_HZ * dt;
  float k = w / (1 + w);
  float x[3] = { fPitch.step(theta, k), fAccel.step(accel, k), fPitchAcc.step(thetaAcc, k) };
  if (phaseTime > SETTLE_S) fitAdd(x);

  if (phaseTime >= MEASURE_S) finishMeasure();
}

