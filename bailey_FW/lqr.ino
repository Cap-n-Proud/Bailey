// LQR balance controller with a disturbance observer (steppers).
//
// Model: the robot is an inverted pendulum on a cart whose acceleration we
// command directly (steppers follow the commanded speed exactly):
//
//   state x = [position (m), speed (m/s), pitch (rad), pitch rate (rad/s)]
//   input a = wheel acceleration (m/s^2)
//   p' = v,  v' = a,  pitch' = rate,  rate' = (g * pitch - a) / L + d
//
// L is the effective pendulum length (measured by the autotuner) and d an
// unknown disturbance: centre of mass not where the IMU zero says, a payload,
// someone leaning on it. The LQR gain K minimises the sum over time of
// x' Q x + R a^2, with Q and R set from "how far each quantity may stray"
// (lqrMax* settings, given for a 0.2 m robot).
//
// The Riccati equation is solved on the Arduino for the reference length
// (~0.1 s of float math: done at boot, or when a weight changes while the
// motors are off). Robots that differ only in L behave identically with time
// scaled by sqrt(L / g), so the gain for the measured L follows exactly by
// rescaling (K_pos ~ 1/L, K_speed ~ 1/sqrt(L), K_pitch const, K_rate ~ sqrt(L)),
// which is instant and safe to do while balancing.
//
// The disturbance observer estimates d from the difference between the
// measured pitch rate and what the model predicts, and turns it into the
// pitch the robot actually balances at (balanceAngle = -d L / g). The
// controller regulates the pitch around that angle instead of around the
// IMU zero, so an offset or payload no longer makes the robot drift.
//
// Each control step: a = -K (x - x_ref), integrated into the speed sent to the
// steppers. Driving moves x_ref (speed and position) along the ramped throttle.

#define MAX_POSITION_ERROR 0.3f   // m: if pushed further, don't drive all the way back

#define LQR_REF_LENGTH 0.20f      // m, length the weights are given for

static float lqrKRef[4];          // gain solved for LQR_REF_LENGTH
static float lqrK[4];             // a = -(K0 p + K1 v + K2 pitch + K3 rate), for lqrLength
static float lqrLength = 0;       // L lqrK is scaled for
static bool lqrSolved = false;
bool lqrWeightsChanged = true;    // set when an lqrMax* setting changes
float balanceAngle = 0;           // deg, from the disturbance observer (telemetry)
// Observer state: pitch rate (rad/s) and the balance angle (rad). Storing the
// disturbance as the angle it implies (d = -g * angle / L) keeps the estimate
// valid when L or the IMU zero change.
static float dobRate = 0, dobAngle = 0;
static float refPosition = 0, position = 0, speedCmd = 0;   // m, m, m/s

// ---------- small dense matrix helpers (4x4, row major) ----------

static void matMul(const Mat4 a, const Mat4 b, Mat4 out) {
  Mat4 t;
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) {
      float s = 0;
      for (uint8_t k = 0; k < 4; k++) s += a[i][k] * b[k][j];
      t[i][j] = s;
    }
  memcpy(out, t, sizeof(Mat4));
}

static void matTranspose(const Mat4 a, Mat4 out) {
  Mat4 t;
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) t[i][j] = a[j][i];
  memcpy(out, t, sizeof(Mat4));
}

static void matAdd(const Mat4 a, const Mat4 b, Mat4 out) {
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) out[i][j] = a[i][j] + b[i][j];
}

// Gauss-Jordan with partial pivoting. Returns false if singular.
static bool matInverse(const Mat4 a, Mat4 out) {
  float m[4][8];
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) {
      m[i][j] = a[i][j];
      m[i][j + 4] = (i == j) ? 1 : 0;
    }
  for (uint8_t c = 0; c < 4; c++) {
    uint8_t p = c;
    for (uint8_t r = c + 1; r < 4; r++)
      if (fabs(m[r][c]) > fabs(m[p][c])) p = r;
    if (fabs(m[p][c]) < 1e-12f) return false;
    if (p != c)
      for (uint8_t j = 0; j < 8; j++) { float t = m[c][j]; m[c][j] = m[p][j]; m[p][j] = t; }
    float inv = 1 / m[c][c];
    for (uint8_t j = 0; j < 8; j++) m[c][j] *= inv;
    for (uint8_t r = 0; r < 4; r++) {
      if (r == c) continue;
      float f = m[r][c];
      if (f != 0)
        for (uint8_t j = 0; j < 8; j++) m[r][j] -= f * m[c][j];
    }
  }
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) out[i][j] = m[i][j + 4];
  return true;
}

// Discretised model for step dt (series to third order; |A| dt is ~0.04).
static void lqrModel(float L, float dt, Mat4 Ad, float Bd[4]) {
  Mat4 A = { { 0, 1, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 1 }, { 0, 0, G_ACC / L, 0 } };
  float B[4] = { 0, 1, 0, -1 / L };
  Mat4 A2, A3, ident = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
  matMul(A, A, A2);
  matMul(A2, A, A3);
  // Ad = I + A dt + A^2 dt^2/2 + A^3 dt^3/6;  Bd = (I dt + A dt^2/2 + A^2 dt^3/6) B
  Mat4 S;
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) {
      Ad[i][j] = ident[i][j] + A[i][j] * dt + A2[i][j] * dt * dt / 2 + A3[i][j] * dt * dt * dt / 6;
      S[i][j] = ident[i][j] * dt + A[i][j] * dt * dt / 2 + A2[i][j] * dt * dt * dt / 6;
    }
  for (uint8_t i = 0; i < 4; i++) {
    Bd[i] = 0;
    for (uint8_t j = 0; j < 4; j++) Bd[i] += S[i][j] * B[j];
  }
}

// Solves the discrete algebraic Riccati equation with the structure-preserving
// doubling algorithm (converges in ~20 iterations, unlike plain Riccati
// iteration which needs thousands at a 5 ms step), then K = (R + B'PB)^-1 B'PA.
// Returns false if it didn't converge.
bool lqrSolve(float L, float dt, const float qDiag[4], float r, float K[4]) {
  Mat4 Ad, Gk, Hk, ident = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
  float Bd[4];
  lqrModel(L, dt, Ad, Bd);
  Mat4 Ak;
  memcpy(Ak, Ad, sizeof(Mat4));
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) {
      Gk[i][j] = Bd[i] * Bd[j] / r;
      Hk[i][j] = (i == j) ? qDiag[i] : 0;
    }

  for (uint8_t it = 0; it < 40; it++) {
    Mat4 GH, W, Winv, AkT, T1, T2, Gn, Hn, An;
    matMul(Gk, Hk, GH);
    matAdd(ident, GH, W);
    if (!matInverse(W, Winv)) return false;
    matTranspose(Ak, AkT);
    // A' = A W^-1 A
    matMul(Ak, Winv, T1);
    matMul(T1, Ak, An);
    // Gk' = Gk + A W^-1 Gk A^T
    matMul(T1, Gk, T2);
    matMul(T2, AkT, T2);
    matAdd(Gk, T2, Gn);
    // Hk' = Hk + A^T Hk W^-1 A
    matMul(AkT, Hk, T1);
    matMul(T1, Winv, T1);
    matMul(T1, Ak, T1);
    matAdd(Hk, T1, Hn);

    float change = 0, size = 0;
    for (uint8_t i = 0; i < 4; i++)
      for (uint8_t j = 0; j < 4; j++) {
        change += fabs(Hn[i][j] - Hk[i][j]);
        size += fabs(Hn[i][j]);
      }
    memcpy(Ak, An, sizeof(Mat4));
    memcpy(Gk, Gn, sizeof(Mat4));
    memcpy(Hk, Hn, sizeof(Mat4));
    if (change <= 1e-6f * size) {
      // Hk is P. K = (r + B'PB)^-1 B'P A
      float PB[4], bpb = 0;
      for (uint8_t i = 0; i < 4; i++) {
        PB[i] = 0;
        for (uint8_t j = 0; j < 4; j++) PB[i] += Hk[i][j] * Bd[j];
        bpb += Bd[i] * PB[i];
      }
      for (uint8_t j = 0; j < 4; j++) {
        float s = 0;
        for (uint8_t i = 0; i < 4; i++) s += PB[i] * Ad[i][j];
        K[j] = s / (r + bpb);
      }
      return true;
    }
  }
  return false;
}

// Solve for the reference length (slow: only when the motors are off).
bool lqrSolveReference() {
  float q[4] = {
    1 / (cfg.lqrMaxPosition * cfg.lqrMaxPosition),
    1 / (cfg.lqrMaxSpeed * cfg.lqrMaxSpeed),
    1 / sq(cfg.lqrMaxAngle / 57.29578f),
    1 / sq(cfg.lqrMaxRate / 57.29578f),
  };
  float r = 1 / (cfg.lqrMaxAccel * cfg.lqrMaxAccel);
  float K[4];
  lqrWeightsChanged = false;
  if (!lqrSolve(LQR_REF_LENGTH, CONTROL_PERIOD_US * 1e-6f, q, r, K)) {
    Serial.println(F("E, LQR: Riccati equation did not converge, keeping the old gain"));
    return false;
  }
  memcpy(lqrKRef, K, sizeof(K));
  lqrSolved = true;
  lqrLength = 0;                  // rescale on next use
  return true;
}

// Gain for the current pendulum length, by similarity from the reference.
static void lqrScale() {
  float s = cfg.pendulumLength / LQR_REF_LENGTH;
  float r = sqrt(s);
  lqrK[0] = lqrKRef[0] / s;
  lqrK[1] = lqrKRef[1] / r;
  lqrK[2] = lqrKRef[2];
  lqrK[3] = lqrKRef[3] * r;
  lqrLength = cfg.pendulumLength;
  LQRParamTX();
}

// False until the Riccati equation has been solved once (the PIDs balance meanwhile).
bool lqrReady() {
  return lqrSolved;
}

// Called every control step while idle and before balancing.
void lqrMaintain(bool balancing) {
  if ((!lqrSolved || lqrWeightsChanged) && !balancing) lqrSolveReference();
  if (lqrSolved && lqrLength != cfg.pendulumLength) lqrScale();
}

void LQRParamTX() {
  Serial.print(F("LQR"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(',');
    Serial.print(lqrK[i], 3);
  }
  Serial.print(',');
  Serial.println(lqrLength, 3);
}

// The IMU zero moved by delta degrees (autotune, CalibrateZero): the measured
// pitch shifts by -delta, so shift the observer's balance angle with it.
void lqrZeroShifted(float delta) {
  dobAngle -= delta / 57.29578f;
}

// Called when balancing starts: start from where the robot is.
void lqrReset() {
  refPosition = position = 0;
  speedCmd = groundSpeed;
  dobRate = pitchRate / 57.29578f;
  // Keep the disturbance estimate: the robot's offset doesn't change when it falls.
  lqrMaintain(false);
}

// One control step. referenceSpeed in m/s, extraPitch (deg) is added to the
// pitch reference (autotune wobble). Returns the motor command in % and sets
// angleError (deg) for statistics and the autotuner.
float lqrStep(float dt, float referenceSpeed, float extraPitch, float &angleError) {
  float L = cfg.pendulumLength;
  float theta = pitch / 57.29578f;
  float rate = pitchRate / 57.29578f;

  // --- Disturbance observer (bandwidth dobBandwidth rad/s, double pole) ---
  // Predict the pitch rate from the model, compare with the gyro, and move the
  // balance angle until the model matches.
  if (cfg.dobBandwidth > 0) {
    // Like the gains, the observer's speed scales with the robot's own (sqrt(g / L)).
    float w = cfg.dobBandwidth * sqrt(LQR_REF_LENGTH / L);
    dobRate += (G_ACC * (theta - dobAngle) - wheelAccel) / L * dt;
    float e = rate - dobRate;
    dobRate += 2 * w * dt * e;
    dobAngle -= L / G_ACC * w * w * dt * e;
    dobAngle = constrain(dobAngle, -0.1745f, 0.1745f);   // never believe more than 10 deg
  } else {
    dobAngle = 0;
  }
  float thetaBalance = dobAngle;
  balanceAngle = thetaBalance * 57.29578f;

  // --- State relative to the reference ---
  position += groundSpeed * dt;
  refPosition += referenceSpeed * dt;
  float posErr = constrain(position - refPosition, -MAX_POSITION_ERROR, MAX_POSITION_ERROR);
  refPosition = position - posErr;
  float thetaRef = thetaBalance + extraPitch / 57.29578f;
  float x[4] = { posErr, groundSpeed - referenceSpeed, theta - thetaRef, rate };
  angleError = x[2] * 57.29578f;

  float a = 0;
  for (uint8_t i = 0; i < 4; i++) a -= lqrK[i] * x[i];
  a = constrain(a, -cfg.lqrAccelLimit, cfg.lqrAccelLimit);

  // Integrate into the speed for the steppers (they follow it exactly).
  float vFull = fullSpeedMps();
  speedCmd = constrain(speedCmd + a * dt, -vFull, vFull);
  return speedCmd / vFull * 100.0f;
}
