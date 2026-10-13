// Model predictive control (ESP32-S3 only: "SCMD controller 2").
//
// Same model, state and disturbance observer as the LQR (lqr.ino). The LQR
// clips its acceleration and speed when it runs into the steppers' limits; the
// MPC looks MPC_N * MPC_STEP seconds ahead and changes course early enough not
// to run into them:
//
//   |acceleration| <= lqrAccelLimit        (what the steppers can do)
//   |wheel speed|  <= full speed           (what the steppers can reach)
//
// It is "pre-stabilised" (closed-loop paradigm): the plan is the LQR plus a
// correction, u_k = -K x_k + c_k, and only the corrections c are optimised.
// With K the LQR gain and P its Riccati solution for the plan's step, the cost
// of a plan is exactly
//
//   x0' P x0 + sum_k (R + B'PB) c_k^2  +  penalties for exceeding the limits
//
// so without limits in sight the best plan is c = 0: exactly the LQR. Planning
// around the LQR-stabilised robot also keeps the numbers well conditioned over
// a long horizon (planning the raw unstable robot 1.5 s ahead does not work in
// 32-bit floats). The limits are stiff penalties, minimised with FISTA
// (accelerated gradient); each gradient is one forward pass of the model and
// one backward (adjoint) pass, ~60 * MPC_N flops. The first correction is added
// to the LQR running at the full control rate.

#if BOARD_ESP32

#ifndef MPC_N
#define MPC_N 40                 // plan steps
#endif
#ifndef MPC_STEP
#define MPC_STEP 0.03f           // s per plan step: plans 1.2 s ahead
#endif
#ifndef MPC_ITERATIONS
#define MPC_ITERATIONS 30
#endif
#ifndef MPC_PENALTY
#define MPC_PENALTY 200.0f       // how much stiffer the limits are than the cost of a correction
#endif

static Mat4 mpcA;                // plan-step model
static float mpcB[4];
static float mpcK[4];            // LQR gain for the plan step
static float mpcW;               // R + B'PB: cost of a correction
static float mpcRhoA, mpcRhoV;   // penalty weights for acceleration and speed excess
static float mpcStepSize;        // 1 / Lipschitz constant of the gradient
static float mpcC[MPC_N];        // last corrections (warm start)
static float mpcBuiltFor[6];
static bool mpcValid = false;
static uint16_t mpcLastIterations = 0, mpcLastMicros = 0;

// One plan: forward pass from x0 with corrections c. Fills u (accelerations),
// v (predicted ground speed after each step) and the states xs.
static void mpcForward(const float x0[4], float speedOffset, const float c[MPC_N], float u[MPC_N], float v[MPC_N],
                       float xs[MPC_N + 1][4]) {
  memcpy(xs[0], x0, sizeof(float) * 4);
  for (uint8_t k = 0; k < MPC_N; k++) {
    float uk = c[k];
    for (uint8_t i = 0; i < 4; i++) uk -= mpcK[i] * xs[k][i];
    u[k] = uk;
    for (uint8_t i = 0; i < 4; i++) {
      float s = mpcB[i] * uk;
      for (uint8_t j = 0; j < 4; j++) s += mpcA[i][j] * xs[k][j];
      xs[k + 1][i] = s;
    }
    v[k] = xs[k + 1][1] + speedOffset;   // state 1 is the speed error: add the reference back
  }
}

static float excess(float value, float limit) {
  if (value > limit) return value - limit;
  if (value < -limit) return value + limit;
  return 0;
}

// Backward (adjoint) pass for x_{k+1} = A x_k + B u_k, u_k = -K x_k + c_k:
// given d(cost)/d(u_k) from the u terms and d(cost)/d(v_k) from the speed
// terms, returns d(cost)/d(c_k) without the 2 W c_k part.
static void mpcAdjoint(const float du[MPC_N], const float dv[MPC_N], float grad[MPC_N]) {
  float lambda[4] = { 0, 0, 0, 0 };     // d(cost)/d(x_{k+1})
  for (int k = MPC_N - 1; k >= 0; k--) {
    lambda[1] += dv[k];
    float gu = du[k];
    for (uint8_t i = 0; i < 4; i++) gu += mpcB[i] * lambda[i];
    grad[k] = gu;
    float next[4];
    for (uint8_t j = 0; j < 4; j++) {
      float s = -mpcK[j] * gu;
      for (uint8_t i = 0; i < 4; i++) s += mpcA[i][j] * lambda[i];
      next[j] = s;
    }
    memcpy(lambda, next, sizeof(next));
  }
}

static void mpcGradient(const float x0[4], float speedOffset, float aMax, float vMax, const float c[MPC_N],
                        float grad[MPC_N]) {
  static float xs[MPC_N + 1][4];
  float u[MPC_N], v[MPC_N], du[MPC_N], dv[MPC_N];
  mpcForward(x0, speedOffset, c, u, v, xs);
  for (uint8_t k = 0; k < MPC_N; k++) {
    du[k] = 2 * mpcRhoA * excess(u[k], aMax);
    dv[k] = 2 * mpcRhoV * excess(v[k], vMax);
  }
  mpcAdjoint(du, dv, grad);
  for (uint8_t k = 0; k < MPC_N; k++) grad[k] += 2 * mpcW * c[k];
}

// Largest singular value squared of the map from corrections to accelerations
// (which = 0) or to speeds (which = 1), by power iteration.
static float mpcSensitivity(uint8_t which) {
  static float xs[MPC_N + 1][4];
  float c[MPC_N], u[MPC_N], v[MPC_N], du[MPC_N], dv[MPC_N], g[MPC_N];
  float zero[4] = { 0, 0, 0, 0 };
  for (uint8_t k = 0; k < MPC_N; k++) c[k] = 1;
  float estimate = 0;
  for (uint8_t it = 0; it < 40; it++) {
    mpcForward(zero, 0, c, u, v, xs);       // linear part only
    for (uint8_t k = 0; k < MPC_N; k++) {
      du[k] = which == 0 ? u[k] : 0;
      dv[k] = which == 1 ? v[k] : 0;
    }
    mpcAdjoint(du, dv, g);                  // g = J'J c
    float gn = 0, cn = 0;
    for (uint8_t k = 0; k < MPC_N; k++) {
      gn += g[k] * g[k];
      cn += c[k] * c[k];
    }
    if (gn <= 0) return 0;
    estimate = sqrt(gn / cn);
    float inv = 1 / sqrt(gn);
    for (uint8_t k = 0; k < MPC_N; k++) c[k] = g[k] * inv;
  }
  return estimate;
}

static bool mpcBuild() {
  float L = cfg.pendulumLength;
  // The LQR weights are given for a 0.2 m robot; scale them like the gain (lqr.ino).
  float s = L / LQR_REF_LENGTH, r = sqrt(s);
  float q[4] = {
    1 / sq(cfg.lqrMaxPosition * s),
    1 / sq(cfg.lqrMaxSpeed * r),
    1 / sq(cfg.lqrMaxAngle / 57.29578f),
    1 / sq(cfg.lqrMaxRate / r / 57.29578f),
  };
  float R = 1 / sq(cfg.lqrMaxAccel);
  Mat4 P;
  lqrModel(L, MPC_STEP, mpcA, mpcB);
  if (!lqrSolve(L, MPC_STEP, q, R, mpcK, &P)) return false;
  float bpb = 0;
  for (uint8_t i = 0; i < 4; i++)
    for (uint8_t j = 0; j < 4; j++) bpb += mpcB[i] * P[i][j] * mpcB[j];
  mpcW = R + bpb;
  // Exceeding a limit by x costs MPC_PENALTY times a correction of the same
  // size (speed scaled to acceleration by aMax / vMax).
  float aMax = cfg.lqrAccelLimit, vMax = fullSpeedMps();
  mpcRhoA = MPC_PENALTY * mpcW;
  mpcRhoV = MPC_PENALTY * mpcW * sq(aMax / vMax);
  float lip = 2 * mpcW + 2 * mpcRhoA * mpcSensitivity(0) + 2 * mpcRhoV * mpcSensitivity(1);
  mpcStepSize = 1 / (1.1f * lip);
  memset(mpcC, 0, sizeof(mpcC));
  return true;
}

static bool mpcUpToDate() {
  float now[6] = { cfg.pendulumLength, cfg.lqrMaxPosition, cfg.lqrMaxSpeed, cfg.lqrMaxAngle, cfg.lqrMaxRate, cfg.lqrMaxAccel };
  if (mpcValid && memcmp(now, mpcBuiltFor, sizeof(now)) == 0) return true;
  // A few ms on the ESP32-S3, only when L or a weight changes (e.g. after autotune).
  mpcValid = mpcBuild();
  memcpy(mpcBuiltFor, now, sizeof(now));
  if (!mpcValid) Serial.println(F("E, MPC: could not build the plan, using LQR"));
  return mpcValid;
}

// x = [position error, speed error, pitch - balance angle, pitch rate] (SI),
// speed = actual ground speed. Returns the wheel acceleration to apply now:
// the LQR's, plus the first planned correction.
float mpcAccel(const float x[4], float speed, float referenceSpeed) {
  (void)referenceSpeed;
  float a = 0;
  for (uint8_t i = 0; i < 4; i++) a -= lqrK[i] * x[i];
  if (!mpcUpToDate()) return a;

  uint32_t start = micros();
  float aMax = cfg.lqrAccelLimit;
  float vMax = fullSpeedMps();
  float speedOffset = speed - x[1];   // the reference speed, as the state sees it

  // FISTA from the previous corrections.
  float c[MPC_N], y[MPC_N], cOld[MPC_N], g[MPC_N];
  memcpy(c, mpcC, sizeof(c));
  memcpy(y, c, sizeof(c));
  memcpy(cOld, c, sizeof(c));
  float t = 1;
  uint16_t it;
  for (it = 0; it < MPC_ITERATIONS; it++) {
    mpcGradient(x, speedOffset, aMax, vMax, y, g);
    for (uint8_t k = 0; k < MPC_N; k++) c[k] = y[k] - mpcStepSize * g[k];
    float tNext = (1 + sqrt(1 + 4 * t * t)) / 2;
    float beta = (t - 1) / tNext;
    float moved = 0;
    for (uint8_t k = 0; k < MPC_N; k++) {
      y[k] = c[k] + beta * (c[k] - cOld[k]);
      moved += fabs(c[k] - cOld[k]);
      cOld[k] = c[k];
    }
    t = tNext;
    if (moved < 1e-4f) break;
  }
  memcpy(mpcC, c, sizeof(c));   // warm start (one control period is small next to MPC_STEP)
  uint32_t took = micros() - start;
  mpcLastIterations = it;
  mpcLastMicros = took > 65535 ? 65535 : took;
  return a + c[0];
}

void mpcStats(uint16_t &iterations, uint16_t &microseconds) {
  iterations = mpcLastIterations;
  microseconds = mpcLastMicros;
}

#endif
