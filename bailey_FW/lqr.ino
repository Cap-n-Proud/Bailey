// LQR balance controller with a disturbance observer (steppers).
//
// Model: an inverted pendulum whose wheels' acceleration we command directly
// (steppers follow the commanded speed exactly):
//
//   state x = [position (m), speed (m/s), pitch (rad), pitch rate (rad/s)]
//   input a = wheel acceleration (m/s^2, relative to the body: the steppers'
//             housings are bolted to it)
//   p' = v,  v' = a,  pitch' = rate,  rate' = (g * pitch - kappa * a) / L + d
//
// L (how fast it falls) and kappa (how hard the wheels' acceleration tips it)
// are measured by the autotuner; d is an unknown disturbance: centre of mass
// not where the IMU zero says, a payload, someone leaning on it. A cart-pole
// has kappa = 1. A real two-wheeler has more, because the wheels turn with the
// body and the motors push back on it: about 1.4 for Bailey. (Found by checking
// the simulator against the MuJoCo physics engine, test/mujoco_check.py; with
// kappa = 1 the model was 55 % off in how fast the robot falls.) The LQR gain K minimises the sum over time of
// x' Q x + R a^2, with Q and R set from "how far each quantity may stray"
// (lqrMax* settings, given for a 0.2 m robot).
//
// The Riccati equation is solved on the board (Eigen, a few ms) for the
// reference length and the current kappa: at boot, when a weight changes while
// the motors are off, or when kappa changes (autotune). Robots that differ only
// in L behave identically with time scaled by sqrt(L / g), so the gain for the
// measured L follows exactly by rescaling (K_pos ~ 1/L, K_speed ~ 1/sqrt(L),
// K_pitch const, K_rate ~ sqrt(L)), which is instant and safe while balancing.
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
// While driving, track the speed, not a position the robot has fallen behind:
// catching up a lag built during the throttle ramp would mean overshooting the
// speed, and near top speed there is no room left to tilt back and brake.
#define MAX_POSITION_ERROR_DRIVING 0.05f

#define LQR_REF_LENGTH 0.20f      // m, length the weights are given for

static float lqrKRef[4];          // gain solved for LQR_REF_LENGTH
static float lqrK[4];             // a = -(K0 p + K1 v + K2 pitch + K3 rate), for lqrLength
static float lqrLength = 0;       // L lqrK is scaled for
static float lqrCoupling = 0;     // kappa lqrKRef was solved for
static bool lqrSolved = false;
bool lqrWeightsChanged = true;    // set when an lqrMax* setting changes
float balanceAngle = 0;           // deg, from the disturbance observer (telemetry)
// Observer state: pitch rate (rad/s) and the balance angle (rad). Storing the
// disturbance as the angle it implies (d = -g * angle / L) keeps the estimate
// valid when L or the IMU zero change.
static float dobRate = 0, dobAngle = 0;
static float refPosition = 0, position = 0, speedCmd = 0;   // m, m, m/s

// ---------- model and Riccati solver (Eigen) ----------

// Exact discretisation for a step dt (the input is held constant over the step).
// With w = sqrt(g / L) the pitch obeys pitch'' = w^2 pitch - kappa a / L, whose
// solution over one step is in cosh/sinh; the cart part is plain kinematics.
void lqrModel(float L, float kappa, float dt, Mat4 &Ad, Vec4 &Bd) {
  float w = sqrt(G_ACC / L), c = cosh(w * dt), s = sinh(w * dt);
  Ad << 1, dt, 0, 0,
        0, 1,  0, 0,
        0, 0,  c, s / w,
        0, 0,  w * s, c;
  Bd << dt * dt / 2, dt, -kappa * (c - 1) / G_ACC, -kappa * w * s / G_ACC;
}

// Solves the discrete algebraic Riccati equation
//   P = A'PA - A'PB (r + B'PB)^-1 B'PA + Q
// with the structure-preserving doubling algorithm (Chu, Fan, Lin, Wang 2004:
// converges in ~20 iterations, where plain Riccati iteration needs thousands at
// a 2 ms step), then K = (r + B'PB)^-1 B'PA. Matrix algebra by Eigen (fixed
// size, no heap). Benchmarked against the old hand-written code and PX4's
// matrix library: without SIMD, as on the ESP32-S3, this is the fastest of the
// three. The solution is checked against SciPy's solve_discrete_are in the
// simulator tests.
// Returns false if it didn't converge.
bool lqrSolve(float L, float kappa, float dt, const float qDiag[4], float r, float K[4], Mat4 *P) {
  Mat4 Ad;
  Vec4 Bd;
  lqrModel(L, kappa, dt, Ad, Bd);
  Mat4 A = Ad;
  Mat4 G = Bd * Bd.transpose() / r;
  Mat4 H = Eigen::Map<const Vec4>(qDiag).asDiagonal();
  Eigen::Matrix<float, 4, 8> rhs, sol;

  for (uint8_t it = 0; it < 40; it++) {
    // One LU factorisation of W = I + G H, solved for [A G] at once.
    Mat4 W = G * H;
    W.diagonal().array() += 1;
    rhs << A, G;
    sol = W.partialPivLu().solve(rhs);                      // [W^-1 A, W^-1 G]
    Mat4 Hn = H + A.transpose() * H * sol.leftCols<4>();    // H + A' H W^-1 A
    G += A * sol.rightCols<4>() * A.transpose();            // G + A W^-1 G A'
    A = A * sol.leftCols<4>();                              // A W^-1 A
    Hn = (Hn + Hn.transpose()) * 0.5f;                      // keep P exactly symmetric
    float change = (Hn - H).cwiseAbs().sum(), size = Hn.cwiseAbs().sum();
    H = Hn;
    if (!(size < 1e30f)) return false;                      // diverged (also catches NaN)
    if (change <= 1e-6f * size) {
      // H is P (also the MPC's terminal cost).
      if (P) *P = H;
      Vec4 PB = H * Bd;
      Eigen::Map<Vec4> gain(K);
      gain = (Ad.transpose() * PB) / (r + Bd.dot(PB));
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
  float kappa = cfg.pendulumCoupling;
  if (!lqrSolve(LQR_REF_LENGTH, kappa, CONTROL_PERIOD_US * 1e-6f, q, r, K)) {
    out.println(F("E, LQR: Riccati equation did not converge, keeping the old gain"));
    return false;
  }
  memcpy(lqrKRef, K, sizeof(K));
  lqrCoupling = kappa;
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
  // The solve takes well under a control period, but a new weight could make
  // a much harder controller: those wait until the motors are off. A new kappa
  // (autotune) is the same robot better described: applied right away.
  if ((!lqrSolved || lqrWeightsChanged) && !balancing) lqrSolveReference();
  else if (lqrSolved && lqrCoupling != cfg.pendulumCoupling) lqrSolveReference();
  if (lqrSolved && lqrLength != cfg.pendulumLength) lqrScale();
}

void LQRParamTX() {
  out.print(F("LQR"));
  for (uint8_t i = 0; i < 4; i++) {
    out.print(',');
    out.print(lqrK[i], 3);
  }
  out.print(',');
  out.println(lqrLength, 3);
}

// The IMU zero moved by delta degrees (autotune, CalibrateZero): the measured
// pitch shifts by -delta, so shift the observer's balance angle with it.
void lqrZeroShifted(float delta) {
  dobAngle -= delta / 57.29578f;
}

// Called when balancing starts: start from where the robot is.
void lqrReset() {
  mpcReset();
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
    dobRate += (G_ACC * (theta - dobAngle) - cfg.pendulumCoupling * wheelAccel) / L * dt;
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
  float maxErr = referenceSpeed != 0 ? MAX_POSITION_ERROR_DRIVING : MAX_POSITION_ERROR;
  float posErr = constrain(position - refPosition, -maxErr, maxErr);
  refPosition = position - posErr;
  float thetaRef = thetaBalance + extraPitch / 57.29578f;
  float x[4] = { posErr, groundSpeed - referenceSpeed, theta - thetaRef, rate };
  angleError = x[2] * 57.29578f;

  float a = 0;
  if (cfg.controller == CONTROLLER_MPC) {
    a = mpcAccel(x, referenceSpeed);
  } else {
    for (uint8_t i = 0; i < 4; i++) a -= lqrK[i] * x[i];
  }
  a += autotuneAccelExcitation();
  a = constrain(a, -cfg.lqrAccelLimit, cfg.lqrAccelLimit);

  // Integrate into the speed for the steppers (they follow it exactly).
  float vFull = fullSpeedMps();
  speedCmd = constrain(speedCmd + a * dt, -vFull, vFull);
  return speedCmd / vFull * 100.0f;
}
