// Model predictive control ("SCMD controller 2").
//
// Same model, state and disturbance observer as the LQR (lqr.ino). The LQR
// clips its acceleration and speed when it runs into the steppers' limits; the
// MPC plans MPC_N steps of MPC_STEP seconds ahead within them:
//
//   |acceleration| <= lqrAccelLimit        (what the steppers can do)
//   |wheel speed|  <= full speed           (what the steppers can reach)
//
// The plan is solved by TinyMPC (src/tinympc: ADMM on a cached Riccati
// solution, built for microcontrollers) every MPC_STEP, with the LQR's cost
// (weights scaled with L like the LQR gain). Solving takes milliseconds, so on
// the ESP32-S3 it runs on the second core (core 0) while the balance loop keeps
// its 500 Hz on core 1. Between plans, the balance loop follows the latest one
// with the LQR gain:
//
//   a = u_plan(t) - K (x - x_plan(t))
//
// so a late or missing plan degrades to plain LQR, never to no control. With
// no limit in sight the plan is the LQR's own trajectory and the MPC behaves
// like the LQR.

// MPC_N, MPC_STEP, MpcRequest and MpcPlan are in bailey.h (the Arduino IDE's
// generated prototypes need them before the .ino files).
#ifndef MPC_RHO
#define MPC_RHO 20.0f            // ADMM penalty, for the cost normalised to R = 1
#endif
#ifndef MPC_MAX_ITER
#define MPC_MAX_ITER 60
#endif
#define MPC_STEP_US ((uint32_t)(MPC_STEP * 1e6f))
#define MPC_PLAN_TIMEOUT_US (3 * MPC_STEP_US)   // older plans are ignored
#define MPC_BIG 1e6f             // "no bound"

static MpcPlan mpcPlan;          // latest plan, shared with the balance loop
static MpcRequest mpcRequest;    // latest request, shared with the solver

// ---------- solver side (core 0 on the ESP32-S3) ----------

static TinySolver *mpcSolver = NULL;
static float mpcBuiltFor[6];

// Model and cost for the current L and weights. The solver memory is
// allocated once; afterwards only the model, cost and TinyMPC's cache change.
static bool mpcUpToDate() {
  float now[6] = { cfg.pendulumLength, cfg.lqrMaxPosition, cfg.lqrMaxSpeed, cfg.lqrMaxAngle, cfg.lqrMaxRate, cfg.lqrMaxAccel };
  if (mpcSolver && memcmp(now, mpcBuiltFor, sizeof(now)) == 0) return true;

  float L = cfg.pendulumLength;
  // The LQR weights are given for a 0.2 m robot; scale them like the gain (lqr.ino),
  // normalised so R = 1: the ADMM penalty then means the same for any weights.
  float s = L / LQR_REF_LENGTH, r = sqrt(s);
  float R = 1 / sq(cfg.lqrMaxAccel);
  tinyVector q(4);
  q << 1 / sq(cfg.lqrMaxPosition * s) / R, 1 / sq(cfg.lqrMaxSpeed * r) / R,
       1 / sq(cfg.lqrMaxAngle / 57.29578f) / R, 1 / sq(cfg.lqrMaxRate / r / 57.29578f) / R;
  tinyMatrix Q = q.asDiagonal(), Rm = tinyMatrix::Identity(1, 1);
  Mat4 Ad;
  Vec4 Bd;
  lqrModel(L, MPC_STEP, Ad, Bd);
  tinyMatrix A = Ad, B = Bd, f = tinyMatrix::Zero(4, 1);

  if (!mpcSolver) {
    if (tiny_setup(&mpcSolver, A, B, f, Q, Rm, MPC_RHO, 4, 1, MPC_N, 0) != 0) {
      mpcSolver = NULL;
      return false;
    }
    mpcSolver->settings->max_iter = MPC_MAX_ITER;
  } else {
    TinyWorkspace *w = mpcSolver->work;
    w->Adyn = A;
    w->Bdyn = B;
    w->Q = q.array() + MPC_RHO;   // as tiny_setup stores them
    w->R = tinyVector::Constant(1, 1 + MPC_RHO);
    if (tiny_precompute_and_set_cache(mpcSolver->cache, A, B, f, Q, Rm, 4, 1, MPC_RHO, 0) != 0) return false;
  }
  memcpy(mpcBuiltFor, now, sizeof(now));
  return true;
}

// Moves last plan's slack and dual variables one step along, as the starting guess.
static void mpcShift(tinyMatrix &m) {
  int n = m.cols();
  m.leftCols(n - 1) = m.rightCols(n - 1).eval();
}

static void mpcSolve(const MpcRequest &req, MpcPlan &plan) {
  uint32_t start = micros();
  plan.valid = false;
  if (!mpcUpToDate()) return;
  TinyWorkspace *w = mpcSolver->work;

  // Bounds: acceleration on every step, speed from the first step on (x[0]
  // is the measurement: it may be slightly over and must not make the problem
  // infeasible). State 1 is the speed error: speed = error + reference.
  tinyMatrix xMin = tinyMatrix::Constant(4, MPC_N, -MPC_BIG), xMax = tinyMatrix::Constant(4, MPC_N, MPC_BIG);
  xMin.block(1, 1, 1, MPC_N - 1).setConstant(-req.vMax - req.referenceSpeed);
  xMax.block(1, 1, 1, MPC_N - 1).setConstant(req.vMax - req.referenceSpeed);
  tinyMatrix uMin = tinyMatrix::Constant(1, MPC_N - 1, -req.aMax), uMax = tinyMatrix::Constant(1, MPC_N - 1, req.aMax);
  tiny_set_bound_constraints(mpcSolver, xMin, xMax, uMin, uMax);

  mpcShift(w->v); mpcShift(w->vnew); mpcShift(w->g);
  mpcShift(w->z); mpcShift(w->znew); mpcShift(w->y);
  tinyVector x0(4);
  x0 << req.x0[0], req.x0[1], req.x0[2], req.x0[3];
  tiny_set_x0(mpcSolver, x0);
  tiny_solve(mpcSolver);

  // The plan: TinyMPC's rollout (x, u) through the model from x0.
  bool finite = w->x.allFinite() && w->u.allFinite();
  for (uint8_t k = 0; k < MPC_N; k++)
    for (uint8_t i = 0; i < 4; i++) plan.x[k][i] = w->x(i, k);
  for (uint8_t k = 0; k < MPC_N - 1; k++) plan.u[k] = w->u(0, k);
  plan.at = req.at;
  plan.generation = req.generation;
  plan.iterations = w->iter;
  uint32_t took = micros() - start;
  plan.micros = took > 65535 ? 65535 : took;
  plan.valid = finite;
  if (!finite) {
    // Start the next solve from scratch.
    w->v.setZero(); w->vnew.setZero(); w->g.setZero();
    w->z.setZero(); w->znew.setZero(); w->y.setZero();
  }
}

// ---------- threading ----------

#ifdef BAILEY_SIMULATOR
// The simulator has no second core: solve right away (its time isn't simulated).
static void mpcSubmit(const MpcRequest &req) {
  mpcRequest = req;
  mpcSolve(mpcRequest, mpcPlan);
}
static void mpcReadPlan(MpcPlan &out) { out = mpcPlan; }
static void mpcWritePlan(const MpcPlan &in) { mpcPlan = in; }
#else
static portMUX_TYPE mpcLock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t mpcTask = NULL;

static void mpcTaskLoop(void *) {
  static MpcRequest req;
  static MpcPlan plan;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    taskENTER_CRITICAL(&mpcLock);
    req = mpcRequest;
    taskEXIT_CRITICAL(&mpcLock);
    mpcSolve(req, plan);
    taskENTER_CRITICAL(&mpcLock);
    mpcPlan = plan;
    taskEXIT_CRITICAL(&mpcLock);
  }
}

static void mpcSubmit(const MpcRequest &req) {
  if (!mpcTask) {
    // Core 0; the Arduino loop (balance) runs on core 1. Low priority, 16 KB stack for Eigen.
    xTaskCreatePinnedToCore(mpcTaskLoop, "mpc", 16384, NULL, 1, &mpcTask, 0);
    if (!mpcTask) return;
  }
  taskENTER_CRITICAL(&mpcLock);
  mpcRequest = req;
  taskEXIT_CRITICAL(&mpcLock);
  xTaskNotifyGive(mpcTask);      // if it's still busy, it picks up this request next
}

static void mpcReadPlan(MpcPlan &out) {
  taskENTER_CRITICAL(&mpcLock);
  out = mpcPlan;
  taskEXIT_CRITICAL(&mpcLock);
}

static void mpcWritePlan(const MpcPlan &in) {
  taskENTER_CRITICAL(&mpcLock);
  mpcPlan = in;
  taskEXIT_CRITICAL(&mpcLock);
}
#endif

// ---------- balance loop side (core 1) ----------

static uint32_t mpcLastRequest = 0;
static bool mpcRequested = false;
static uint16_t mpcGeneration = 0;   // plans requested before the last mpcReset are ignored
static MpcPlan mpcCurrent;       // the balance loop's copy of the latest plan

// x = [position error, speed error, pitch - balance angle, pitch rate] (SI).
// Returns the wheel acceleration to apply now.
float mpcAccel(const float x[4], float referenceSpeed) {
  uint32_t now = micros();
  if (!mpcRequested || now - mpcLastRequest >= MPC_STEP_US) {
    MpcRequest req;
    memcpy(req.x0, x, sizeof(req.x0));
    req.referenceSpeed = referenceSpeed;
    req.aMax = cfg.lqrAccelLimit;
    req.vMax = fullSpeedMps();
    req.at = now;
    req.generation = mpcGeneration;
    mpcLastRequest = now;
    mpcRequested = true;
    mpcSubmit(req);
  }

  mpcReadPlan(mpcCurrent);
  const MpcPlan &plan = mpcCurrent;
  float a = 0;
  uint32_t age = now - plan.at;
  if (!plan.valid || plan.generation != mpcGeneration || age > MPC_PLAN_TIMEOUT_US) {
    for (uint8_t i = 0; i < 4; i++) a -= lqrK[i] * x[i];   // no plan: plain LQR
    return a;
  }
  // Where the plan says the robot should be now (linear between knots), and its input.
  float t = age / (float)MPC_STEP_US;
  uint8_t k = (uint8_t)t;
  if (k > MPC_N - 2) k = MPC_N - 2;
  float frac = t - k;
  if (frac > 1) frac = 1;
  a = plan.u[k];
  for (uint8_t i = 0; i < 4; i++) {
    float xp = plan.x[k][i] + frac * (plan.x[k + 1][i] - plan.x[k][i]);
    a -= lqrK[i] * (x[i] - xp);
  }
  return a;
}

void mpcStats(uint16_t &iterations, uint16_t &microseconds) {
  iterations = mpcCurrent.iterations;
  microseconds = mpcCurrent.micros;
}

// Back to "no plan" (motors off, controller switched): the next call starts fresh.
void mpcReset() {
  mpcRequested = false;
  mpcGeneration++;
  MpcPlan none;
  memset(&none, 0, sizeof(none));
  mpcWritePlan(none);
  mpcCurrent = none;
}
