// Autotuner: particle swarm optimisation (SPO) of the conservative angle PID gains
// (Kp, Ki, Kd) while the robot balances.
//
// "SCMD AUTOTUNE 1" starts it (the robot must be balancing). Each particle is a
// set of gains; it is applied, given SPOSettle ms to settle, then scored for
// SPOConfigEval ms by the mean of (angle error^2 + small motor effort term).
// After every particle has been scored the swarm moves, for maxIterations
// rounds. The search stays within +/- SPOspread of the gains at start.
// When it ends ("SCMD AUTOTUNE 0" or all rounds done) the best gains found are
// applied; send "SCMD SaveCfg 1" to keep them. Falling scores the particle as
// very bad and the search continues once the robot balances again; after
// MAX_FALLS falls it gives up and restores the best gains found so far.

#define DIMS 3
#define MAX_FALLS 3
#define FALL_PENALTY 1e6f

struct Particle {
  float pos[DIMS];
  float vel[DIMS];
  float best[DIMS];
  float bestFitness;
};

static Particle swarm[MAX_PARTICLES];
static float globalBest[DIMS];
static float globalBestFitness;
static float baseline[DIMS];
static float lo[DIMS], hi[DIMS];
static bool tuning = false;
static bool waitingForBalance = false;
static uint8_t numParticles = 0;
static uint8_t current = 0;
static uint8_t iteration = 0;
static uint8_t falls = 0;
static float elapsedMs = 0;
static float cost = 0;

static float rand01() {
  return random(0, 10001) / 10000.0f;
}

static void getGains(float *g) {
  g[0] = cfg.anglePIDConKp;
  g[1] = cfg.anglePIDConKi;
  g[2] = cfg.anglePIDConKd;
}

static void setGains(const float *g) {
  cfg.anglePIDConKp = g[0];
  cfg.anglePIDConKi = g[1];
  cfg.anglePIDConKd = g[2];
}

static void beginEval() {
  setGains(swarm[current].pos);
  elapsedMs = 0;
  cost = 0;
  waitingForBalance = false;
}

static void moveSwarm() {
  const float w = 0.729f;      // inertia
  const float c1 = 1.49445f;   // pull towards the particle's own best
  const float c2 = 1.49445f;   // pull towards the swarm's best
  for (uint8_t i = 0; i < numParticles; i++) {
    Particle &p = swarm[i];
    for (uint8_t d = 0; d < DIMS; d++) {
      float vmax = (hi[d] - lo[d]) * 0.5f;
      float v = w * p.vel[d]
                + c1 * rand01() * (p.best[d] - p.pos[d])
                + c2 * rand01() * (globalBest[d] - p.pos[d]);
      v = constrain(v, -vmax, vmax);
      float x = p.pos[d] + v;
      if (x < lo[d]) { x = lo[d]; v = 0; }
      if (x > hi[d]) { x = hi[d]; v = 0; }
      p.pos[d] = x;
      p.vel[d] = v;
    }
  }
}

static void report(float fitness) {
  Serial.print(F("SPO,"));
  Serial.print(iteration);
  Serial.print(',');
  Serial.print(current);
  Serial.print(',');
  Serial.print(fitness, 4);
  Serial.print(',');
  Serial.print(globalBestFitness, 4);
  for (uint8_t d = 0; d < DIMS; d++) {
    Serial.print(',');
    Serial.print(globalBest[d], 3);
  }
  Serial.println();
}

static void finishEval(float fitness) {
  Particle &p = swarm[current];
  if (fitness < p.bestFitness) {
    p.bestFitness = fitness;
    memcpy(p.best, p.pos, sizeof(p.best));
  }
  if (fitness < globalBestFitness) {
    globalBestFitness = fitness;
    memcpy(globalBest, p.pos, sizeof(globalBest));
  }
  report(fitness);

  if (++current >= numParticles) {
    current = 0;
    if (++iteration >= cfg.maxIterations) {
      autotuneStop(true);
      return;
    }
    moveSwarm();
  }
}

void autotuneStart() {
  if (tuning) return;
  if (state != STATE_BALANCING) {
    Serial.println(F("E, AUTOTUNE needs the robot balancing"));
    return;
  }

  numParticles = constrain(cfg.numParticles, 2, MAX_PARTICLES);
  getGains(baseline);
  for (uint8_t d = 0; d < DIMS; d++) {
    float span = fabs(baseline[d]) * cfg.SPOspread;
    if (span < 0.05f) span = 0.05f;
    lo[d] = baseline[d] - span;
    hi[d] = baseline[d] + span;
    if (lo[d] < 0) lo[d] = 0;
  }

  for (uint8_t i = 0; i < numParticles; i++) {
    Particle &p = swarm[i];
    for (uint8_t d = 0; d < DIMS; d++) {
      // Particle 0 starts on the current gains, so the result is never worse.
      p.pos[d] = (i == 0) ? baseline[d] : lo[d] + rand01() * (hi[d] - lo[d]);
      p.vel[d] = (rand01() - 0.5f) * (hi[d] - lo[d]) * 0.2f;
      p.best[d] = p.pos[d];
    }
    p.bestFitness = FALL_PENALTY * 10;
  }
  memcpy(globalBest, baseline, sizeof(globalBest));
  globalBestFitness = FALL_PENALTY * 10;

  current = 0;
  iteration = 0;
  falls = 0;
  tuning = true;
  beginEval();
  setEvent("Autotune ON");
}

void autotuneStop(bool applyBest) {
  if (!tuning) return;
  tuning = false;
  if (applyBest && globalBestFitness < FALL_PENALTY)
    setGains(globalBest);
  else
    setGains(baseline);
  setEvent(applyBest ? "Autotune done" : "Autotune OFF");
  PIDParamTX();
}

void autotuneStep(float dt, float angleError, float motorOutput) {
  if (!tuning) return;
  if (waitingForBalance) {
    // Back on its wheels after a fall: carry on with the next particle.
    beginEval();
    return;
  }

  elapsedMs += dt * 1000.0f;
  if (elapsedMs < cfg.SPOSettle) return;

  cost += (angleError * angleError + 0.0005f * motorOutput * motorOutput) * dt;
  if (elapsedMs >= (float)cfg.SPOSettle + cfg.SPOConfigEval) {
    finishEval(cost / (cfg.SPOConfigEval / 1000.0f));
    if (tuning) beginEval();
  }
}

void autotuneOnFall() {
  if (!tuning) return;
  finishEval(FALL_PENALTY);
  if (!tuning) return;
  if (++falls >= MAX_FALLS) {
    setEvent("Autotune: too many falls");
    autotuneStop(true);
    return;
  }
  // Recover with the safest gains we know, test the next particle once balancing again.
  setGains(globalBestFitness < FALL_PENALTY ? globalBest : baseline);
  waitingForBalance = true;
}
