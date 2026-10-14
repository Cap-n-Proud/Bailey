// PC simulation of the Bailey firmware.
//
// Compiles the real firmware (.ino files, with Eigen and TinyMPC) against the
// stub Arduino API in stubs/ and runs it against a simple model of a
// two-wheeled inverted pendulum on steppers.
//
//   ./run_tests.sh            builds the simulator and runs every scenario
//   ./sim <scenario>          runs one scenario, prints a summary, exits 1 on failure
#define ARDUINO_ARCH_ESP32 1     // the firmware's only board
#define BAILEY_SIMULATOR 1       // mpc.ino: solve in line instead of on the second core
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "MPU6050.h"
#include "FastAccelStepper.h"
#include <algorithm>
#ifndef __wasm__
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#endif
#include <random>
#include <vector>

uint64_t simMicros = 0;
int simPinValue[64];
int simPwm[64];
int16_t simAccel[3], simGyro[3];
SimSerial Serial;
SimEEPROM EEPROM;
SimWire Wire;

#include "../bailey_FW.ino"
#include "../autotune.ino"
#include "../comms.ino"
#include "../gains.ino"
#include "../imu.ino"
#include "../lqr.ino"
#include "../motors.ino"
#include "../mpc.ino"
#include "../settings.ino"

// ---------------- Plant model ----------------
// A rigid body on two wheels; each stepper turns its wheel relative to the
// body (its housing is bolted to it), and the wheels roll without slipping.
// Equations of motion from Lagrange with the wheel angle relative to the body
// prescribed, checked against MuJoCo (test/mujoco_check.py: agrees within
// 0.02 deg). Linearised they read pitch'' = (g pitch - kappa a) / L, with
// a the wheels' acceleration relative to the body (what the firmware commands);
// L and kappa (1 for a cart-pole; ~1.4 for Bailey) are what the autotuner
// measures, see fallLength() and coupling().
struct Plant {
  // Geometry: 94 mm wheels, centre of mass 15 cm above the axle.
  double wheelRadius = 0.047;
  double bodyMass = 1.5;        // kg
  double wheelMass = 0.2;       // kg, both wheels (uniform discs)
  double comHeight = 0.15;      // m, centre of mass above the axle
  double bodyLength = 0.20;     // (I + m l^2) / (m l) of the body alone, about the axle
  double g = 9.81;
  double stepsPerRev = 1600;    // 200-step motor, 1/8 microstepping
  double mountOffsetDeg = 1.5;  // IMU is tilted this much when the robot is truly upright
  double noiseDeg = 0.05;       // scales accelerometer and gyro noise
  double imuHeight = 0.12;      // m above the axle: it also feels the body's rotation
  double gyroBiasDps = 0.3;     // residual bias after the boot calibration
  double gyroBiasDrift = 0;     // deg/s per s, e.g. the chip warming up
  double sensorLag = 0.0048;    // s, MPU-6050 low-pass filter delay at DLPF_BW_42
  double slip = 1.0;            // ground motion / wheel motion (< 1: wheels slipping)
  double lagOmega = 0, lagFy = 0, lagFz = 9.81;

  double theta = 0, omega = 0;  // rad, + = leaning forward
  double ul = 0, ur = 0;        // wheel surface speed relative to the body (what the steppers set), m/s
  double vl = 0, vr = 0;        // wheel ground speed m/s
  double x = 0;
  bool held = true;
  double alpha = 0;
  double pickupRate = 0;        // rad/s while someone is turning the robot by hand
  double disturbance = 0;       // rad/s^2 on the body: payload off-centre, someone leaning on it
  std::mt19937 rng{42};
  std::normal_distribution<double> noise{0, 1};

  // Lagrange terms (r: wheel radius, mb: body, mw: wheels, l: centre of mass):
  //   M = (mw + mb) r^2 + Iw,  J = mb l bodyLength (body inertia about the axle)
  //   pitch'' (J + M + 2 mb r l cos) = mb g l sin + mb r l sin pitch'^2 - (a / r) (M + mb r l cos)
  double wheelM() const { return (wheelMass + bodyMass) * wheelRadius * wheelRadius + 0.5 * wheelMass * wheelRadius * wheelRadius; }
  double bodyJ() const { return bodyMass * comHeight * bodyLength; }
  double fallRate() const {     // g / L of the linearised model, 1/s^2
    return bodyMass * g * comHeight / (bodyJ() + wheelM() + 2 * bodyMass * wheelRadius * comHeight);
  }
  double fallLength() const { return g / fallRate(); }   // L
  double coupling() const {     // kappa
    double p3 = (wheelM() + bodyMass * wheelRadius * comHeight) / wheelRadius /
                (bodyJ() + wheelM() + 2 * bodyMass * wheelRadius * comHeight);
    return p3 * fallLength();
  }
  // A robot with the given L, same proportions (centre of mass at 3/4 of
  // bodyLength). Big robots get a taller body on the same wheels; below
  // 0.25 m the whole robot, wheels included, is scaled down (94 mm wheels
  // can't make a shorter pendulum). Returns false if the wheel size changed.
  bool setLength(double L) {
    const double rDefault = 0.047, Lmin = 0.25;
    wheelRadius = rDefault;
    double scale = 1;
    if (L < Lmin) { scale = L / Lmin; L = Lmin; }
    double lo = 0.06, hi = 2;
    for (int i = 0; i < 60; i++) {
      bodyLength = (lo + hi) / 2;
      comHeight = 0.75 * bodyLength;
      (fallLength() < L ? lo : hi) = bodyLength;
    }
    wheelRadius *= scale;
    bodyLength *= scale;
    comHeight *= scale;
    return scale == 1;
  }

  void step(double dt) {
    // Steppers follow the library's ramped step rate exactly (no stalling modelled).
    double k = 2 * M_PI * wheelRadius / stepsPerRev;
    stepperLeft->advance(dt);
    stepperRight->advance(dt);
    double al = (stepperLeft->speed * k - ul) / dt, ar = (stepperRight->speed * k - ur) / dt;
    ul += al * dt;
    ur += ar * dt;
    double v0 = (vl + vr) / 2;
    if (held) {
      // Held in the air / on a stand: the body doesn't move, the wheels spin freely.
      omega = 0;
      alpha = 0;
      vl = ul;
      vr = ur;
    } else {
      // slip < 1: the wheels grip only partly (a crude model: their push on the
      // body and the ground motion are both scaled down).
      double a = slip * (al + ar) / 2, r = wheelRadius, mbrl = bodyMass * r * comHeight;
      double M = wheelM(), c = cos(theta), s = sin(theta);
      alpha = (bodyMass * g * comHeight * s + mbrl * s * omega * omega - a / r * (M + mbrl * c)) /
              (bodyJ() + M + 2 * mbrl * c) + disturbance;
      omega += alpha * dt;
      theta += omega * dt;
      // Lying on the floor.
      if (fabs(theta) > 1.3) { theta = theta > 0 ? 1.3 : -1.3; omega = 0; }
      // The wheels turn with the body too; slip < 1 loses some of that on the ground.
      vl = slip * (ul + r * omega);
      vr = slip * (ur + r * omega);
    }
    if (!held) x += (vl + vr) / 2 * dt;
    // MPU-6050 on the body, pitch axis = sensor X, Z up when upright. It measures
    // gravity plus its own acceleration (wheels + rotation of the body), which is
    // what makes a plain accelerometer angle useless while balancing.
    double a = held ? 0 : ((vl + vr) / 2 - v0) / dt + imuHeight * alpha;
    double t = theta + mountOffsetDeg * M_PI / 180;
    double fy = cos(t) * a + sin(t) * g;
    double fz = cos(t) * g - sin(t) * a;
    gyroBiasDps += gyroBiasDrift * dt;
    double lagK = std::min(1.0, dt / sensorLag);
    lagFy += (fy - lagFy) * lagK;
    lagFz += (fz - lagFz) * lagK;
    lagOmega += ((held ? pickupRate : omega) - lagOmega) * lagK;
    // +/-4 g and +/-500 deg/s ranges; the real sensor clips at full scale.
    const double accLsb = 8192 / g, gyroLsb = 65.5;
    auto raw = [](double v) { return (int16_t)std::max(-32768.0, std::min(32767.0, std::round(v))); };
    simAccel[0] = raw(noiseDeg * 0.02 * g * noise(rng) * accLsb);
    simAccel[1] = raw((lagFy + noiseDeg * 0.02 * g * noise(rng)) * accLsb);
    simAccel[2] = raw((lagFz + noiseDeg * 0.02 * g * noise(rng)) * accLsb);
    simGyro[0] = raw((lagOmega * 180 / M_PI + gyroBiasDps + noiseDeg * 4 * noise(rng)) * gyroLsb);
    simGyro[1] = raw(noiseDeg * 4 * noise(rng) * gyroLsb);
    simGyro[2] = raw(noiseDeg * 4 * noise(rng) * gyroLsb);
  }
};

Plant plant;

// Gives the simulated robot pendulum length L (see Plant::setLength). If that
// needs smaller wheels, tells the firmware their size, as a user would.
static void setRobotLength(double L) {
  if (!plant.setLength(L)) {
    cfg.wheelDiameter = plant.wheelRadius * 2000;
    applyGainsForLength(cfg.pendulumLength);
    lqrWeightsChanged = true;
  }
}
std::vector<std::string> received;

static void onLine(const std::string &l) {
  received.push_back(l);
  if (getenv("SIM_VERBOSE")) printf("  < %s\n", l.c_str());
}

// Like the web UI: a non-zero Throttle/Steer is repeated every 100 ms until it
// is set back to 0 (the firmware stops driving if the repeats stop).
std::string heldThrottle, heldSteer;
bool remoteConnected = true;

static void send(const std::string &cmd) {
  if (getenv("SIM_VERBOSE")) printf("  > %s\n", cmd.c_str());
  Serial.send(cmd + "\n");
  auto hold = [&](const char *prefix, std::string &held) {
    if (cmd.rfind(prefix, 0) == 0) held = atof(cmd.c_str() + strlen(prefix)) != 0 ? cmd : "";
  };
  hold("SCMD Throttle ", heldThrottle);
  hold("SCMD Steer ", heldSteer);
}

struct Stats {
  double maxAbsDeg = 0, sumSq = 0, maxSpeed = 0;
  long n = 0;
  int falls = 0;
  void reset() { *this = Stats(); }
};
Stats stats;

const double SIM_DT = 0.0002;

static void run(double seconds) {
  uint64_t end = simMicros + (uint64_t)(seconds * 1e6);
  RobotState prev = state;
  static uint64_t lastRepeat = 0;
  while (simMicros < end) {
    if (remoteConnected && simMicros - lastRepeat >= 100000) {
      lastRepeat = simMicros;
      if (!heldThrottle.empty()) Serial.send(heldThrottle + "\n");
      if (!heldSteer.empty()) Serial.send(heldSteer + "\n");
    }
    plant.step(SIM_DT);
    simMicros += (uint64_t)(SIM_DT * 1e6);
    loop();
    if (!plant.held && state == STATE_BALANCING) {
      double d = fabs(plant.theta * 180 / M_PI);
      stats.maxAbsDeg = std::max(stats.maxAbsDeg, d);
      stats.sumSq += d * d;
      stats.maxSpeed = std::max(stats.maxSpeed, fabs((plant.vl + plant.vr) / 2));
      stats.n++;
    }
    if (prev == STATE_BALANCING && state == STATE_FALLEN) stats.falls++;
    static FILE *csv = getenv("SIM_CSV") ? fopen(getenv("SIM_CSV"), "w") : nullptr;
    static uint64_t lastCsv = 0;
    if (csv && simMicros - lastCsv >= 10000) {
      if (lastCsv == 0) fprintf(csv, "t,theta,pitch,angleSetpoint,motor,speed,speedSetpoint,state\n");
      lastCsv = simMicros;
      fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.2f,%d\n", simMicros / 1e6, plant.theta * 180 / M_PI, pitch,
              angleSetpoint, angleOutput, (plant.vl + plant.vr) / 2, speedSetpoint, (int)state);
      fflush(csv);
    }
    static uint64_t lastTrace = 0;
    if (getenv("SIM_TRACE") && simMicros - lastTrace >= (uint64_t)(getenv("SIM_TRACE_US") ? atoi(getenv("SIM_TRACE_US")) : 50000)) {
      lastTrace = simMicros;
      printf("t=%.2f st=%d th=%.2f meas=%.2f set=%.2f out=%.1f v=%.3f ws=%.1f\n", simMicros / 1e6, state,
             plant.theta * 180 / M_PI, pitch, angleSetpoint, angleOutput, (plant.vl + plant.vr) / 2, wheelSpeed);
    }
    prev = state;
  }
}

static double rmsDeg() { return stats.n ? sqrt(stats.sumSq / stats.n) : 0; }

static bool check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
  return ok;
}

static void boot() {
  // Plant variations from the environment (used by the tuning scripts).
  if (getenv("SIM_OFFSET")) plant.mountOffsetDeg = atof(getenv("SIM_OFFSET"));
  if (getenv("SIM_NOISE")) plant.noiseDeg = atof(getenv("SIM_NOISE"));
  if (getenv("SIM_LAG")) plant.sensorLag = atof(getenv("SIM_LAG"));
  if (getenv("SIM_IMU_HEIGHT")) plant.imuHeight = atof(getenv("SIM_IMU_HEIGHT"));
  if (getenv("SIM_BIAS")) plant.gyroBiasDps = atof(getenv("SIM_BIAS"));
  if (getenv("SIM_DIST")) plant.disturbance = atof(getenv("SIM_DIST"));
  heldThrottle.clear();
  heldSteer.clear();
  remoteConnected = true;
  // Like a power cycle: runtime state back to power-on values (EEPROM persists).
  autotuneAbort("reset");
  motorsEnabled = false;
  state = STATE_DISARMED;
  throttleCmd = steerCmd = 0;
  speedSetpoint = 0;
  imuStarted = false;
  Serial.onLine = onLine;
  setup();
  if (getenv("SIM_CONTROLLER")) cfg.controller = atoi(getenv("SIM_CONTROLLER"));
  if (getenv("SIM_ACCELCOMP")) cfg.imuAccelCompensation = atoi(getenv("SIM_ACCELCOMP"));
  // LQR="maxPos maxSpeed maxAngle maxRate maxAccel dobBandwidth" overrides the LQR settings.
  if (getenv("LQR")) {
    sscanf(getenv("LQR"), "%f %f %f %f %f %f", &cfg.lqrMaxPosition, &cfg.lqrMaxSpeed, &cfg.lqrMaxAngle,
           &cfg.lqrMaxRate, &cfg.lqrMaxAccel, &cfg.dobBandwidth);
    lqrWeightsChanged = true;
  }
  if (getenv("KALMAN")) sscanf(getenv("KALMAN"), "%f %f %f", &cfg.kalmanQAngle, &cfg.kalmanQBias, &cfg.kalmanR);
  if (getenv("SIM_L")) setRobotLength(atof(getenv("SIM_L")));
  if (getenv("SIM_SCALE")) {          // as if the autotuner had measured L and kappa
    cfg.pendulumCoupling = plant.coupling();
    applyGainsForLength(plant.fallLength());
  }
  // GAINS=aKp,aKi,aKd,sKp,sKi,sLimit[,trigger,aggFactor] overrides the defaults (used by tuning scripts).
  if (getenv("GAINS")) {
    float g[8];
    int n = sscanf(getenv("GAINS"), "%f,%f,%f,%f,%f,%f,%f,%f", &g[0], &g[1], &g[2], &g[3], &g[4], &g[5], &g[6], &g[7]);
    if (n >= 6) {
      cfg.anglePIDConKp = g[0]; cfg.anglePIDConKi = g[1]; cfg.anglePIDConKd = g[2];
      cfg.speedPIDKp = g[3]; cfg.speedPIDKi = g[4]; cfg.speedPIDOutputLimit = g[5];
    }
    if (n >= 8) { cfg.TriggerAngleAggressive = g[6]; cfg.anglePIDAggKp = g[0] * g[7]; cfg.anglePIDAggKi = g[1] * g[7]; cfg.anglePIDAggKd = g[2] * g[7]; }
    else cfg.TriggerAngleAggressive = 0;
  }
  plant.held = true;
  plant.theta = 0;
}

double uprightReading = 0;

// Hold the robot upright, switch motors on, release it tilted.
static void startBalancing(double releaseDeg) {
  run(1.0);
  send("SCMD Motors 1");
  // Let go as soon as it starts balancing, like a person holding it would.
  while (state != STATE_BALANCING) run(0.001);
  uprightReading = pitchRaw;   // what the IMU filter reads with the robot truly upright
  plant.theta = releaseDeg * M_PI / 180;
  plant.held = false;
}

static bool scenarioBalance() {
  boot();
  startBalancing(3);
  stats.reset();
  run(20);
  bool ok = true;
  ok &= check(state == STATE_BALANCING, "still balancing after 20 s");
  ok &= check(stats.falls == 0, "no falls");
  ok &= check(stats.maxAbsDeg < 6, "max tilt < 6 deg");
  printf("  max tilt %.2f deg, rms %.2f deg, max speed %.2f m/s, drift %.2f m\n",
         stats.maxAbsDeg, rmsDeg(), stats.maxSpeed, plant.x);
  // The IMU is mounted 1.5 deg off and calibratedZeroAngle is 0: the speed
  // loop must find the real balance point instead of running away.
  stats.reset();
  double x0 = plant.x;
  run(10);
  ok &= check(fabs(plant.x - x0) < 0.3, "holds position (< 30 cm in 10 s) despite IMU offset");
  ok &= check(fabs((plant.vl + plant.vr) / 2) < 0.05, "wheel speed settles near 0");
  printf("  moved %.3f m in last 10 s, rms %.2f deg\n", plant.x - x0, rmsDeg());
  return ok;
}

static bool scenarioPush() {
  boot();
  startBalancing(0);
  run(5);
  stats.reset();
  plant.omega += 0.6;   // shove forward (rad/s)
  run(5);
  bool ok = check(stats.falls == 0 && state == STATE_BALANCING, "recovers from a forward push");
  printf("  max tilt %.2f deg\n", stats.maxAbsDeg);
  plant.omega -= 0.6;   // and backward
  stats.reset();
  run(5);
  ok &= check(stats.falls == 0 && state == STATE_BALANCING, "recovers from a backward push");
  printf("  max tilt %.2f deg\n", stats.maxAbsDeg);
  return ok;
}

static bool scenarioDrive() {
  boot();
  startBalancing(0);
  run(5);
  stats.reset();
  double x0 = plant.x;
  send("SCMD Throttle 100");
  run(4);
  // Average over the next 2 s so a slow oscillation around the target doesn't fool the check.
  double v = 0;
  for (int i = 0; i < 200; i++) {
    run(0.01);
    v += (plant.vl + plant.vr) / 2 / 200;
  }
  double expected = cfg.Maxthrottle / 100 * cfg.stepperMaxSpeed / plant.stepsPerRev * 2 * M_PI * plant.wheelRadius;
  bool ok = check(stats.falls == 0 && state == STATE_BALANCING, "no fall while driving forward");
  ok &= check(plant.x - x0 > 0.3, "moves forward");
  ok &= check(fabs(v - expected) < 0.15 * expected, "reaches the throttle speed (+/- 15 %)");
  printf("  speed %.3f m/s (expected %.3f), moved %.2f m\n", v, expected, plant.x - x0);
  send("SCMD Throttle -100");
  run(6);
  ok &= check((plant.vl + plant.vr) / 2 < 0, "drives backward");
  send("SCMD Throttle 0");
  run(6);
  ok &= check(stats.falls == 0 && state == STATE_BALANCING, "no fall when reversing and stopping");
  ok &= check(fabs((plant.vl + plant.vr) / 2) < 0.05, "stops when throttle is released");

  // The remote disappears mid-drive: the firmware must stop driving by itself.
  send("SCMD Throttle 100");
  run(2);
  remoteConnected = false;
  run(1.2);
  ok &= check(throttleCmd == 0, "drive watchdog zeroes throttle when the remote goes quiet");
  run(5);
  ok &= check(fabs((plant.vl + plant.vr) / 2) < 0.05 && state == STATE_BALANCING, "...and the robot stops and keeps balancing");
  printf("  max tilt %.2f deg during the whole drive\n", stats.maxAbsDeg);
  return ok;
}

static bool scenarioFall() {
  boot();
  startBalancing(0);
  run(3);
  plant.omega += 6;   // kick it over
  run(2);
  bool ok = check(state == STATE_FALLEN, "detects the fall");
  ok &= check(angleOutput == 0, "motor command is zero after falling");
  ok &= check(!stepperLeft->enabled, "stepper drivers disabled after falling");
  // Pick it up again: rotate it back upright over half a second so the gyro sees it.
  plant.held = true;
  plant.vl = plant.vr = plant.ul = plant.ur = 0;
  double from = plant.theta;
  for (int i = 1; i <= 50; i++) {
    plant.theta = from * (1 - i / 50.0);
    plant.pickupRate = -from / 0.5;
    run(0.01);
  }
  plant.pickupRate = 0;
  for (int i = 0; i < 2000 && state != STATE_BALANCING; i++) run(0.001);
  ok &= check(state == STATE_BALANCING, "re-arms when held upright");
  plant.held = false;
  stats.reset();
  run(5);
  ok &= check(stats.falls == 0 && state == STATE_BALANCING, "balances again after being picked up");
  send("SCMD Motors 0");
  run(0.1);
  ok &= check(state == STATE_DISARMED, "Motors 0 disarms");
  return ok;
}

struct AtResult {
  bool seen = false, accepted = false;
  double L = 0, kappa = 0, offset = 0, r2 = 0, before = 0, after = 0;
};

static AtResult lastAutotuneResult() {
  AtResult r;
  for (auto &l : received) {
    if (l.rfind("AT,RESULT,", 0) == 0) {
      int acc;
      r.seen = sscanf(l.c_str(), "AT,RESULT,%d,%lf,%lf,%lf,%lf,%lf,%lf", &acc, &r.L, &r.kappa, &r.offset,
                      &r.r2, &r.before, &r.after) == 7;
      r.accepted = acc == 1;
    }
  }
  return r;
}

// One autotune run on a robot with the given true pendulum length and IMU offset.
static bool autotuneCase(double length, double offsetDeg, double tolerance = 0.15) {
  plant = Plant();
  received.clear();
  boot();
  setRobotLength(length);
  plant.mountOffsetDeg = offsetDeg;
  startBalancing(0);
  run(4);
  send("SCMD AUTOTUNE 1");
  stats.reset();
  run(35);
  AtResult r = lastAutotuneResult();
  printf("  L %.2f m, kappa %.2f, IMU reads %+.2f deg upright -> measured L %.3f, kappa %.2f, offset %+.2f, R2 %.2f, "
         "score %.4f -> %.4f\n", length, plant.coupling(), uprightReading, r.L, r.kappa, r.offset, r.r2, r.before, r.after);
  bool ok = check(r.seen && r.accepted && !autotuneRunning(), "result accepted");
  char what[48];
  snprintf(what, sizeof what, "length within %.0f %%", tolerance * 100);
  ok &= check(fabs(r.L / length - 1) < tolerance, what);
  ok &= check(fabs(r.kappa / plant.coupling() - 1) < 0.15, "coupling within 15 %");
  ok &= check(fabs(r.offset - uprightReading) < 0.3, "zero within 0.3 deg of the upright reading");
  ok &= check(fabs(cfg.pendulumLength - r.L) < 1e-3 && fabs(cfg.pendulumCoupling - r.kappa) < 1e-3 &&
              fabs(cfg.calibratedZeroAngle - r.offset) < 0.01,
              "gains and zero applied");
  ok &= check(stats.falls == 0 && state == STATE_BALANCING, "balanced throughout");
  return ok;
}


static bool scenarioAutotune() {
  if (getenv("AT_L")) return autotuneCase(atof(getenv("AT_L")), getenv("AT_OFFSET") ? atof(getenv("AT_OFFSET")) : 1.5);
  bool ok = true;
  ok &= autotuneCase(0.31, 1.5);    // Bailey
  ok &= autotuneCase(0.25, -1.0);   // the shortest robot on Bailey's 94 mm wheels
  ok &= autotuneCase(0.45, 2.5);    // taller
  // A smaller robot on smaller wheels: its faster dynamics are measured less
  // well (L 20-25 % high), but the result still balances better.
  ok &= autotuneCase(0.15, 0.5, 0.35);

  // Wheels slipping: the wheels tip the body far less than they should, so the
  // measured coupling is implausible; nothing may change.
  plant = Plant();
  plant.slip = 0.5;
  received.clear();
  boot();
  startBalancing(0);
  run(4);
  float kp = cfg.anglePIDConKp, zero = cfg.calibratedZeroAngle;
  send("SCMD AUTOTUNE 1");
  run(35);
  AtResult r = lastAutotuneResult();
  printf("  slipping wheels -> L %.3f, kappa %.2f\n", r.L, r.kappa);
  ok &= check(r.seen && !r.accepted, "slipping wheels: result rejected");
  ok &= check(cfg.anglePIDConKp == kp && cfg.calibratedZeroAngle == zero, "slipping wheels: settings restored");
  ok &= check(state == STATE_BALANCING, "slipping wheels: still balancing");

  // Touching the throttle aborts and restores.
  plant = Plant();
  received.clear();
  boot();
  startBalancing(0);
  run(4);
  kp = cfg.anglePIDConKp;
  send("SCMD AUTOTUNE 1");
  run(10);
  send("SCMD Throttle 50");
  run(0.1);
  bool aborted = false;
  for (auto &l : received) if (l.rfind("E, autotune aborted: driving", 0) == 0) aborted = true;
  ok &= check(aborted && !autotuneRunning() && cfg.anglePIDConKp == kp, "throttle aborts and restores");
  return ok;
}

// Gains scaled to the true pendulum length (what the autotuner ends up with)
// against the fixed defaults, on robots much shorter and taller than assumed.
static bool scenarioScaling() {
  bool ok = true;
  std::vector<double> lengths = { 0.07, 0.12, 0.2, 0.32, 0.45 };
  if (getenv("SCALING_L")) lengths = { atof(getenv("SCALING_L")) };
  for (double L : lengths) {
    double rmsFixed = 0, rmsScaled = 0;
    int fallsFixed = 0, fallsScaled = 0;
    for (int scaled = 0; scaled < 2; scaled++) {
      plant = Plant();
      received.clear();
      boot();
      setRobotLength(L);
      if (scaled) {
        cfg.pendulumCoupling = plant.coupling();
        applyGainsForLength(L);
      }
      startBalancing(2);
      stats.reset();
      run(6);
      plant.omega += 0.4;
      run(4);
      send("SCMD Throttle 100");
      run(4);
      send("SCMD Throttle 0");
      run(5);
      int falls = stats.falls + (state != STATE_BALANCING);
      (scaled ? rmsScaled : rmsFixed) = rmsDeg();
      (scaled ? fallsScaled : fallsFixed) = falls;
    }
    printf("  L %.2f m: default gains %s rms %.2f deg | scaled gains %s rms %.2f deg\n", L,
           fallsFixed ? "FELL," : "ok,  ", rmsFixed, fallsScaled ? "FELL," : "ok,  ", rmsScaled);
    char what[64];
    snprintf(what, sizeof what, "scaled gains balance at L = %.2f m", L);
    ok &= check(fallsScaled == 0, what);
  }
  return ok;
}

// Used by tune_gains.py: plant and gains from the environment, prints one score line.
static bool scenarioScore() {
  boot();
  startBalancing(3);
  stats.reset();
  run(8);
  plant.omega += 0.5;
  run(4);
  plant.omega -= 0.5;
  run(4);
  send("SCMD Throttle 100");
  run(3);
  // Speed tracking: mean |speed error| over the next 3 s, as a fraction of the target.
  double target = cfg.Maxthrottle / 100 * cfg.speedFeedForward;  // fraction of full speed
  double full = cfg.stepperMaxSpeed / plant.stepsPerRev * 2 * M_PI * plant.wheelRadius;
  double trackErr = 0;
  for (int i = 0; i < 300; i++) {
    run(0.01);
    trackErr += fabs((plant.vl + plant.vr) / 2 / full - target) / target / 300;
  }
  send("SCMD Throttle -100");
  run(5);
  send("SCMD Throttle 0");
  double x0 = plant.x;
  run(8);
  printf("SCORE %d %.3f %.3f %.3f %.3f %.3f\n", stats.falls + (state != STATE_BALANCING), rmsDeg(), stats.maxAbsDeg,
         fabs(plant.x - x0), fabs((plant.vl + plant.vr) / 2), trackErr);
  return true;
}

// A single run showing everything, for plotting: SIM_CSV=out.csv ./sim_stepper demo
static bool scenarioDemo() {
  boot();
  startBalancing(3);     // let go 3 deg off
  run(5);
  plant.omega += 0.5;    // shove forward
  run(4);
  send("SCMD Throttle 100");
  run(6);
  send("SCMD Throttle -100");
  run(6);
  send("SCMD Throttle 0");
  run(6);
  return state == STATE_BALANCING;
}

// Virtual robot for the web server (server/ with "serial.port": "sim"): runs in
// real time, reads commands on stdin and writes the robot's serial output on
// stdout. A simulated person holds the robot upright until it starts
// balancing, and picks it back up 2 s after it falls.
#ifndef __wasm__
static bool scenarioLive() {
  remoteConnected = false;        // the server repeats drive commands itself
  received.clear();
  Serial.onLine = [](const std::string &l) {
    fwrite(l.data(), 1, l.size(), stdout);
    fputc('\n', stdout);
    fflush(stdout);
  };
  setup();
  plant.held = true;
  std::string pending;
  double fallenFor = 0;
  auto next = std::chrono::steady_clock::now();
  for (;;) {
    struct pollfd p = { 0, POLLIN, 0 };
    while (poll(&p, 1, 0) > 0) {
      char buf[256];
      ssize_t n = read(0, buf, sizeof buf);
      if (n <= 0) return true;    // server went away
      Serial.send(std::string(buf, n));
    }
    run(0.01);
    if (plant.held && state == STATE_BALANCING) plant.held = false;
    if (state == STATE_FALLEN) {
      fallenFor += 0.01;
      if (fallenFor > 2) {       // pick it up again
        plant.held = true;
        plant.vl = plant.vr = plant.ul = plant.ur = 0;
        double from = plant.theta;
        for (int i = 1; i <= 50; i++) {
          plant.theta = from * (1 - i / 50.0);
          plant.pickupRate = -from / 0.5;
          run(0.01);
        }
        plant.pickupRate = 0;
        fallenFor = 0;
      }
    } else {
      fallenFor = 0;
    }
    if (state == STATE_DISARMED) plant.held = true;   // someone holds it while the motors are off
    next += std::chrono::milliseconds(10);
    std::this_thread::sleep_until(next);
  }
}
#endif

// Prints the LQR gain solved by the firmware's own Riccati solver (checked
// against SciPy in tune tooling): ./sim_stepper lqrgain <L> <qp> <qv> <qa> <qr> <r>
static bool scenarioLqrGain() {
  float L = 0.2, q[4] = { 1 / 0.09f, 1 / 0.09f, 1 / sq(2 / 57.29578f), 1 / sq(30 / 57.29578f) }, r = 1 / 9.0f;
  if (getenv("LQR_ARGS")) sscanf(getenv("LQR_ARGS"), "%f %f %f %f %f %f", &L, &q[0], &q[1], &q[2], &q[3], &r);
  float K[4];
  bool ok = lqrSolve(L, 1, 0.005f, q, r, K);
  printf("K %.6f %.6f %.6f %.6f\n", K[0], K[1], K[2], K[3]);
  return ok;
}

// Disturbance observer: a payload shifts the centre of mass mid-run. With the
// observer the robot finds its new balance angle and holds position.
static bool scenarioDisturbance() {
  bool ok = true;
  const double offsetDeg = 2.5;
  const double dist = -Plant().fallRate() * sin(offsetDeg * M_PI / 180);   // same as the mass moving 2.5 deg back
  double moved[2] = { 0, 0 };
  for (int withObserver = 0; withObserver < 2; withObserver++) {
    plant = Plant();
    received.clear();
    boot();
    if (!withObserver) cfg.dobBandwidth = 0;
    startBalancing(0);
    run(6);
    double x0 = plant.x;
    stats.reset();
    plant.disturbance = dist;
    run(12);
    moved[withObserver] = plant.x - x0;
    printf("  observer %s: moved %+.3f m, balance angle %.2f deg, true tilt %.2f deg, falls %d\n",
           withObserver ? "on " : "off", moved[withObserver], balanceAngle, plant.theta * 180 / M_PI, stats.falls);
    ok &= check(stats.falls == 0 && state == STATE_BALANCING, withObserver ? "observer on: still balancing" : "observer off: still balancing");
    if (withObserver) {
      // The payload pulls back, so the robot must lean forward by offsetDeg; the IMU
      // reads that plus its own mounting offset.
      double expected = uprightReading + offsetDeg;
      ok &= check(fabs(balanceAngle + cfg.calibratedZeroAngle - expected) < 0.3, "observer finds the new balance angle (0.3 deg)");
    }
  }
  ok &= check(fabs(moved[1]) < 0.05, "observer on: holds position (< 5 cm)");
  ok &= check(fabs(moved[1]) < fabs(moved[0]) / 3, "observer cuts the drift by 3x or more");
  return ok;
}

// Kalman filter: the gyro bias drifts as the chip warms up; pitch must stay accurate.
static bool scenarioKalman() {
  plant = Plant();
  received.clear();
  boot();
  startBalancing(0);
  run(3);
  plant.gyroBiasDrift = 0.1;        // +2 deg/s over 20 s
  double worst = 0;
  for (int i = 0; i < 2000; i++) {
    run(0.01);
    if (i > 300) worst = std::max(worst, fabs(pitchRaw - (plant.theta * 180 / M_PI + plant.mountOffsetDeg)));
  }
  printf("  bias now %.2f deg/s, estimated %.2f; worst pitch error %.2f deg\n", plant.gyroBiasDps, imuGyroBias, worst);
  bool ok = check(worst < 0.4, "pitch stays within 0.4 deg while the gyro bias drifts");
  ok &= check(fabs(imuGyroBias - plant.gyroBiasDps) < 0.3, "gyro bias tracked within 0.3 deg/s");
  ok &= check(stats.falls == 0 && state == STATE_BALANCING, "still balancing");

  // Accelerometer compensation (imuAccelCompensation): pitch error while
  // driving back and forth, without and with.
  double err[2];
  for (int comp = 0; comp < 2; comp++) {
    plant = Plant();
    received.clear();
    boot();
    cfg.imuAccelCompensation = comp;
    startBalancing(0);
    run(3);
    double s = 0;
    long n = 0;
    for (int k = 0; k < 2; k++)
      for (const char *t : { "SCMD Throttle 100", "SCMD Throttle -100" }) {
        send(t);
        for (int i = 0; i < 400; i++) {
          run(0.01);
          double e = pitchRaw - (plant.theta * 180 / M_PI + plant.mountOffsetDeg);
          s += e * e;
          n++;
        }
      }
    send("SCMD Throttle 0");
    run(5);
    err[comp] = sqrt(s / n);
    ok &= check(stats.falls == 0 && state == STATE_BALANCING, comp ? "with compensation: drives without falling" : "without compensation: drives without falling");
  }
  printf("  pitch error while driving: %.2f deg RMS without accelerometer compensation, %.2f with\n", err[0], err[1]);
  ok &= check(err[1] < err[0] / 2, "accelerometer compensation at least halves the pitch error while driving");
  return ok;
}

// The firmware's Riccati solver against SciPy's solve_discrete_are (same model, 5 ms).
static bool scenarioLqrSolver() {
  struct { float L, q[4], r, K[4]; } cases[] = {
    { 0.2f, { 1 / 0.09f, 1 / 0.09f, 1 / sq(2 / 57.29578f), 1 / sq(30 / 57.29578f) }, 1 / 9.0f, { -8.926f, -15.9437f, -105.7175f, -11.5772f } },
    { 0.1f, { 100, 1, 500, 1 }, 1, { -9.2558f, -7.7844f, -41.4681f, -3.7442f } },
    { 0.4f, { 1, 10, 1000, 10 }, 0.1f, { -2.8818f, -12.205f, -121.9978f, -18.7427f } },
  };
  bool ok = true;
  for (auto &c : cases) {
    float K[4];
    bool solved = lqrSolve(c.L, 1, 0.005f, c.q, c.r, K);
    double err = 0;
    for (int i = 0; i < 4; i++) err = std::max(err, (double)fabs((K[i] - c.K[i]) / c.K[i]));
    char what[80];
    snprintf(what, sizeof what, "L = %.1f: gain matches SciPy (rel. error %.1e)", c.L, err);
    ok &= check(solved && err < 1e-3, what);
  }
  return ok;
}

// LQR vs MPC near the limits (ESP32-S3 build): the hardest push each survives,
// and full throttle with Maxthrottle 95 (the firmware caps the target at 90 %
// of top speed) with full reversals.
static bool scenarioLimits() {
  if (getenv("LIMITS_TRACE")) {         // one fast drive with one controller, traced
    plant = Plant();
    boot();
    cfg.controller = atoi(getenv("LIMITS_TRACE"));
    cfg.Maxthrottle = getenv("MAXTHROTTLE") ? atof(getenv("MAXTHROTTLE")) : 95;
    startBalancing(0);
    run(3);
    send("SCMD Throttle 100");
    for (int k = 0; k < 800 && state == STATE_BALANCING; k++) {
      run(0.01);
      if (k % 10 == 0)
        printf("t=%.1f set%%=%.1f v=%.3f vcmd=%.3f th=%.2f plan c0=%.3f c10=%.3f iter=%u\n", simMicros / 1e6, speedSetpoint,
               (plant.vl + plant.vr) / 2, speedCmd, plant.theta * 180 / M_PI, mpcCurrent.c[0], mpcCurrent.c[10],
               mpcCurrent.iterations);
    }
    return true;
  }
  const char *names[] = { "LQR", "MPC" };
  double hardest[2] = { 0, 0 };
  int driveFalls[2] = { 0, 0 };
  double driveTilt[2] = { 0, 0 };
  uint16_t iterations = 0, worstIterations = 0;
  for (int c = 0; c < 2; c++) {
    for (double push = 0.5; push <= 6.01; push += 0.25) {
      plant = Plant();
      received.clear();
      boot();
      cfg.controller = c ? CONTROLLER_MPC : CONTROLLER_LQR;
      startBalancing(0);
      run(3);
      stats.reset();
      plant.omega += push;
      run(4);
      if (stats.falls || state != STATE_BALANCING) break;
      hardest[c] = push;
    }
    plant = Plant();
    received.clear();
    boot();
    cfg.controller = c ? CONTROLLER_MPC : CONTROLLER_LQR;
    cfg.Maxthrottle = getenv("MAXTHROTTLE") ? atof(getenv("MAXTHROTTLE")) : 95;
    startBalancing(0);
    run(3);
    stats.reset();
    for (int i = 0; i < 3; i++) {
      send("SCMD Throttle 100");
      for (int k = 0; k < 600; k++) {
        run(0.01);
        if (c) { uint16_t us; mpcStats(iterations, us); worstIterations = std::max(worstIterations, iterations); }
      }
      send("SCMD Throttle -100");
      run(6);
    }
    send("SCMD Throttle 0");
    run(5);
    driveFalls[c] = stats.falls + (state != STATE_BALANCING);
    driveTilt[c] = stats.maxAbsDeg;
    printf("  %s: hardest push survived %.2f rad/s; fast driving: %d falls, max tilt %.1f deg, top speed %.2f m/s\n",
           names[c], hardest[c], driveFalls[c], driveTilt[c], stats.maxSpeed);
  }
  printf("  MPC (TinyMPC) iterations per plan: worst %u of %d\n", worstIterations, MPC_MAX_ITER);
  bool ok = check(hardest[1] >= hardest[0], "MPC survives pushes at least as hard as LQR");
  ok &= check(driveFalls[0] == 0 && driveFalls[1] == 0,
              "LQR and MPC: no falls at full throttle (Maxthrottle 95, capped at 90 % of top speed) with full reversals");
  return ok;
}

static bool scenarioCommands() {
  boot();
  bool ok = true;
  float kd = cfg.anglePIDConKd;
  send("SCMD TriggerAngleAggressive 350");
  run(0.01);
  ok &= check(fabs(cfg.TriggerAngleAggressive - 3.5f) < 1e-4, "TriggerAngleAggressive 350 -> 3.5 deg");
  ok &= check(cfg.anglePIDConKd == kd, "TriggerAngleAggressive does not touch anglePIDConKd");
  send("SCMD speedPIDKp 3854");
  send("SCMD anglePIDConKp 880");
  send("SCMD calibratedZeroAngle -150");
  run(0.01);
  ok &= check(fabs(cfg.speedPIDKp - 0.3854f) < 1e-5, "speedPIDKp scaled by 1/10000");
  ok &= check(fabs(cfg.anglePIDConKp - 8.8f) < 1e-5, "anglePIDConKp scaled by 1/100");
  ok &= check(fabs(cfg.calibratedZeroAngle + 1.5f) < 1e-5, "calibratedZeroAngle scaled by 1/100");
  bool sawPid = false, sawEcho = false;
  for (auto &l : received) {
    if (l.rfind("PID,3854,", 0) == 0) sawPid = true;
    if (l == "SCMD speedPIDKp 3854") sawEcho = true;
  }
  ok &= check(sawPid, "PID line sent after a gain change");
  ok &= check(sawEcho, "SCMD echoed back");

  received.clear();
  send("READ RemoteInit");
  run(0.01);
  ok &= check(received.size() == 3 && received[0].rfind("TH,", 0) == 0, "RemoteInit sends TH/PIDH/SYSH");
  // Telemetry has as many fields as its header.
  received.clear();
  run(0.6);
  auto count = [](const std::string &s) { return std::count(s.begin(), s.end(), ','); };
  bool telemetryOk = false;
  for (auto &l : received)
    if (l.rfind("T,", 0) == 0)
      telemetryOk = count(l) == count("TH,yaw,pitch,roll,pitchSpeed,dISTE,leftMotorSpeed,rightMotorSpeed,LoopT,bal,balAngle") &&
                    l.size() < 80;
  ok &= check(telemetryOk, "telemetry line matches header and stays short");

  send("SCMD Throttle 50");
  run(0.01);
  ok &= check(throttleCmd == 50, "Throttle stored without double scaling");
  send("SCMD Throttle 500");
  run(0.01);
  ok &= check(throttleCmd == 100, "Throttle clamped to 100");

  received.clear();
  send("SCMD bogus 1");
  run(0.01);
  ok &= check(std::any_of(received.begin(), received.end(), [](const std::string &l) { return l.rfind("E, Unknown command bogus", 0) == 0; }),
              "unknown command reported");

  // A line longer than the buffer must not overflow it.
  send(std::string(200, 'X'));
  run(0.01);
  ok &= check(true, "long line ignored without crashing");
  return ok;
}

static bool scenarioEeprom() {
  boot();
  bool ok = check(cfg.magic == SETTINGS_MAGIC, "blank EEPROM -> defaults");
  ok &= check(EEPROM.begun > 0 && EEPROM.commits > 0, "ESP32: flash EEPROM begun and committed");
  ok &= check(Wire.sda == PIN_I2C_SDA && Wire.scl == PIN_I2C_SCL, "ESP32: I2C on the configured pins");
  cfg.anglePIDConKp = 12.34f;
  saveSettings();
  cfg.anglePIDConKp = 0;
  ok &= check(loadSettings() && fabs(cfg.anglePIDConKp - 12.34f) < 1e-6, "save/load round trip");
  EEPROM.mem[SETTINGS_ADDRESS + 10] ^= 0x55;   // corrupt one byte
  ok &= check(!loadSettings(), "corrupted settings are rejected");
  eraseSettings();
  ok &= check(!loadSettings(), "erased settings are rejected");
  return ok;
}

static bool scenarioMotorTest() {
  boot();
  run(1);
  received.clear();
  send("SCMD MotorTest 40");
  run(1.5);
  bool ok = false;
  for (auto &l : received) {
    if (l.rfind("TEST,", 0) == 0) {
      double a, b;
      sscanf(l.c_str(), "TEST,%lf,%lf", &a, &b);
      printf("  %s\n", l.c_str());
      ok = a > 10 && b > 10;
    }
  }
  return check(ok, "MotorTest spins both wheels forward and reports positive speeds");
}

#ifndef SIM_NO_MAIN
int main(int argc, char **argv) {
  struct { const char *name; bool (*fn)(); } scenarios[] = {
    { "balance", scenarioBalance },   { "push", scenarioPush },
    { "drive", scenarioDrive },       { "fall", scenarioFall },
    { "autotune", scenarioAutotune }, { "commands", scenarioCommands },
    { "scaling", scenarioScaling },   { "disturbance", scenarioDisturbance },
    { "kalman", scenarioKalman },     { "lqrsolver", scenarioLqrSolver },
    { "limits", scenarioLimits },
    { "eeprom", scenarioEeprom },     { "score", scenarioScore },
    { "demo", scenarioDemo },         { "lqrgain", scenarioLqrGain },         { "live", scenarioLive },     { "motortest", scenarioMotorTest },
  };
  if (argc < 2) {
    printf("usage: %s <scenario>\n", argv[0]);
    for (auto &s : scenarios) printf("  %s\n", s.name);
    return 2;
  }
  for (auto &s : scenarios) {
    if (strcmp(argv[1], s.name) == 0) {
      if (strcmp(s.name, "live") != 0) printf("%s:\n", s.name);
      return s.fn() ? 0 : 1;
    }
  }
  printf("unknown scenario %s\n", argv[1]);
  return 2;
}
#endif
