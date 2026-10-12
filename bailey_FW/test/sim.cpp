// PC simulation of the Bailey firmware.
//
// Compiles the real firmware (.ino files) against the stub Arduino API in
// stubs/ and runs it against a simple model of a two-wheeled inverted pendulum:
// DC gear motors with encoders (default) or steppers (-DMOTOR_TYPE=2).
//
//   ./run_tests.sh            builds both variants and runs every scenario
//   ./sim_dc <scenario>       runs one scenario, prints a summary, exits 1 on failure
#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"
#include "Encoder.h"
#include "FreeIMU.h"
#include <algorithm>
#include <random>
#include <vector>

uint64_t simMicros = 0;
int simPinValue[64];
int simPwm[64];
long simEncoder[64];
float simYpr[3];
SimSerial Serial;
SimEEPROM EEPROM;
SimWire Wire;

#include "../bailey_FW.ino"
#include "../autotune.ino"
#include "../comms.ino"
#include "../imu.ino"
#include "../motors.ino"
#include "../settings.ino"

// ---------------- Plant model ----------------
struct Plant {
  // Geometry: 94 mm wheels, centre of mass ~15 cm above the axle.
  double wheelRadius = 0.047;
  double effLength = 0.20;      // (I + m l^2) / (m l) of the body
  double g = 9.81;
  // Pololu 100:1, ~100 rpm at 12 V
  double vMax = 100.0 / 60.0 * 2 * M_PI * 0.047;
  double motorTau = 0.08;       // s, speed response of the gear motor
  int stictionPwm = 50;         // PWM below which the motor does not move
  double countsPerRev = 6400;
  double stepsPerRev = 1600;
  double mountOffsetDeg = 1.5;  // IMU reads this when the robot is truly upright
  double noiseDeg = 0.05;

  double theta = 0, omega = 0;  // rad, + = leaning forward
  double vl = 0, vr = 0;        // wheel ground speed m/s
  double x = 0;
  double encL = 0, encR = 0;
  bool held = true;
  double imuLag = 0;
  std::mt19937 rng{42};
  std::normal_distribution<double> noise{0, 1};

  double driveAccel(double v, int pwmPin, int in1, int in2) {
#if MOTOR_TYPE == MOTOR_DC
    int pwm = simPwm[pwmPin];
    int dir = (simPinValue[in1] && !simPinValue[in2]) ? 1 : (simPinValue[in2] && !simPinValue[in1]) ? -1 : 0;
    if (pwm == 0 || dir == 0) return -v / 0.5;                  // coasting
    double u = dir * std::max(0, pwm - stictionPwm) / double(255 - stictionPwm);
    return (u * vMax - v) / motorTau;
#else
    (void)v; (void)pwmPin; (void)in1; (void)in2;
    return 0;
#endif
  }

  void step(double dt) {
    double al, ar;
#if MOTOR_TYPE == MOTOR_DC
    al = driveAccel(vl, PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2);
    ar = driveAccel(vr, PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2);
#else
    // Steppers follow the commanded rate exactly (no stalling modelled).
    double k = 2 * M_PI * wheelRadius / stepsPerRev;
    double tl = (simPinValue[PIN_STEPPER_EN] ? 0 : stepSpeedLeft) * k;
    double tr = (simPinValue[PIN_STEPPER_EN] ? 0 : stepSpeedRight) * k;
    al = (tl - vl) / dt;
    ar = (tr - vr) / dt;
#endif
    if (held) {
      // Held in the air / on a stand: the body doesn't move, the wheels spin freely.
      omega = 0;
      vl += al * dt;
      vr += ar * dt;
    } else {
      double a = (al + ar) / 2;
      double alpha = (g * sin(theta) - a * cos(theta)) / effLength;
      omega += alpha * dt;
      theta += omega * dt;
      vl += al * dt;
      vr += ar * dt;
      // Lying on the floor.
      if (fabs(theta) > 1.3) { theta = theta > 0 ? 1.3 : -1.3; omega = 0; }
    }
    if (!held) x += (vl + vr) / 2 * dt;
    double perMeter = countsPerRev / (2 * M_PI * wheelRadius);
    encL += vl * dt * perMeter;
    encR += vr * dt * perMeter;
#if MOTOR_TYPE == MOTOR_DC
    simEncoder[PIN_ENC_LEFT_A] = lround(encL);
    simEncoder[PIN_ENC_RIGHT_A] = lround(encR);
#endif
    double measured = theta * 180 / M_PI + mountOffsetDeg + noiseDeg * noise(rng);
    imuLag += (measured - imuLag) * std::min(1.0, dt / 0.005);
    simYpr[0] = 0;
    simYpr[1] = imuLag;
    simYpr[2] = 0;
  }
};

Plant plant;
std::vector<std::string> received;

static void onLine(const std::string &l) {
  received.push_back(l);
  if (getenv("SIM_VERBOSE")) printf("  < %s\n", l.c_str());
}

static void send(const std::string &cmd) {
  if (getenv("SIM_VERBOSE")) printf("  > %s\n", cmd.c_str());
  Serial.send(cmd + "\n");
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
  while (simMicros < end) {
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
    static uint64_t lastTrace = 0;
    if (getenv("SIM_TRACE") && simMicros - lastTrace >= (getenv("SIM_TRACE_US") ? atoi(getenv("SIM_TRACE_US")) : 50000)) {
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
  Serial.onLine = onLine;
  setup();
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

// Hold the robot upright, switch motors on, release it tilted.
static void startBalancing(double releaseDeg) {
  run(1.0);
  send("SCMD Motors 1");
  // Let go as soon as it starts balancing, like a person holding it would.
  while (state != STATE_BALANCING) run(0.001);
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
  double expected = cfg.Maxthrottle / 100 * plant.vMax;
#if MOTOR_TYPE == MOTOR_STEPPER
  expected = cfg.Maxthrottle / 100 * cfg.stepperMaxSpeed / plant.stepsPerRev * 2 * M_PI * plant.wheelRadius;
#endif
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
#if MOTOR_TYPE == MOTOR_DC
  ok &= check(simPwm[PIN_LEFT_PWM] == 0 && simPwm[PIN_RIGHT_PWM] == 0, "PWM is zero after falling");
#else
  ok &= check(simPinValue[PIN_STEPPER_EN] == HIGH, "stepper drivers disabled after falling");
#endif
  // Pick it up again.
  plant.held = true;
  plant.theta = 0;
  plant.vl = plant.vr = 0;
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

static bool scenarioAutotune() {
  boot();
  // Start from deliberately poor (but stable) gains so there is something to find.
  cfg.anglePIDConKp *= 0.7;
  cfg.anglePIDConKd *= 1.4;
  startBalancing(0);
  run(3);
  float before[3] = { cfg.anglePIDConKp, cfg.anglePIDConKi, cfg.anglePIDConKd };
  send("SCMD numParticles 6");
  send("SCMD maxIterations 6");
  send("SCMD SPOConfigEval 2000");
  send("SCMD AUTOTUNE 1");
  stats.reset();
  run(6 * 6 * 2.6 + 10);
  bool ok = check(!tuning, "autotune finished by itself");
  ok &= check(state == STATE_BALANCING, "still balancing at the end");
  int spoLines = 0;
  double firstFit = -1, lastBest = -1;
  for (auto &l : received) {
    if (l.rfind("SPO,", 0) == 0) {
      spoLines++;
      double it, p, fit, best;
      sscanf(l.c_str(), "SPO,%lf,%lf,%lf,%lf", &it, &p, &fit, &best);
      if (firstFit < 0) firstFit = fit;
      lastBest = best;
    }
  }
  ok &= check(spoLines == 36, "one SPO report per particle evaluation (36)");
  ok &= check(lastBest <= firstFit, "best fitness is no worse than the starting gains");
  printf("  falls %d, start gains fitness %.4f -> best %.4f\n", stats.falls, firstFit, lastBest);
  printf("  gains Kp %.2f Ki %.2f Kd %.3f -> Kp %.2f Ki %.2f Kd %.3f\n", before[0], before[1], before[2],
         cfg.anglePIDConKp, cfg.anglePIDConKi, cfg.anglePIDConKd);
  bool reported = false;
  for (auto &l : received) if (l == "EV,Autotune done") reported = true;
  ok &= check(reported, "reports 'Autotune done'");
  return ok;
}

// Used by tune_gains.py: plant and gains from the environment, prints one score line.
static bool scenarioScore() {
  if (getenv("SIM_TAU")) plant.motorTau = atof(getenv("SIM_TAU"));
  if (getenv("SIM_L")) plant.effLength = atof(getenv("SIM_L"));
  if (getenv("SIM_OFFSET")) plant.mountOffsetDeg = atof(getenv("SIM_OFFSET"));
  if (getenv("SIM_NOISE")) plant.noiseDeg = atof(getenv("SIM_NOISE"));
  if (getenv("SIM_STICTION")) plant.stictionPwm = atoi(getenv("SIM_STICTION"));
  boot();
  startBalancing(3);
  stats.reset();
  run(8);
  plant.omega += 0.5;
  run(4);
  plant.omega -= 0.5;
  run(4);
  send("SCMD Throttle 100");
  run(5);
  send("SCMD Throttle -100");
  run(5);
  send("SCMD Throttle 0");
  double x0 = plant.x;
  run(8);
  printf("SCORE %d %.3f %.3f %.3f %.3f\n", stats.falls + (state != STATE_BALANCING), rmsDeg(), stats.maxAbsDeg,
         fabs(plant.x - x0), fabs((plant.vl + plant.vr) / 2));
  return true;
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
      telemetryOk = count(l) == count("TH,yaw,pitch,roll,pitchSpeed,dISTE,leftMotorSpeed,rightMotorSpeed,LoopT,bal") &&
                    l.size() < 64;
  ok &= check(telemetryOk, "telemetry line matches header and fits the 64 byte TX buffer");

  send("SCMD Throttle 50");
  run(0.01);
  ok &= check(throttleCmd == 50, "Throttle stored without double scaling");
  send("SCMD Throttle 500");
  run(0.01);
  ok &= check(throttleCmd == 100, "Throttle clamped to 100");

  received.clear();
  send("SCMD bogus 1");
  run(0.01);
  ok &= check(!received.empty() && received.back().rfind("E, Unknown command bogus", 0) == 0, "unknown command reported");

  // A line longer than the buffer must not overflow it.
  send(std::string(200, 'X'));
  run(0.01);
  ok &= check(true, "long line ignored without crashing");
  return ok;
}

static bool scenarioEeprom() {
  boot();
  bool ok = check(cfg.magic == SETTINGS_MAGIC, "blank EEPROM -> defaults");
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

int main(int argc, char **argv) {
  struct { const char *name; bool (*fn)(); } scenarios[] = {
    { "balance", scenarioBalance },   { "push", scenarioPush },
    { "drive", scenarioDrive },       { "fall", scenarioFall },
    { "autotune", scenarioAutotune }, { "commands", scenarioCommands },
    { "eeprom", scenarioEeprom },     { "score", scenarioScore },     { "motortest", scenarioMotorTest },
  };
  if (argc < 2) {
    printf("usage: %s <scenario>\n", argv[0]);
    for (auto &s : scenarios) printf("  %s\n", s.name);
    return 2;
  }
  for (auto &s : scenarios) {
    if (strcmp(argv[1], s.name) == 0) {
      printf("%s:\n", s.name);
      return s.fn() ? 0 : 1;
    }
  }
  printf("unknown scenario %s\n", argv[1]);
  return 2;
}
