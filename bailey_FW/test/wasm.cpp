// The simulator as a WebAssembly module, for the interactive page in web/.
// Same firmware and robot model as sim.cpp; the page calls these functions
// instead of a scenario. Build with ./build_wasm.sh.
#define SIM_NO_MAIN
#include "sim.cpp"

// The WASI C++ runtime is built with exceptions but has no unwinder; the
// firmware never throws, so out of memory just stops the module.
extern "C" void *__cxa_allocate_exception(size_t) { abort(); }
extern "C" void __cxa_throw(void *, void *, void (*)(void *)) { abort(); }

#define EXPORT extern "C" __attribute__((visibility("default"))) __attribute__((used))

static const double TRACK = 0.20;    // m between the wheels, for the top view only
static double heading = 0, posX = 0, posY = 0, wheelAngle = 0;
static bool autoPickup = true;
static double fallenFor = 0;
static std::string outLines;
static char inBuf[256];

static void collect(const std::string &l) {
  outLines += l;
  outLines += '\n';
}

// Power-on: firmware setup(), robot held upright by a (simulated) person.
EXPORT void sim_reset() {
  plant = Plant();
  boot();
  remoteConnected = true;            // drive commands are repeated like the web UI does
  Serial.onLine = collect;
  outLines.clear();
  heading = posX = posY = wheelAngle = 0;
  fallenFor = 0;
}

// Runs the robot for `seconds` of simulated time in 10 ms slices, with the
// person from the live mode: lets go once it balances, holds it while the
// motors are off, and picks it back up 2 s after a fall.
EXPORT void sim_run(double seconds) {
  for (double t = 0; t < seconds - 1e-9; t += 0.01) {
    double x0 = plant.x;
    run(0.01);
    double d = plant.x - x0;
    wheelAngle += d / plant.wheelRadius;
    heading += (plant.vl - plant.vr) * 0.01 / TRACK;   // vl > vr turns right
    posX += d * sin(heading);
    posY += d * cos(heading);
    if (plant.held && state == STATE_BALANCING) plant.held = false;
    if (state == STATE_FALLEN && autoPickup) {
      fallenFor += 0.01;
      if (fallenFor > 2) {
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
    if (state == STATE_DISARMED) {   // someone holds it while the motors are off
      plant.held = true;
      plant.theta = 0;
      plant.omega = 0;
    }
  }
}

// Serial input: the page writes a line into sim_input() and calls sim_send().
EXPORT char *sim_input() { return inBuf; }
EXPORT void sim_send() {
  inBuf[sizeof inBuf - 1] = 0;
  send(inBuf);
}

// Serial output since the last call, one line per '\n'.
EXPORT const char *sim_output() {
  static std::string taken;
  taken.swap(outLines);
  outLines.clear();
  return taken.c_str();
}

enum { G_THETA, G_OMEGA, G_X, G_SPEED, G_PITCH, G_STATE, G_CONTROLLER, G_BALANCE, G_HELD, G_LENGTH,
       G_TIME, G_HEADING, G_POSX, G_POSY, G_WHEEL, G_DISTURBANCE, G_THROTTLE, G_STEER, G_ZERO, G_LOOPUS,
       G_COM_HEIGHT, G_WHEEL_RADIUS, G_COUPLING, G_FW_COUPLING };

EXPORT double sim_get(int what) {
  switch (what) {
    case G_THETA: return plant.theta * 180 / M_PI;
    case G_OMEGA: return plant.omega * 180 / M_PI;
    case G_X: return plant.x;
    case G_SPEED: return (plant.vl + plant.vr) / 2;
    case G_PITCH: return pitch;
    case G_STATE: return state;
    case G_CONTROLLER: return cfg.controller;
    case G_BALANCE: return dobAngle * 180 / M_PI;
    case G_HELD: return plant.held;
    case G_LENGTH: return plant.fallLength();
    case G_TIME: return simMicros / 1e6;
    case G_HEADING: return heading * 180 / M_PI;
    case G_POSX: return posX;
    case G_POSY: return posY;
    case G_WHEEL: return wheelAngle;
    case G_DISTURBANCE: return plant.disturbance;
    case G_THROTTLE: return throttleCmd;
    case G_STEER: return steerCmd;
    case G_ZERO: return cfg.calibratedZeroAngle;
    case G_LOOPUS: return CONTROL_PERIOD_US;
    case G_COM_HEIGHT: return plant.comHeight;
    case G_WHEEL_RADIUS: return plant.wheelRadius;
    case G_COUPLING: return plant.coupling();
    case G_FW_COUPLING: return cfg.pendulumCoupling;
  }
  return 0;
}

enum { S_PUSH, S_LOAD, S_LENGTH, S_HELD, S_NOISE, S_SLIP, S_AUTOPICKUP, S_PICKUP };

EXPORT void sim_set(int what, double v) {
  switch (what) {
    case S_PUSH: if (!plant.held) plant.omega += v; break;                 // rad/s kick
    case S_LOAD: plant.disturbance = -plant.fallRate() * sin(v * M_PI / 180); break;  // centre of mass moved v deg
    case S_LENGTH: setRobotLength(v); break;
    case S_HELD: plant.held = v != 0; if (plant.held) { plant.theta = 0; plant.omega = 0; } break;
    case S_NOISE: plant.noiseDeg = v; break;
    case S_SLIP: plant.slip = v; break;
    case S_AUTOPICKUP: autoPickup = v != 0; break;
    case S_PICKUP:
      plant.held = true;
      plant.vl = plant.vr = plant.ul = plant.ur = 0;
      plant.theta = plant.omega = 0;
      fallenFor = 0;
      break;
  }
}
