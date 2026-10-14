// Serial protocol with the Node.js server (server/app/server.js).
//
// Commands (one per line):
//   SCMD <name> <value>   set a parameter or trigger an action, echoed back as "SCMD <name> <value>"
//   READ <what>           RemoteInit | PIDParamTX | SYSParamTX | printConfig
//
// Lines sent by the robot start with a tag:
//   TH/T     telemetry header / values      PIDH/PID  PID header / values
//   SYSH/SYS system header / values         EV        event (state changes, autotune)
//   AT       autotune progress and result   TEST      motor test result
//   ***      configuration dump             E         error

#define LINE_LEN 80          // longest command plus its "*HH" checksum
static char lineBuf[LINE_LEN];
static uint8_t lineLen = 0;

LineOut out;

size_t LineOut::write(uint8_t c) {
  if (c == '\r') return 1;   // println's "\r\n": the checksum goes before it
  if (c == '\n') {
    static const char hex[] = "0123456789ABCDEF";
    Serial.write('*');
    Serial.write(hex[crc >> 4]);
    Serial.write(hex[crc & 15]);
    Serial.write('\r');
    Serial.write('\n');
    crc = 0;
    return 1;
  }
  crc = crc8(crc, c);
  return Serial.write(c);
}

// Checks and removes a "*HH" checksum. Returns false for a line whose
// checksum doesn't match (lines without one pass unchanged).
static bool checkLine(char *line) {
  size_t n = strlen(line);
  if (n < 3 || line[n - 3] != '*') return true;
  char *end;
  long expected = strtol(line + n - 2, &end, 16);
  if (*end != 0) return true;   // not "*HH": just text that happens to contain '*'
  line[n - 3] = 0;
  uint8_t crc = 0;
  for (const char *p = line; *p; p++) crc = crc8(crc, *p);
  return crc == expected;
}

unsigned long badLines = 0;     // lines dropped for a wrong checksum (printConfig)

void commsRead() {
  // Non-blocking: only consume what has arrived, never wait for more.
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
        lineLen = 0;
      }
    } else if (lineLen < LINE_LEN - 1) {
      lineBuf[lineLen++] = c;
    }
  }
}

static bool is(const char *a, const char *progmemName) {
  return strcmp_P(a, progmemName) == 0;
}

// Parameters set as "SCMD <name> <value>", value is divided by `scale`.
// The scales match the web UI sliders (gains are sent as integers).
struct FloatParam {
  const char *name;
  float *value;
  float scale;
  bool pid;          // part of the PID line: send it again after a change
};

static const char P_speedPIDKp[] PROGMEM = "speedPIDKp";
static const char P_speedPIDKi[] PROGMEM = "speedPIDKi";
static const char P_speedPIDKd[] PROGMEM = "speedPIDKd";
static const char P_anglePIDConKp[] PROGMEM = "anglePIDConKp";
static const char P_anglePIDConKi[] PROGMEM = "anglePIDConKi";
static const char P_anglePIDConKd[] PROGMEM = "anglePIDConKd";
static const char P_anglePIDAggKp[] PROGMEM = "anglePIDAggKp";
static const char P_anglePIDAggKi[] PROGMEM = "anglePIDAggKi";
static const char P_anglePIDAggKd[] PROGMEM = "anglePIDAggKd";
static const char P_TriggerAngleAggressive[] PROGMEM = "TriggerAngleAggressive";
static const char P_calibratedZeroAngle[] PROGMEM = "calibratedZeroAngle";
static const char P_anglePIDOutputLimit[] PROGMEM = "anglePIDOutputLimit";
static const char P_fallAngle[] PROGMEM = "fallAngle";
static const char P_speedPIDMoveFactor[] PROGMEM = "speedPIDMoveFactor";
static const char P_speedPIDOutputLimit[] PROGMEM = "speedPIDOutputLimit";
static const char P_speedFilter[] PROGMEM = "speedFilter";
static const char P_speedRamp[] PROGMEM = "speedRamp";
static const char P_speedFeedForward[] PROGMEM = "speedFeedForward";
static const char P_Maxsteer[] PROGMEM = "Maxsteer";
static const char P_Maxthrottle[] PROGMEM = "Maxthrottle";
static const char P_stepperMaxSpeed[] PROGMEM = "stepperMaxSpeed";
static const char P_stepperMaxAccel[] PROGMEM = "stepperMaxAccel";
static const char P_stepsPerRev[] PROGMEM = "stepsPerRev";
static const char P_wheelDiameter[] PROGMEM = "wheelDiameter";
static const char P_autotuneAmplitude[] PROGMEM = "autotuneAmplitude";
static const char P_lqrMaxPosition[] PROGMEM = "lqrMaxPosition";
static const char P_lqrMaxSpeed[] PROGMEM = "lqrMaxSpeed";
static const char P_lqrMaxAngle[] PROGMEM = "lqrMaxAngle";
static const char P_lqrMaxRate[] PROGMEM = "lqrMaxRate";
static const char P_lqrMaxAccel[] PROGMEM = "lqrMaxAccel";
static const char P_lqrAccelLimit[] PROGMEM = "lqrAccelLimit";
static const char P_dobBandwidth[] PROGMEM = "dobBandwidth";
static const char P_kalmanQAngle[] PROGMEM = "kalmanQAngle";
static const char P_kalmanQBias[] PROGMEM = "kalmanQBias";
static const char P_kalmanR[] PROGMEM = "kalmanR";
static const char P_pendulumCoupling[] PROGMEM = "pendulumCoupling";

static const FloatParam floatParams[] = {
  { P_speedPIDKp, &cfg.speedPIDKp, 10000, true },
  { P_speedPIDKi, &cfg.speedPIDKi, 10000, true },
  { P_speedPIDKd, &cfg.speedPIDKd, 10000, true },
  { P_anglePIDConKp, &cfg.anglePIDConKp, 100, true },
  { P_anglePIDConKi, &cfg.anglePIDConKi, 100, true },
  { P_anglePIDConKd, &cfg.anglePIDConKd, 100, true },
  { P_anglePIDAggKp, &cfg.anglePIDAggKp, 100, true },
  { P_anglePIDAggKi, &cfg.anglePIDAggKi, 100, true },
  { P_anglePIDAggKd, &cfg.anglePIDAggKd, 100, true },
  { P_TriggerAngleAggressive, &cfg.TriggerAngleAggressive, 100, true },
  { P_calibratedZeroAngle, &cfg.calibratedZeroAngle, 100, true },
  { P_anglePIDOutputLimit, &cfg.anglePIDOutputLimit, 1, false },
  { P_fallAngle, &cfg.fallAngle, 1, false },
  { P_speedPIDMoveFactor, &cfg.speedPIDMoveFactor, 100, false },
  { P_speedPIDOutputLimit, &cfg.speedPIDOutputLimit, 1, false },
  { P_speedFilter, &cfg.speedFilter, 100, false },
  { P_speedRamp, &cfg.speedRamp, 1, false },
  { P_speedFeedForward, &cfg.speedFeedForward, 100, false },
  { P_Maxsteer, &cfg.Maxsteer, 1, false },
  { P_Maxthrottle, &cfg.Maxthrottle, 1, false },
  { P_stepperMaxSpeed, &cfg.stepperMaxSpeed, 1, false },
  { P_stepperMaxAccel, &cfg.stepperMaxAccel, 1, false },
  { P_stepsPerRev, &cfg.stepsPerRev, 1, false },
  { P_wheelDiameter, &cfg.wheelDiameter, 1, false },
  { P_autotuneAmplitude, &cfg.autotuneAmplitude, 100, false },
  { P_lqrMaxPosition, &cfg.lqrMaxPosition, 1, false },
  { P_lqrMaxSpeed, &cfg.lqrMaxSpeed, 1, false },
  { P_lqrMaxAngle, &cfg.lqrMaxAngle, 1, false },
  { P_lqrMaxRate, &cfg.lqrMaxRate, 1, false },
  { P_lqrMaxAccel, &cfg.lqrMaxAccel, 1, false },
  { P_lqrAccelLimit, &cfg.lqrAccelLimit, 1, false },
  { P_dobBandwidth, &cfg.dobBandwidth, 1, false },
  { P_kalmanQAngle, &cfg.kalmanQAngle, 1, false },
  { P_kalmanQBias, &cfg.kalmanQBias, 1, false },
  { P_kalmanR, &cfg.kalmanR, 1, false },
  { P_pendulumCoupling, &cfg.pendulumCoupling, 100, true },
};

struct IntParam {
  const char *name;
  void *value;
  uint8_t size;      // 1 or 2 bytes
  long minValue, maxValue;
};

static const char P_imuAxis[] PROGMEM = "imuAxis";
static const char P_imuInvert[] PROGMEM = "imuInvert";
static const char P_imuAccelCompensation[] PROGMEM = "imuAccelCompensation";
static const char P_motorLeftInvert[] PROGMEM = "motorLeftInvert";
static const char P_motorRightInvert[] PROGMEM = "motorRightInvert";
static const char P_debugSampleRate[] PROGMEM = "debugSampleRate";
static const char P_telemetryPeriod[] PROGMEM = "telemetryPeriod";
static const char P_controller[] PROGMEM = "controller";

static const IntParam intParams[] = {
  { P_imuAxis, &cfg.imuAxis, 1, 0, 2 },
  { P_imuInvert, &cfg.imuInvert, 1, 0, 1 },
  { P_imuAccelCompensation, &cfg.imuAccelCompensation, 1, 0, 1 },
  { P_motorLeftInvert, &cfg.motorLeftInvert, 1, 0, 1 },
  { P_motorRightInvert, &cfg.motorRightInvert, 1, 0, 1 },
  { P_debugSampleRate, &cfg.debugSampleRate, 2, 50, 10000 },
  { P_telemetryPeriod, &cfg.telemetryPeriod, 2, 50, 10000 },
  { P_controller, &cfg.controller, 1, 0, 2 },
};

#define COUNT(a) (sizeof(a) / sizeof(a[0]))

static void unknownCommand(const char *arg, const char *value) {
  out.print(F("E, Unknown command "));
  out.print(arg ? arg : "");
  if (value) {
    out.print(F(" arg "));
    out.print(value);
  }
  out.println();
}

static bool setParam(const char *arg, const char *value) {
  for (uint8_t i = 0; i < COUNT(floatParams); i++) {
    if (is(arg, floatParams[i].name)) {
      float v = atof(value) / floatParams[i].scale;
      // LQR weights must stay positive; the new gain is solved when the motors are off.
      if (strncmp_P(arg, PSTR("lqrMax"), 6) == 0) {
        if (v <= 0) return true;
        lqrWeightsChanged = true;
        if (state == STATE_BALANCING) out.println(F("E, LQR weights apply when the motors are off"));
      }
      if (floatParams[i].value == &cfg.calibratedZeroAngle) lqrZeroShifted(v - cfg.calibratedZeroAngle);
      *floatParams[i].value = v;
      if (floatParams[i].pid) PIDParamTX();
      return true;
    }
  }
  for (uint8_t i = 0; i < COUNT(intParams); i++) {
    if (is(arg, intParams[i].name)) {
      long v = atol(value);
      if (v < intParams[i].minValue) v = intParams[i].minValue;
      if (v > intParams[i].maxValue) v = intParams[i].maxValue;
      if (intParams[i].size == 1)
        *(uint8_t *)intParams[i].value = (uint8_t)v;
      else
        *(uint16_t *)intParams[i].value = (uint16_t)v;
      return true;
    }
  }
  return false;
}

static void setCommand(const char *arg, const char *value) {
  float v = atof(value);

  if (is(arg, P_controller)) {
    // Switching controllers mid-balance would hand over a half-wound state.
    if (motorsEnabled) {
      out.println(F("E, switch controller with the motors off"));
      return;
    }
    setParam(arg, value);
    PIDParamTX();
    LQRParamTX();
  } else if (setParam(arg, value)) {
    // The gains are physical (gains.ino): if the speed scale changed, recompute them.
    if (is(arg, P_stepperMaxSpeed) || is(arg, P_stepsPerRev) || is(arg, P_wheelDiameter) || is(arg, P_pendulumCoupling)) {
      applyGainsForLength(cfg.pendulumLength);
      PIDParamTX();
    }
  } else if (is(arg, PSTR("Motors"))) {
    motorsEnabled = atoi(value) != 0;
    if (!motorsEnabled) {
      autotuneAbort("motors off");
      motorsStop();
    }
  } else if (is(arg, PSTR("Steer"))) {
    steerCmd = constrain(v, -100, 100);
    lastDriveCmd = millis();
  } else if (is(arg, PSTR("Throttle"))) {
    throttleCmd = constrain(v, -100, 100);
    lastDriveCmd = millis();
  } else if (is(arg, PSTR("AUTOTUNE"))) {
    if (atoi(value)) autotuneStart();
    else autotuneAbort("stopped");
  } else if (is(arg, PSTR("pendulumLength"))) {
    // "SCMD pendulumLength 230": set gains for a measured length in mm (steppers).
    float mm = constrain(v, 50, 600);
    applyGainsForLength(mm / 1000.0f);
    PIDParamTX();
  } else if (is(arg, PSTR("SerialDebug"))) {
    cfg.debug = atoi(value) != 0;
  } else if (is(arg, PSTR("CalibrateZero"))) {
    // Hold the robot at its balance point and send "SCMD CalibrateZero 1".
    lqrZeroShifted(pitchRaw - cfg.calibratedZeroAngle);
    cfg.calibratedZeroAngle = pitchRaw;
    PIDParamTX();
  } else if (is(arg, PSTR("MotorTest"))) {
    // "SCMD MotorTest 30": both wheels at 30 % for one second with Motors off.
    // Both wheels must turn forward and the reported speeds must be positive.
    if (state == STATE_DISARMED) {
      motorTestPower = constrain(v, -100, 100);
      motorTestUntil = millis() + 1000;
    } else {
      out.println(F("E, MotorTest needs Motors 0"));
    }
  } else if (is(arg, PSTR("resetPID"))) {
    cfg.anglePIDConKp = cfg.anglePIDConKi = cfg.anglePIDConKd = 0;
    cfg.anglePIDAggKp = cfg.anglePIDAggKi = cfg.anglePIDAggKd = 0;
    cfg.speedPIDKp = cfg.speedPIDKi = cfg.speedPIDKd = 0;
    PIDParamTX();
  } else if (is(arg, PSTR("SaveCfg"))) {
    saveSettings();
    setEvent("Config saved");
  } else if (is(arg, PSTR("Load_def"))) {
    loadDefaultSettings();
    setEvent("Defaults loaded");
    PIDParamTX();
  } else if (is(arg, PSTR("E_EEPROM"))) {
    eraseSettings();
    setEvent("Settings erased");
  } else if (is(arg, PSTR("test"))) {
    out.print(F("this is a test arg "));
    out.println(value);
  } else {
    unknownCommand(arg, value);
  }
}

void handleLine(char *line) {
  if (!checkLine(line)) {
    badLines++;
    out.println(F("E, checksum mismatch, line ignored"));
    return;
  }
  char *cmd = strtok(line, " \t");
  char *arg = strtok(NULL, " \t");
  char *value = strtok(NULL, " \t");
  if (!cmd) return;

  if (is(cmd, PSTR("SCMD"))) {
    if (!arg || !value) {
      unknownCommand(arg, NULL);
      return;
    }
    // Echo, except for the drive commands that arrive ~10 times a second.
    if (!is(arg, PSTR("Throttle")) && !is(arg, PSTR("Steer"))) {
      out.print(F("SCMD "));
      out.print(arg);
      out.print(' ');
      out.println(value);
    }
    setCommand(arg, value);
  } else if (is(cmd, PSTR("READ"))) {
    if (!arg) arg = (char *)"";
    if (is(arg, PSTR("RemoteInit"))) remoteInit();
    else if (is(arg, PSTR("PIDParamTX"))) PIDParamTX();
    else if (is(arg, PSTR("LQRParamTX"))) LQRParamTX();
    else if (is(arg, PSTR("SYSParamTX"))) SYSParamTX();
    else if (is(arg, PSTR("printConfig"))) printConfig();
    else unknownCommand(arg, NULL);
  } else {
    unknownCommand(cmd, NULL);
  }
}

// ---------------- Outgoing ----------------

static void printField(float v, uint8_t decimals) {
  out.print(',');
  out.print(v, decimals);
}

void remoteInit() {
  // The server builds its dictionaries from these headers, so the order must
  // match telemetryTX(), PIDParamTX() and SYSParamTX().
  out.println(F("TH,yaw,pitch,roll,pitchSpeed,dISTE,leftMotorSpeed,rightMotorSpeed,LoopT,bal,balAngle"));
  out.println(F("PIDH,speedPIDKp,speedPIDKi,speedPIDKd,anglePIDConKp,anglePIDConKi,anglePIDConKd,"
                   "anglePIDAggKp,anglePIDAggKi,anglePIDAggKd,TriggerAngleAggressive,calibratedZeroAngle,OPT,controller"));
  out.println(F("SYSH,FirmwareVersion,Board,ControlPeriodUs"));
}

void telemetryTX() {
  // Short (~65 chars): at 115200 baud the 64-byte TX buffer drains in ~6 ms, so
  // printing never holds up the control loop for more than a fraction of a ms.
  out.print('T');
  printField(yaw, 1);
  printField(pitch, 2);
  printField(roll, 1);
  printField(pitchRate, 1);
  printField(iste, 3);
  printField(leftSpeed, 1);
  printField(rightSpeed, 1);
  out.print(',');
  out.print(maxLoopUs);
  out.print(',');
  out.print((int)state);
  printField(balanceAngle, 2);     // pitch the disturbance observer says the robot balances at
  out.println();
  iste = 0;
  maxLoopUs = 0;
}

void debugTX() {
  static const char *const names[] = { "OFF", "WAIT", "BAL", "FALL" };
  out.print(names[state]);
  out.print(F(" pitch:"));
  out.print(pitch, 2);
  out.print(F(" raw:"));
  out.print(pitchRaw, 2);
  out.print(F(" set:"));
  out.print(angleSetpoint, 2);
  out.print(F(" out:"));
  out.print(angleOutput, 1);
  out.print(F(" spd:"));
  out.print(wheelSpeed, 1);
  out.print('/');
  out.print(speedSetpoint, 1);
  out.print(F(" L:"));
  out.print(leftSpeed, 1);
  out.print(F(" R:"));
  out.print(rightSpeed, 1);
  out.print(F(" us:"));
  out.println(maxLoopUs);
  maxLoopUs = 0;
}

void motorTestReport() {
  // Speeds measured at the end of a MotorTest run.
  if (motorTestUntil == 0 || millis() < motorTestUntil) return;
  motorTestUntil = 0;
  out.print(F("TEST"));
  printField(leftSpeed, 1);
  printField(rightSpeed, 1);
  out.println();
}

void PIDParamTX() {
  // Same integer scaling as the "SCMD" setters so the UI sliders round-trip.
  out.print(F("PID,"));
  out.print(lround(cfg.speedPIDKp * 10000)); out.print(',');
  out.print(lround(cfg.speedPIDKi * 10000)); out.print(',');
  out.print(lround(cfg.speedPIDKd * 10000)); out.print(',');
  out.print(lround(cfg.anglePIDConKp * 100)); out.print(',');
  out.print(lround(cfg.anglePIDConKi * 100)); out.print(',');
  out.print(lround(cfg.anglePIDConKd * 100)); out.print(',');
  out.print(lround(cfg.anglePIDAggKp * 100)); out.print(',');
  out.print(lround(cfg.anglePIDAggKi * 100)); out.print(',');
  out.print(lround(cfg.anglePIDAggKd * 100)); out.print(',');
  out.print(lround(cfg.TriggerAngleAggressive * 100)); out.print(',');
  out.print(lround(cfg.calibratedZeroAngle * 100)); out.print(',');
  out.print(2); out.print(',');   // OPT: motor type, always 2 (steppers) since firmware 3
  out.println(cfg.controller);
}

void SYSParamTX() {
  out.print(F("SYS,"));
  out.print(F(FIRMWARE_VERSION));
  out.print(F("," BOARD_NAME ","));
  out.println(CONTROL_PERIOD_US);
}

void printConfig() {
  out.print(F("*** bad lines (checksum) = "));
  out.println(badLines);
  out.print(F("*** pendulumLength = "));
  out.println(cfg.pendulumLength, 3);
  out.print(F("*** pendulumCoupling = "));
  out.println(cfg.pendulumCoupling, 3);
  for (uint8_t i = 0; i < COUNT(floatParams); i++) {
    out.print(F("*** "));
    out.print((const __FlashStringHelper *)floatParams[i].name);
    out.print(F(" = "));
    out.println(*floatParams[i].value, 4);
  }
  for (uint8_t i = 0; i < COUNT(intParams); i++) {
    out.print(F("*** "));
    out.print((const __FlashStringHelper *)intParams[i].name);
    out.print(F(" = "));
    if (intParams[i].size == 1)
      out.println(*(uint8_t *)intParams[i].value);
    else
      out.println(*(uint16_t *)intParams[i].value);
  }
}
