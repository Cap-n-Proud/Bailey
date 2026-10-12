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

#define LINE_LEN 64
static char lineBuf[LINE_LEN];
static uint8_t lineLen = 0;

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
static const char P_encoderCountsPerSecFull[] PROGMEM = "encoderCountsPerSecFull";
static const char P_stepperMaxSpeed[] PROGMEM = "stepperMaxSpeed";
static const char P_stepperMaxAccel[] PROGMEM = "stepperMaxAccel";
static const char P_stepsPerRev[] PROGMEM = "stepsPerRev";
static const char P_encoderCountsPerRev[] PROGMEM = "encoderCountsPerRev";
static const char P_wheelDiameter[] PROGMEM = "wheelDiameter";
static const char P_autotuneAmplitude[] PROGMEM = "autotuneAmplitude";

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
  { P_encoderCountsPerSecFull, &cfg.encoderCountsPerSecFull, 1, false },
  { P_stepperMaxSpeed, &cfg.stepperMaxSpeed, 1, false },
  { P_stepperMaxAccel, &cfg.stepperMaxAccel, 1, false },
  { P_stepsPerRev, &cfg.stepsPerRev, 1, false },
  { P_encoderCountsPerRev, &cfg.encoderCountsPerRev, 1, false },
  { P_wheelDiameter, &cfg.wheelDiameter, 1, false },
  { P_autotuneAmplitude, &cfg.autotuneAmplitude, 100, false },
};

struct IntParam {
  const char *name;
  void *value;
  uint8_t size;      // 1 or 2 bytes
  long minValue, maxValue;
};

static const char P_motorLeftMinimumSpeed[] PROGMEM = "motorLeftMinimumSpeed";
static const char P_motorRightMinimumSpeed[] PROGMEM = "motorRightMinimumSpeed";
static const char P_imuAxis[] PROGMEM = "imuAxis";
static const char P_imuInvert[] PROGMEM = "imuInvert";
static const char P_motorLeftInvert[] PROGMEM = "motorLeftInvert";
static const char P_motorRightInvert[] PROGMEM = "motorRightInvert";
static const char P_encoderLeftInvert[] PROGMEM = "encoderLeftInvert";
static const char P_encoderRightInvert[] PROGMEM = "encoderRightInvert";
static const char P_debugSampleRate[] PROGMEM = "debugSampleRate";
static const char P_telemetryPeriod[] PROGMEM = "telemetryPeriod";

static const IntParam intParams[] = {
  { P_motorLeftMinimumSpeed, &cfg.motorLeftMinimumSpeed, 1, 0, 254 },
  { P_motorRightMinimumSpeed, &cfg.motorRightMinimumSpeed, 1, 0, 254 },
  { P_imuAxis, &cfg.imuAxis, 1, 0, 2 },
  { P_imuInvert, &cfg.imuInvert, 1, 0, 1 },
  { P_motorLeftInvert, &cfg.motorLeftInvert, 1, 0, 1 },
  { P_motorRightInvert, &cfg.motorRightInvert, 1, 0, 1 },
  { P_encoderLeftInvert, &cfg.encoderLeftInvert, 1, 0, 1 },
  { P_encoderRightInvert, &cfg.encoderRightInvert, 1, 0, 1 },
  { P_debugSampleRate, &cfg.debugSampleRate, 2, 50, 10000 },
  { P_telemetryPeriod, &cfg.telemetryPeriod, 2, 50, 10000 },
};

#define COUNT(a) (sizeof(a) / sizeof(a[0]))

static void unknownCommand(const char *arg, const char *value) {
  Serial.print(F("E, Unknown command "));
  Serial.print(arg ? arg : "");
  if (value) {
    Serial.print(F(" arg "));
    Serial.print(value);
  }
  Serial.println();
}

static bool setParam(const char *arg, const char *value) {
  for (uint8_t i = 0; i < COUNT(floatParams); i++) {
    if (is(arg, floatParams[i].name)) {
      *floatParams[i].value = atof(value) / floatParams[i].scale;
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

  if (setParam(arg, value)) {
#if MOTOR_TYPE == MOTOR_STEPPER
    // The gains are physical (gains.ino): if the speed scale changed, recompute them.
    if (is(arg, P_stepperMaxSpeed) || is(arg, P_stepsPerRev) || is(arg, P_wheelDiameter)) {
      applyGainsForLength(cfg.pendulumLength);
      PIDParamTX();
    }
#endif
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
#if MOTOR_TYPE == MOTOR_STEPPER
    float mm = constrain(v, 50, 600);
    applyGainsForLength(mm / 1000.0f);
    PIDParamTX();
#else
    Serial.println(F("E, pendulumLength needs stepper motors"));
#endif
  } else if (is(arg, PSTR("SerialDebug"))) {
    cfg.debug = atoi(value) != 0;
  } else if (is(arg, PSTR("CalibrateZero"))) {
    // Hold the robot at its balance point and send "SCMD CalibrateZero 1".
    cfg.calibratedZeroAngle = pitchRaw;
    PIDParamTX();
  } else if (is(arg, PSTR("MotorTest"))) {
    // "SCMD MotorTest 30": both wheels at 30 % for one second with Motors off.
    // Both wheels must turn forward and the reported speeds must be positive.
    if (state == STATE_DISARMED) {
      motorTestPower = constrain(v, -100, 100);
      motorTestUntil = millis() + 1000;
    } else {
      Serial.println(F("E, MotorTest needs Motors 0"));
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
    setEvent("EEPROM erased");
  } else if (is(arg, PSTR("test"))) {
    Serial.print(F("this is a test arg "));
    Serial.println(value);
  } else {
    unknownCommand(arg, value);
  }
}

void handleLine(char *line) {
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
      Serial.print(F("SCMD "));
      Serial.print(arg);
      Serial.print(' ');
      Serial.println(value);
    }
    setCommand(arg, value);
  } else if (is(cmd, PSTR("READ"))) {
    if (!arg) arg = (char *)"";
    if (is(arg, PSTR("RemoteInit"))) remoteInit();
    else if (is(arg, PSTR("PIDParamTX"))) PIDParamTX();
    else if (is(arg, PSTR("SYSParamTX"))) SYSParamTX();
    else if (is(arg, PSTR("printConfig"))) printConfig();
    else unknownCommand(arg, NULL);
  } else {
    unknownCommand(cmd, NULL);
  }
}

// ---------------- Outgoing ----------------

static void printField(float v, uint8_t decimals) {
  Serial.print(',');
  Serial.print(v, decimals);
}

void remoteInit() {
  // The server builds its dictionaries from these headers, so the order must
  // match telemetryTX(), PIDParamTX() and SYSParamTX().
  Serial.println(F("TH,yaw,pitch,roll,pitchSpeed,dISTE,leftMotorSpeed,rightMotorSpeed,LoopT,bal"));
  Serial.println(F("PIDH,speedPIDKp,speedPIDKi,speedPIDKd,anglePIDConKp,anglePIDConKi,anglePIDConKd,"
                   "anglePIDAggKp,anglePIDAggKi,anglePIDAggKd,TriggerAngleAggressive,calibratedZeroAngle,OPT"));
  Serial.println(F("SYSH,FirmwareVersion"));
}

void telemetryTX() {
  // Kept short (< 64 chars, the serial TX buffer) so printing never blocks the control loop.
  Serial.print('T');
  printField(yaw, 1);
  printField(pitch, 2);
  printField(roll, 1);
  printField(pitchRate, 1);
  printField(iste, 3);
  printField(leftSpeed, 1);
  printField(rightSpeed, 1);
  Serial.print(',');
  Serial.print(maxLoopUs);
  Serial.print(',');
  Serial.println((int)state);
  iste = 0;
  maxLoopUs = 0;
}

void debugTX() {
  static const char *const names[] = { "OFF", "WAIT", "BAL", "FALL" };
  Serial.print(names[state]);
  Serial.print(F(" pitch:"));
  Serial.print(pitch, 2);
  Serial.print(F(" raw:"));
  Serial.print(pitchRaw, 2);
  Serial.print(F(" set:"));
  Serial.print(angleSetpoint, 2);
  Serial.print(F(" out:"));
  Serial.print(angleOutput, 1);
  Serial.print(F(" spd:"));
  Serial.print(wheelSpeed, 1);
  Serial.print('/');
  Serial.print(speedSetpoint, 1);
  Serial.print(F(" L:"));
  Serial.print(leftSpeed, 1);
  Serial.print(F(" R:"));
  Serial.print(rightSpeed, 1);
  Serial.print(F(" us:"));
  Serial.println(maxLoopUs);
  maxLoopUs = 0;
}

void motorTestReport() {
  // Speeds measured at the end of a MotorTest run.
  if (motorTestUntil == 0 || millis() < motorTestUntil) return;
  motorTestUntil = 0;
  Serial.print(F("TEST"));
  printField(leftSpeed, 1);
  printField(rightSpeed, 1);
  Serial.println();
}

void PIDParamTX() {
  // Same integer scaling as the "SCMD" setters so the UI sliders round-trip.
  Serial.print(F("PID,"));
  Serial.print(lround(cfg.speedPIDKp * 10000)); Serial.print(',');
  Serial.print(lround(cfg.speedPIDKi * 10000)); Serial.print(',');
  Serial.print(lround(cfg.speedPIDKd * 10000)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDConKp * 100)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDConKi * 100)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDConKd * 100)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDAggKp * 100)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDAggKi * 100)); Serial.print(',');
  Serial.print(lround(cfg.anglePIDAggKd * 100)); Serial.print(',');
  Serial.print(lround(cfg.TriggerAngleAggressive * 100)); Serial.print(',');
  Serial.print(lround(cfg.calibratedZeroAngle * 100)); Serial.print(',');
  Serial.println(MOTOR_TYPE);
}

void SYSParamTX() {
  Serial.print(F("SYS,"));
  Serial.println(F(FIRMWARE_VERSION));
}

void printConfig() {
  Serial.print(F("*** pendulumLength = "));
  Serial.println(cfg.pendulumLength, 3);
  for (uint8_t i = 0; i < COUNT(floatParams); i++) {
    Serial.print(F("*** "));
    Serial.print((const __FlashStringHelper *)floatParams[i].name);
    Serial.print(F(" = "));
    Serial.println(*floatParams[i].value, 4);
  }
  for (uint8_t i = 0; i < COUNT(intParams); i++) {
    Serial.print(F("*** "));
    Serial.print((const __FlashStringHelper *)intParams[i].name);
    Serial.print(F(" = "));
    if (intParams[i].size == 1)
      Serial.println(*(uint8_t *)intParams[i].value);
    else
      Serial.println(*(uint16_t *)intParams[i].value);
  }
}
