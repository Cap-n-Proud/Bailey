// Settings: defaults and EEPROM storage.
// Stored settings are only used when the magic number, layout version and
// checksum all match, so a new firmware layout or a blank EEPROM falls back to
// the defaults instead of loading garbage.

void loadDefaultSettings() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;

  // Gains from an LQR design of the robot, refined and checked in the simulator
  // (test/) over a range of motor lags, body heights, IMU noise and offsets,
  // with the MPU-6050 complementary filter in the loop.
#if MOTOR_TYPE == MOTOR_DC
  cfg.anglePIDConKp = 16.0;
  cfg.anglePIDConKi = 6.0;
  cfg.anglePIDConKd = 3.5;
  cfg.speedPIDKp = 0.12;
  cfg.speedPIDKi = 0.02;
#else
  // Steppers follow the commanded speed exactly, so the integral term does
  // most of the work.
  cfg.anglePIDConKp = 12.0;
  cfg.anglePIDConKi = 12.0;
  cfg.anglePIDConKd = 1.3;
  cfg.speedPIDKp = 0.15;
  cfg.speedPIDKi = 0.02;
#endif
  cfg.anglePIDAggKp = cfg.anglePIDConKp * 1.3f;
  cfg.anglePIDAggKi = cfg.anglePIDConKi * 1.3f;
  cfg.anglePIDAggKd = cfg.anglePIDConKd * 1.3f;
  cfg.TriggerAngleAggressive = 8.0;
  cfg.anglePIDOutputLimit = 100;
  cfg.calibratedZeroAngle = 0;
  cfg.fallAngle = 40;

  cfg.speedPIDKd = 0;
  cfg.speedPIDMoveFactor = 0.7;
  cfg.speedPIDOutputLimit = 6;
  cfg.speedFilter = 0.15;
  cfg.speedRamp = 30;
  cfg.speedFeedForward = 1.0;

  cfg.Maxsteer = 20;
  cfg.Maxthrottle = 30;

  cfg.motorLeftMinimumSpeed = 55;
  cfg.motorRightMinimumSpeed = 58;
  // Pololu 100:1 gear motor with 64 CPR encoder (x4 decoding): 6400 counts per
  // wheel turn, ~100 rpm at 12 V.
  cfg.encoderCountsPerSecFull = 10700;
  cfg.stepperMaxSpeed = 4000;     // 1/8 microstepping: 1600 steps/rev -> 2.5 rev/s
  cfg.stepperMaxAccel = 40000;

  cfg.imuAxis = 0;
  cfg.imuInvert = 0;
  cfg.motorLeftInvert = 0;
  cfg.motorRightInvert = 0;
  cfg.encoderLeftInvert = 0;
  cfg.encoderRightInvert = 0;

  cfg.numParticles = 8;
  cfg.maxIterations = 10;
  cfg.SPOConfigEval = 3000;
  cfg.SPOSettle = 500;
  cfg.SPOspread = 0.3;

  cfg.debug = 0;
  cfg.debugSampleRate = 500;
  cfg.telemetryPeriod = 250;
}

static uint16_t settingsCrc(const Settings &s) {
  // Fletcher-16 over everything except the crc field itself.
  const uint8_t *p = (const uint8_t *)&s;
  uint16_t a = 0, b = 0;
  for (unsigned int i = 0; i < offsetof(Settings, crc); i++) {
    a = (a + p[i]) % 255;
    b = (b + a) % 255;
  }
  return (b << 8) | a;
}

bool loadSettings() {
  Settings stored;
  EEPROM.get(SETTINGS_ADDRESS, stored);
  if (stored.magic != SETTINGS_MAGIC || stored.version != SETTINGS_VERSION)
    return false;
  if (stored.crc != settingsCrc(stored))
    return false;
  cfg = stored;
  return true;
}

void saveSettings() {
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;
  cfg.crc = settingsCrc(cfg);
  EEPROM.put(SETTINGS_ADDRESS, cfg);   // put() only writes bytes that changed
}

void eraseSettings() {
  for (unsigned int i = 0; i < sizeof(Settings); i++)
    EEPROM.write(SETTINGS_ADDRESS + i, 0xFF);
}
