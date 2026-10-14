// Settings: defaults and EEPROM storage.
// Stored settings are only used when the magic number, layout version and
// checksum all match, so a new firmware layout or a blank EEPROM falls back to
// the defaults instead of loading garbage.

void loadDefaultSettings() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = SETTINGS_MAGIC;
  cfg.version = SETTINGS_VERSION;

  // Hardware (needed first: the stepper gains are computed from it)
  cfg.wheelDiameter = 94;
  cfg.stepperMaxSpeed = 4000;     // 200-step motor at 1/8 microstepping: 2.5 rev/s
  cfg.stepperMaxAccel = 40000;
  cfg.stepsPerRev = 1600;

  cfg.imuAxis = 0;
  cfg.imuInvert = 0;
  cfg.imuAccelCompensation = 1;   // see imu.ino
  cfg.motorLeftInvert = 0;
  cfg.motorRightInvert = 0;

  // Gains, checked in the simulator (test/) over a range of body heights, IMU
  // offsets and noise. The autotuner replaces the guessed
  // pendulum length with a measured one.
  // Bailey's geometry (94 mm wheels, centre of mass about 15 cm up) gives
  // about L = 0.3 m and kappa = 1.4; the autotuner measures both.
  cfg.pendulumLength = 0.30;
  cfg.pendulumCoupling = 1.35;
  applyGainsForLength(cfg.pendulumLength);
  cfg.TriggerAngleAggressive = 8.0;
  cfg.anglePIDOutputLimit = 100;
  cfg.calibratedZeroAngle = 0;
  cfg.fallAngle = 40;

  cfg.speedPIDMoveFactor = 1.0;
  cfg.speedPIDOutputLimit = 6;
  cfg.speedFilter = 0.15;
  cfg.speedRamp = 30;
  cfg.speedFeedForward = 1.0;

  cfg.Maxsteer = 20;
  cfg.Maxthrottle = 30;

  cfg.autotuneAmplitude = 0.7;

  cfg.controller = CONTROLLER_LQR;
  // LQR weights, observer and Kalman settings: chosen in the simulator over
  // robots of 12-30 cm, IMU offsets of +/-2 deg and up to 4x the expected
  // noise, then checked for gyro drift, payload shifts and 7-45 cm robots.
  cfg.lqrMaxPosition = 0.1;
  cfg.lqrMaxSpeed = 0.5;
  cfg.lqrMaxAngle = 4.5;
  cfg.lqrMaxRate = 10;
  cfg.lqrMaxAccel = 2.7;
  cfg.lqrAccelLimit = 6;           // below the steppers' ~7 m/s^2 (stepperMaxAccel)
  cfg.dobBandwidth = 2;

  cfg.kalmanQAngle = 0.002;
  cfg.kalmanQBias = 0.003;
  cfg.kalmanR = 3;

  cfg.debug = 0;
  cfg.debugSampleRate = 500;
  cfg.telemetryPeriod = 100;   // 10 Hz: ~60 bytes per line, fine at 115200 baud
}

static_assert(SETTINGS_ADDRESS + sizeof(Settings) <= EEPROM_BYTES, "Settings don't fit in EEPROM_BYTES");

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
  EEPROM.commit();                     // the ESP32 writes its flash copy only on commit
}

void eraseSettings() {
  for (unsigned int i = 0; i < sizeof(Settings); i++)
    EEPROM.write(SETTINGS_ADDRESS + i, 0xFF);
  EEPROM.commit();
}
