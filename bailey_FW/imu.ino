// IMU: MPU-6050 read directly at the control rate, fused with a complementary filter.
//
// The balance axis is the gyro axis the robot pitches around (imuAxis: 0 = X,
// 1 = Y, 2 = Z of the sensor). The tilt from the accelerometer is measured in the
// plane of the other two axes, so the board can be mounted in any orientation:
// pick the axis, set imuInvert so leaning forward is positive, then CalibrateZero.

MPU6050 mpu(MPU_ADDRESS);

#define GYRO_SCALE   (1.0f / 65.5f)    // +/-500 deg/s range: LSB per deg/s
#define FILTER_TAU   0.5f              // s, complementary filter time constant

static float tiltEstimate = 0;          // deg, continuous (not wrapped)
static bool imuStarted = false;

static float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void imuSetup() {
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection())
    Serial.println(F("E, MPU6050 not found"));
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);   // removes motor vibration, ~5 ms delay
  // Measures and removes the gyro bias: keep the robot still during boot.
  mpu.CalibrateGyro(6);
}

void imuRead(float dt) {
  int16_t a[3], g[3];
  mpu.getMotion6(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2]);

  uint8_t r = cfg.imuAxis > 2 ? 0 : cfg.imuAxis;
  uint8_t u = (r + 1) % 3, v = (r + 2) % 3;
  float sign = cfg.imuInvert ? -1.0f : 1.0f;

  float rate = sign * g[r] * GYRO_SCALE;
  float accTilt = sign * atan2((float)a[u], (float)a[v]) * 57.29578f;

  if (!imuStarted) {
    tiltEstimate = accTilt;
    imuStarted = true;
  }
  // Gyro for fast changes, accelerometer to remove drift. Work on the wrapped
  // difference so it doesn't jump when the tilt passes +/-180.
  float k = dt / (FILTER_TAU + dt);
  tiltEstimate += rate * dt;
  tiltEstimate += k * wrap180(accTilt - tiltEstimate);
  tiltEstimate = wrap180(tiltEstimate);

  pitchRaw = tiltEstimate;
  pitch = wrap180(pitchRaw - cfg.calibratedZeroAngle);
  pitchRate = rate;

  // For telemetry only: heading from the gyro axis that is vertical when upright,
  // and side tilt from the accelerometer.
  yaw = wrap180(yaw + g[v] * GYRO_SCALE * dt);
  roll = atan2((float)a[r], (float)a[v]) * 57.29578f;
}
