// IMU: MPU-6050 read directly at the control rate, fused with a Kalman filter.
//
// The filter's state is [pitch, gyro bias]. Each step it predicts pitch from
// the gyro minus the estimated bias, then corrects both with the tilt the
// accelerometer measures. So, unlike a complementary filter, it keeps learning
// the gyro bias (which drifts as the board warms up) instead of leaving a small
// standing pitch error, and pitchRate is bias-corrected for the controllers.
// kalmanR sets how much the accelerometer is trusted: while balancing it is
// disturbed by the robot's own acceleration, so it is trusted only slowly.
//
// The balance axis is the gyro axis the robot pitches around (imuAxis: 0 = X,
// 1 = Y, 2 = Z of the sensor). The tilt from the accelerometer is measured in the
// plane of the other two axes, so the board can be mounted in any orientation:
// pick the axis, set imuInvert so leaning forward is positive, then CalibrateZero.

MPU6050 mpu(MPU_ADDRESS);

#define GYRO_SCALE   (1.0f / 65.5f)    // +/-500 deg/s range: LSB per deg/s
#define ACCEL_LSB_PER_G   8192.0f              // +/-4 g range
#define ACCEL_GATE        0.15f                // ignore the accelerometer beyond 1 g +/- 15 %
static float kalmanPitch = 0;           // deg, wrapped to +/-180
static float kalmanBias = 0;            // deg/s
static float kalmanP[2][2];             // covariance of [pitch, bias]
static bool imuStarted = false;
float imuGyroBias = 0;                  // deg/s, Kalman estimate (telemetry)

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
  // +/-4 g: stepper acceleration ramps alone reach ~2 g and would clip at +/-2 g.
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
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

  float rawRate = sign * g[r] * GYRO_SCALE;

  float accTilt = sign * atan2((float)a[u], (float)a[v]) * 57.29578f;

  // The accelerometer measures gravity plus the robot's own acceleration.
  // While the robot accelerates hard (stepper ramps reach ~2 g) its tilt
  // reading is meaningless, so only trust it when the total is close to 1 g.
  float norm = sqrt((float)a[0] * a[0] + (float)a[1] * a[1] + (float)a[2] * a[2]) / ACCEL_LSB_PER_G;
  bool accelUsable = fabs(norm - 1.0f) < ACCEL_GATE;

  if (!imuStarted) {
    kalmanPitch = accTilt;
    kalmanBias = 0;
    kalmanP[0][0] = 10;   kalmanP[0][1] = 0;    // start: pitch roughly known,
    kalmanP[1][0] = 0;    kalmanP[1][1] = 1;    // bias within ~1 deg/s
    imuStarted = true;
  }

  // Predict: integrate the bias-corrected gyro.
  float rate = rawRate - kalmanBias;
  kalmanPitch += rate * dt;
  float (&P)[2][2] = kalmanP;
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + cfg.kalmanQAngle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += cfg.kalmanQBias * dt;

  // Correct with the accelerometer's pitch, when it can be trusted. Work on the
  // wrapped difference so nothing jumps when the pitch passes +/-180.
  if (accelUsable) {
    float S = P[0][0] + cfg.kalmanR;
    float K0 = P[0][0] / S, K1 = P[1][0] / S;
    float y = wrap180(accTilt - kalmanPitch);
    kalmanPitch += K0 * y;
    kalmanBias += K1 * y;
    float p00 = P[0][0], p01 = P[0][1];
    P[0][0] -= K0 * p00;
    P[0][1] -= K0 * p01;
    P[1][0] -= K1 * p00;
    P[1][1] -= K1 * p01;
  }
  kalmanPitch = wrap180(kalmanPitch);

  pitchRaw = kalmanPitch;
  pitch = wrap180(pitchRaw - cfg.calibratedZeroAngle);
  pitchRate = rawRate - kalmanBias;
  imuGyroBias = kalmanBias;

  // For telemetry only: heading from the gyro axis that is vertical when upright,
  // and side tilt from the accelerometer.
  yaw = wrap180(yaw + g[v] * GYRO_SCALE * dt);
  roll = atan2((float)a[r], (float)a[v]) * 57.29578f;
}
