// IMU: FreeIMU sensor fusion, read at the control rate.

FreeIMU imu = FreeIMU();

void imuSetup() {
  delay(50);
  imu.init();
  delay(100);
}

void imuRead(float dt) {
  imu.getYawPitchRoll(ypr);

  float axis = ypr[cfg.imuAxis == 2 ? 2 : 1];
  pitchRaw = cfg.imuInvert ? -axis : axis;

  float newPitch = pitchRaw - cfg.calibratedZeroAngle;
  if (dt > 0)
    pitchRate += 0.5f * ((newPitch - pitch) / dt - pitchRate);
  pitch = newPitch;
}
