#pragma once
// Simulated MPU-6050: the simulator writes raw readings into simAccel/simGyro.
#include <stdint.h>
extern int16_t simAccel[3], simGyro[3];
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_DLPF_BW_42 0x03
class MPU6050 {
 public:
  explicit MPU6050(uint8_t = 0x68) {}
  void initialize() {}
  bool testConnection() { return true; }
  void setFullScaleGyroRange(uint8_t) {}
  void setFullScaleAccelRange(uint8_t) {}
  void setDLPFMode(uint8_t) {}
  void CalibrateGyro(uint8_t = 15) {}
  void getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    *ax = simAccel[0]; *ay = simAccel[1]; *az = simAccel[2];
    *gx = simGyro[0]; *gy = simGyro[1]; *gz = simGyro[2];
  }
};
