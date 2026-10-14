// Build-time hardware options.
// Everything that can be tuned at runtime lives in Settings (settings.h) and is
// stored in EEPROM; the values here only change when the wiring changes.
#ifndef BAILEY_CONFIG_H
#define BAILEY_CONFIG_H

#define FIRMWARE_VERSION "3.0"
#define SERIAL_BAUD 115200         // must match serial.baudRate in server/config.json
                                   // (ignored over USB: the ESP32-S3's USB serial runs at full USB speed)

// ---------------- Board ----------------
// ESP32-S3 (Waveshare ESP32-S3-Pico, or any ESP32-S3 board): Arduino ESP32 core,
// board "ESP32S3 Dev Module" with USB CDC On Boot = Enabled (see install/flash-firmware.sh).
// The matrix maths (Eigen) and the MPC solver (TinyMPC) need its C++ standard
// library and FPU; the Arduino Mega and DC motor builds are gone (see the git
// history before firmware 3.0).
#if !defined(ARDUINO_ARCH_ESP32)
#error "Bailey firmware 3 runs on the ESP32-S3: select an ESP32S3 board (see install/flash-firmware.sh)"
#endif
#define BOARD_NAME "ESP32-S3"

// ---------------- Steppers ----------------
// STEP/DIR drivers (A4988, DRV8825, TMC2209, ...). ESP32-S3 GPIO numbers: these
// are free on every ESP32-S3 board with quad PSRAM (like the ESP32-S3-Pico's
// ESP32-S3R2); avoid 0, 3, 45, 46 (boot mode), 19, 20 (USB), 26-32
// (flash/PSRAM) and 21 (the Pico's RGB LED). Any GPIO can drive a STEP pin.
// All signals are 3.3 V: the driver inputs and the MPU-6050 accept that directly.
#define PIN_LEFT_STEP   11
#define PIN_LEFT_DIR    12
#define PIN_RIGHT_STEP  13
#define PIN_RIGHT_DIR   14
#define PIN_STEPPER_EN  17       // active LOW, shared by both drivers
#define PIN_I2C_SDA     15       // MPU-6050
#define PIN_I2C_SCL     16

// ---------------- IMU ----------------
#define MPU_ADDRESS 0x68           // MPU-6050 with AD0 low (0x69 if AD0 is high)

// ---------------- Timing ----------------
#ifndef CONTROL_PERIOD_US
#define CONTROL_PERIOD_US   2000UL   // balance loop: 500 Hz
#endif
#define ARM_DELAY_MS        2000UL   // let the tilt filter converge after boot before motors may run
#define UPRIGHT_ANGLE       5.0f     // deg: robot must be this close to upright to start balancing
#define UPRIGHT_HOLD_MS     300UL    // ...for this long
#define DRIVE_TIMEOUT_MS    1000UL   // throttle/steer fall back to 0 unless refreshed this often
#define MAX_DRIVE_SPEED     90.0f    // % of full speed the throttle may ask for (headroom to brake)

#endif
