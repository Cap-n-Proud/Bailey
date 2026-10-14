// Build-time hardware options.
// Everything that can be tuned at runtime lives in Settings (settings.h) and is
// stored in EEPROM; the values here only change when the wiring changes.
#ifndef BAILEY_CONFIG_H
#define BAILEY_CONFIG_H

#define FIRMWARE_VERSION "2.1"
#define SERIAL_BAUD 115200         // must match serial.baudRate in server/config.json
                                   // (ignored on the ESP32-S3: its USB serial runs at full USB speed)

// ---------------- Board ----------------
// Picked automatically from the board selected when compiling:
//   ESP32-S3 (Waveshare ESP32-S3-Pico, or any ESP32-S3 board): Arduino ESP32 core,
//     board "ESP32S3 Dev Module" with USB CDC On Boot = Enabled (see install/flash-firmware.sh)
//   Arduino Mega 2560 (the original Bailey board)
#if defined(ARDUINO_ARCH_ESP32)
#define BOARD_NAME "ESP32-S3"
#define BOARD_ESP32 1
#else
#define BOARD_NAME "Mega2560"
#define BOARD_ESP32 0
#endif

// ---------------- Motor driver ----------------
// MOTOR_DC:      DC gear motors with quadrature encoders on an L298-style H-bridge
//                (PWM enable + two direction pins per motor).
// MOTOR_STEPPER: stepper motors on STEP/DIR drivers (A4988, DRV8825, ...).
#define MOTOR_DC      1
#define MOTOR_STEPPER 2
#ifndef MOTOR_TYPE
#define MOTOR_TYPE MOTOR_STEPPER
#endif

#if MOTOR_TYPE == MOTOR_DC && BOARD_ESP32
#error "The DC motor build is only set up for the Mega; use MOTOR_STEPPER on the ESP32-S3"
#endif

#if MOTOR_TYPE == MOTOR_DC
// Same wiring as the original Bailey build (L29x motorRight(8, 9, 10), motorLeft(11, 12, 13)).
#define PIN_RIGHT_PWM  8
#define PIN_RIGHT_IN1  9
#define PIN_RIGHT_IN2 10
#define PIN_LEFT_PWM  11
#define PIN_LEFT_IN1  12
#define PIN_LEFT_IN2  13
// Encoders: the first pin of each pair should be an interrupt pin.
#define PIN_ENC_LEFT_A   2
#define PIN_ENC_LEFT_B   4
#define PIN_ENC_RIGHT_A  3
#define PIN_ENC_RIGHT_B  5
#elif MOTOR_TYPE == MOTOR_STEPPER && BOARD_ESP32
// ESP32-S3 GPIO numbers. These are free on every ESP32-S3 board with quad PSRAM
// (like the ESP32-S3-Pico's ESP32-S3R2); avoid 0, 3, 45, 46 (boot mode),
// 19, 20 (USB), 26-32 (flash/PSRAM) and 21 (the Pico's RGB LED). Any GPIO
// can drive a STEP pin on the ESP32-S3. All signals are 3.3 V: the A4988,
// DRV8825 and TMC2209 driver inputs and the MPU-6050 accept that directly.
#define PIN_LEFT_STEP   11
#define PIN_LEFT_DIR    12
#define PIN_RIGHT_STEP  13
#define PIN_RIGHT_DIR   14
#define PIN_STEPPER_EN  17       // active LOW, shared by both drivers
#define PIN_I2C_SDA     15       // MPU-6050
#define PIN_I2C_SCL     16
#elif MOTOR_TYPE == MOTOR_STEPPER
// FastAccelStepper generates the pulses in hardware: on the Mega the STEP pins
// must be outputs of Timer 4 (pins 6, 7 or 8); on an Uno, Timer 1 (pins 9, 10).
#define PIN_LEFT_STEP    6
#define PIN_RIGHT_STEP   7
#define PIN_LEFT_DIR     9
#define PIN_RIGHT_DIR   10
#define PIN_STEPPER_EN  12       // active LOW on A4988/DRV8825/TMC2209, shared by both drivers
#else
#error "Unknown MOTOR_TYPE"
#endif

// ---------------- IMU ----------------
#define MPU_ADDRESS 0x68           // MPU-6050 with AD0 low (0x69 if AD0 is high)

// ---------------- Timing ----------------
// Balance loop rate. The ESP32-S3 has a hardware FPU and does it 2.5x as often.
#ifndef CONTROL_PERIOD_US
#if BOARD_ESP32
#define CONTROL_PERIOD_US   2000UL   // 500 Hz
#else
#define CONTROL_PERIOD_US   5000UL   // 200 Hz
#endif
#endif
#define ARM_DELAY_MS        2000UL   // let the tilt filter converge after boot before motors may run
#define UPRIGHT_ANGLE       5.0f     // deg: robot must be this close to upright to start balancing
#define UPRIGHT_HOLD_MS     300UL    // ...for this long
#define DRIVE_TIMEOUT_MS    1000UL   // throttle/steer fall back to 0 unless refreshed this often
#define MAX_DRIVE_SPEED     90.0f    // % of full speed the throttle may ask for (headroom to brake)

#endif
