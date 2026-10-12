// Build-time hardware options.
// Everything that can be tuned at runtime lives in Settings (settings.h) and is
// stored in EEPROM; the values here only change when the wiring changes.
#ifndef BAILEY_CONFIG_H
#define BAILEY_CONFIG_H

#define FIRMWARE_VERSION "2.0"
#define SERIAL_BAUD 115200         // must match serial.baudRate in server/config.json

// ---------------- Motor driver ----------------
// MOTOR_DC:      DC gear motors with quadrature encoders on an L298-style H-bridge
//                (PWM enable + two direction pins per motor).
// MOTOR_STEPPER: stepper motors on STEP/DIR drivers (A4988, DRV8825, ...).
#define MOTOR_DC      1
#define MOTOR_STEPPER 2
#ifndef MOTOR_TYPE
#define MOTOR_TYPE MOTOR_STEPPER
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
#define CONTROL_PERIOD_US   5000UL   // 200 Hz balance loop (IMU read + both PIDs)
#define ARM_DELAY_MS        2000UL   // let the tilt filter converge after boot before motors may run
#define UPRIGHT_ANGLE       5.0f     // deg: robot must be this close to upright to start balancing
#define UPRIGHT_HOLD_MS     300UL    // ...for this long
#define DRIVE_TIMEOUT_MS    1000UL   // throttle/steer fall back to 0 unless refreshed this often

#endif
