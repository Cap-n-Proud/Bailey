Bailey firmware
===============

Firmware for the Bailey two-wheeled self-balancing robot: an Arduino Mega 2560 does the balancing
and drives the motors (steppers by default), using the MPU-6050 on the IMU board. A Raspberry Pi
runs the Node.js server in `server/app` (web UI, video) and talks to the Arduino over USB serial.

Version 2.0 is a rewrite. Why the previous firmware never balanced:

* The motor commands were commented out and the motor/encoder code had been deleted.
* `anglePIDInput` was never assigned from the IMU and `speedPIDInput` never from the encoders,
  so both PIDs always saw zero. The IMU was only read every 100 ms.
* `double UserControl[1]` was indexed with `[1]`: the throttle read/wrote past the array.
* All `TimedAction`s were built from `configuration` before it was loaded, so every interval was
  0 and the wheel speed was "encoder counts per loop pass", which changed with serial traffic.
* `SCMD TriggerAngleAggressive` overwrote `anglePIDConKd`; `SCMD anglePIDSampling` overwrote
  `anglePIDOutputLowerLimit`.
* Throttle was multiplied by `Maxthrottle` twice.
* The autotuner used `LoopT/1000` (integer division, always 0) as its fitness, its search range was
  computed from an unloaded configuration (all zeros), and its particle arrays were declared with 2
  elements but indexed with 3.
* `setup()` always overwrote the EEPROM with defaults, so saved tuning never survived a reset.
  The stored struct also contained a `String` (a heap pointer).
* After falling, `SetTunings(0,0,0)` left the PID library's integral term driving the motors.

Structure
---------

| File | What |
| --- | --- |
| `config.h` | Build-time options: motor type (`MOTOR_STEPPER` default, or `MOTOR_DC`), pins, IMU address, loop rate |
| `bailey.h` | `Settings` (everything tunable, stored in EEPROM), `Pid`, prototypes |
| `bailey_FW.ino` | `setup()`, `loop()`, state machine and the 200 Hz balance loop |
| `imu.ino` | MPU-6050 read + complementary filter, any mounting orientation, zero offset |
| `motors.ino` | Steppers via FastAccelStepper, or DC motors + encoders |
| `comms.ino` | Serial commands, telemetry, PID/SYS lines |
| `autotune.ino` | Particle swarm autotuner for the angle PID |
| `settings.ino` | Defaults, EEPROM load/save with checksum |
| `test/` | PC simulator: builds the real firmware against a physics model (not part of the sketch) |

Control loop (200 Hz):

```
throttle -> speed setpoint (ramped) -> [speed PID] -> pitch setpoint -> [angle PID] -(+ feedforward)-> motors
                                          ^ wheel speed (encoders)        ^ pitch (IMU)
```

Signs everywhere: pitch > 0 = leaning forward, motor command > 0 = drive forward, wheel speed > 0
= moving forward. Fix the hardware to match with the `*Invert` / `imuAxis` settings, not in code.

State machine: `OFF` (Motors 0) -> `WAIT` (Motors 1, waiting until the robot is held within 5 deg
of upright for 0.3 s) -> `BAL` -> `FALL` (tilt > `fallAngle`, motors cut) -> `WAIT` again once
picked up. Telemetry field `bal` is the state: 0 OFF, 1 WAIT, 2 BAL, 3 FALL.

Libraries
---------

Install from the Arduino Library Manager (versions the firmware was built with):

* **MPU6050** by Electronic Cats, v1.4.5. This is the maintained continuation of Jeff Rowberg's
  i2cdevlib and includes `I2Cdev`. It replaces FreeIMU, which has not been maintained since about
  2014. Balancing needs only the pitch angle and rate, which a complementary filter on the raw
  gyro and accelerometer provides with less delay than a full AHRS. The magnetometer and
  barometer are not used.
* **FastAccelStepper** by gin66, v2.0.0 (steppers). Generates step pulses from a hardware timer
  with acceleration ramps, so the steppers never get a speed step they can't follow. On the Mega
  the STEP pins must be Timer 4 outputs: pins 6, 7 or 8. v2.0.0 is a brand-new major release; if
  it misbehaves, v1.4.0 has the same API for everything used here.
* **Encoder** by Paul Stoffregen, v1.4.4 (DC motors only; the current git master does not compile
  for AVR).
* Wire, EEPROM (bundled with the Arduino AVR core).

Wiring for steppers is in `config.h`: STEP 6 (left) and 7 (right), DIR 9 and 10, shared active-low
ENABLE on 12, MPU-6050 on SDA/SCL (20/21 on the Mega). For DC motors, set `MOTOR_TYPE` to
`MOTOR_DC`.

Bring-up (do it in this order)
------------------------------

Send the commands from the web UI command box or a serial monitor at 38400 baud. Settings changed
with `SCMD` are only kept over a reset after `SCMD SaveCfg 1`.

1. **Turn on debug output:** `SCMD SerialDebug 1` prints a readable status line twice a second.
2. **IMU axis:** tilt the robot forward. `pitch` must follow the tilt and go positive. `imuAxis`
   is the sensor axis the robot pitches around (0 = X, the default; 1 = Y; 2 = Z). Try the others
   if pitch doesn't follow the tilt. If it follows but goes negative, use `SCMD imuInvert 1`.
   Any mounting orientation works.
3. **Zero:** hold the robot at its balance point (where it neither falls forward nor backward)
   and send `SCMD CalibrateZero 1`.
4. **Motors and encoders:** lift the wheels off the ground, motors off. `SCMD MotorTest 30` runs
   both wheels at 30 % for one second, then prints `TEST,<left>,<right>`.
   * Both wheels must turn **forward** (the direction the robot leans to when pitch is positive).
     If a wheel turns backward, use `SCMD motorLeftInvert 1` or `SCMD motorRightInvert 1`.
   * Both reported speeds must be **positive**. If one is negative, use `SCMD encoderLeftInvert 1`
     or `SCMD encoderRightInvert 1`.
   * Steppers: `stepperMaxSpeed` (steps/s at 100 %, default 4000 = 2.5 rev/s at 1/8 microstepping)
     and `stepperMaxAccel` must suit your motors and microstepping. If the motors stall or skip at
     `MotorTest 100`, lower them, or raise the driver current. The default gains assume about the
     same wheel speed in m/s, so if you change the microstepping, scale `stepperMaxSpeed` and
     `stepperMaxAccel` by the same factor.
   * DC: at `SCMD MotorTest 100` the speeds should be about 100. If not, set
     `SCMD encoderCountsPerSecFull <counts/s>`. Lower `motorLeftMinimumSpeed` /
     `motorRightMinimumSpeed` (PWM 0-255) to the value where each wheel just starts turning.
5. **Save:** `SCMD SaveCfg 1`.
6. **Balance:** `SCMD Motors 1`, hold the robot upright and let go once the wheels engage.
7. **Tune** with the sliders, or `SCMD AUTOTUNE 1` while it balances (below), then `SCMD SaveCfg 1`.

Keep the robot still while it boots: the gyro bias is measured then.

If it falls the same way every time, re-check steps 2-4: a sign error makes it run away instantly.
If it oscillates fast, lower `anglePIDConKp` or `anglePIDConKd`. If it drifts and has to keep
driving to stay up, redo step 3.

Default gains
-------------

There are separate defaults for steppers and DC motors. They started from an LQR design and were
then tuned in the simulator. Each set was checked across a range of assumptions, because the robot's
mass distribution isn't in the CAD files: centre of mass 12-30 cm above the axle, tilt offsets of
±2°, IMU noise, and (DC) motor response times of 40-150 ms. Geometry from `hardware/`: 94 mm
wheels, 220 mm body, head with the Pi and camera on top. Treat them as a starting point, not a
final tuning.

Autotuner
---------

`SCMD AUTOTUNE 1` (robot balancing) runs a particle swarm search over the conservative angle gains
(Kp, Ki, Kd) within ±`SPOspread` of the current values. Each candidate gets `SPOSettle` ms to
settle, then is scored for `SPOConfigEval` ms on mean squared angle error plus a small motor
effort term. `numParticles` x `maxIterations` candidates are tried (8 x 10 x 3.5 s = about 5 min
by default). Progress lines: `SPO,<iteration>,<particle>,<score>,<best score>,<Kp>,<Ki>,<Kd>`.

When it finishes, or on `SCMD AUTOTUNE 0`, the best gains are applied (send `SCMD SaveCfg 1` to
keep them). If the robot falls, that candidate scores as bad, the best gains so far are restored,
and the search continues once you stand the robot up again. After 3 falls it stops. `SCMD Motors 0`
aborts and restores the gains from before the run.

Commands
--------

`SCMD <name> <value>` (echoed back), `READ <RemoteInit|PIDParamTX|SYSParamTX|printConfig>`.

| Name | Value |
| --- | --- |
| `speedPIDKp`, `speedPIDKi`, `speedPIDKd` | gain x 10000 (UI sliders) |
| `anglePIDConKp/Ki/Kd`, `anglePIDAggKp/Ki/Kd` | gain x 100 |
| `TriggerAngleAggressive`, `calibratedZeroAngle` | deg x 100 |
| `speedPIDMoveFactor`, `speedFilter`, `speedFeedForward`, `SPOspread` | fraction x 100 |
| `anglePIDOutputLimit` (%), `fallAngle` (deg), `speedPIDOutputLimit` (deg), `speedRamp` (%/s), `Maxsteer` (%), `Maxthrottle` (% of full speed), `encoderCountsPerSecFull`, `stepperMaxSpeed` (steps/s), `stepperMaxAccel` (steps/s²) | plain number |
| `motorLeftMinimumSpeed`, `motorRightMinimumSpeed` | PWM 0-254 |
| `imuAxis` (1/2), `imuInvert`, `motorLeftInvert`, `motorRightInvert`, `encoderLeftInvert`, `encoderRightInvert` | 0/1 |
| `numParticles`, `maxIterations`, `SPOConfigEval` (ms), `SPOSettle` (ms), `debugSampleRate` (ms), `telemetryPeriod` (ms) | integer |
| `Motors`, `AUTOTUNE`, `SerialDebug` | 0/1 |
| `Throttle`, `Steer` | -100..100 |
| `CalibrateZero 1`, `MotorTest <-100..100>`, `SaveCfg 1`, `Load_def 1`, `E_EEPROM 1`, `resetPID 1` | actions |

Lines from the robot start with a tag: `T`/`TH` telemetry, `PID`/`PIDH`, `SYS`/`SYSH`, `EV` events,
`SPO` autotune progress, `TEST` motor test, `***` config dump, `E` errors.

Simulator
---------

`test/run_tests.sh` (needs g++) compiles the firmware for both motor types against a model of the
robot and checks balancing, pushes, driving, fall detection and recovery, the autotuner, the
command parser, telemetry format and EEPROM handling. `SIM_TRACE=1 test/sim_dc balance` prints the
control loop as it runs.
