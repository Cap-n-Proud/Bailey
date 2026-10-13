Bailey firmware
===============

Firmware for the Bailey two-wheeled self-balancing robot: an Arduino Mega 2560 does the balancing
and drives the motors (steppers by default), using the MPU-6050 on the IMU board. A Raspberry Pi
runs the Node.js server in `server/` (web UI, video) and talks to the Arduino over USB serial at 115200 baud.

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
| `imu.ino` | MPU-6050 read + Kalman filter (pitch and gyro bias), any mounting orientation, zero offset |
| `lqr.ino` | LQR controller (Riccati equation solved on the Arduino) and disturbance observer |
| `motors.ino` | Steppers via FastAccelStepper, or DC motors + encoders |
| `comms.ino` | Serial commands, telemetry, PID/SYS lines |
| `autotune.ino` | Autotuner: measures pendulum length and IMU offset while balancing |
| `gains.ino` | PID gains from the pendulum length (physical units, scaled by dynamic similarity) |
| `settings.ino` | Defaults, EEPROM load/save with checksum |
| `test/` | PC simulator: builds the real firmware against a physics model (not part of the sketch) |

Control loop (200 Hz). There are two controllers, selected with `SCMD controller <1|0>`. Steppers
use **LQR** by default; DC motors always use the **PIDs**.

```
LQR (steppers):
  gyro + accelerometer -> [Kalman] -> pitch, pitch rate (bias-corrected)
  pitch, rate, wheel acceleration -> [disturbance observer] -> balance angle
  throttle -> speed/position reference (ramped)
  a = -K · [position error, speed error, pitch - balance angle, pitch rate] -> integrate -> stepper speed

PID (both):
  throttle -> speed setpoint (ramped) -> [speed PID] -> pitch setpoint -> [angle PID] -(+ feedforward)-> motors
```

**LQR** (`lqr.ino`) treats the robot as an inverted pendulum on wheels whose acceleration it commands
directly; steppers do exactly what they're told. It weighs position, speed, pitch and pitch rate
together and finds the gain that keeps them small for the least effort. The trade-off is set by the
`lqrMax*` settings: how far each quantity may stray, given for a 0.2 m robot. The Riccati equation is
solved on the Arduino at boot, or when a weight changes with the motors off; it takes about 0.1 s of
float math. The gain is then rescaled instantly to the measured pendulum length (exact by dynamic
similarity), so autotune can apply it mid-balance. The solver matches SciPy to 1e-4 (`test/`, `lqrsolver`).

The **disturbance observer** compares the gyro with what the model predicts for the commanded
acceleration. It works out the angle the robot actually balances at, and LQR regulates around that:
- a payload or a shifted battery is absorbed;
- an IMU zero error is absorbed;
- a constant push is absorbed;
- the robot holds its position instead of creeping.

In the simulator, a payload shift that made the robot creep 3 m without the observer moves it 7 mm
with it. The estimate is the telemetry field `balAngle`.

The **Kalman filter** (`imu.ino`) estimates pitch and gyro bias together. It keeps learning the bias,
which drifts as the chip warms up, so pitch doesn't creep and the pitch rate the controllers use is
bias-free. The accelerometer is ignored while the robot accelerates hard, when its reading isn't
gravity.

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
  2014. Balancing needs only the pitch angle and rate, which a Kalman filter on the raw
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

Send the commands from the web UI console or a serial monitor at 115200 baud. Settings changed
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
7. **Autotune** (steppers): `SCMD AUTOTUNE 1` while it balances (below), then `SCMD SaveCfg 1`.
   This also replaces step 3, so a rough zero is enough to get it balancing. Fine-tune with the
   sliders if you like.

Keep the robot still while it boots: the gyro bias is measured then.

If it falls the same way every time, re-check steps 2-4: a sign error makes it run away instantly.
If it oscillates fast: with LQR lower `lqrMaxAccel` (effort becomes more expensive, so the
controller is gentler); with PIDs lower `anglePIDConKp` or `anglePIDConKd`. If it drifts and has to keep
driving to stay up, redo step 3.

Default gains
-------------

**Steppers, LQR (default).** The settings were chosen in the simulator over robots of 12-30 cm, IMU
offsets of ±2° and up to 4× the expected sensor noise, then checked against gyro drift, payload
shifts and robots of 7-45 cm:
* `lqrMaxPosition` 0.1 m, `lqrMaxSpeed` 0.5 m/s, `lqrMaxAngle` 4.5°, `lqrMaxRate` 10°/s,
  `lqrMaxAccel` 2.7 m/s²;
* `dobBandwidth` 2 rad/s;
* Kalman: `kalmanQAngle` 0.002, `kalmanQBias` 0.003, `kalmanR` 3.

The gain only depends on the robot through L, and the autotuner measures L. Before autotuning, the
defaults (for L = 0.2 m) balance robots from 7 cm to 45 cm in the simulator.

**Steppers, PID.** The gains live in physical units in `gains.ino` and are converted using
`wheelDiameter`, `stepsPerRev` and `stepperMaxSpeed`. Changing those settings recomputes them, so
you can change microstepping or wheels without retuning. They depend on one property of the robot,
its effective pendulum length L: how high the mass sits above the axle, which sets how fast it
falls. Robots that differ only in L behave identically with time scaled by √(L/g), so gains tuned
once carry over exactly. The angle loop's Kp scales with √L, Ki stays constant and Kd scales with
L; the speed loop's Kp scales with 1/√L and Ki with 1/L.

The reference gains were tuned in the simulator for L = 0.2 m and are the defaults until the robot
has been measured. In the simulator they balance robots from L = 7 cm to 45 cm, but are noticeably
better once scaled to the true L. If you know L, set it with `SCMD pendulumLength <mm>`.
Otherwise let the autotuner measure it.

**DC motors.** Fixed defaults, checked in the simulator over a range of body heights, IMU offsets,
noise and motor response times (40-150 ms).

Geometry from `hardware/`: 94 mm wheels, 220 mm body, head with the Pi and camera on top. The
mass distribution isn't in the CAD files, which is why L is measured rather than assumed.

Autotuner (steppers)
--------------------

`SCMD AUTOTUNE 1` while the robot balances, with throttle and steer at 0. It takes about 30 s and
**measures** the robot instead of trying gains on it. A robot on stepper wheels obeys

```
pitch'' = (g / L) · (pitch − offset) − (1 / L) · wheel acceleration
```

and everything in that equation is known except L and the IMU offset. The autotuner runs in
three phases:

1. **Baseline (6 s).** It balances on the current settings, scores how much the robot wobbles,
   and measures the gyro's residual bias (balancing in place, the true average rate is 0).
2. **Measure (18 s).** It adds a gentle wobble (two sines, about ±`autotuneAmplitude` = 0.7°) to
   the pitch setpoint, so there is motion to measure. It then fits pitch'' = p1·pitch + p2 +
   p3·acceleration by least squares. That gives **two independent estimates of L**, g/p1 and
   −1/p3, plus the offset, −p2/p1. Before the fit:
   * pitch is integrated from the gyro, because the accelerometer is fooled by the robot's own
     acceleration;
   * the wheel acceleration is delayed by 5 ms to line up with the MPU-6050's internal filter;
   * every signal gets the same 3 Hz low-pass.
3. **Verify (6 s).** It applies gains for the measured L (LQR gain rescaled, or PID gains) and sets the measured offset as the new
   zero, then scores the wobble again.

The result is **rejected and everything restored** if:
* the two L estimates disagree by more than 25% (wheels slipping, steps skipped);
* the model explains less than 60% of the motion;
* L is outside 5-60 cm;
* the signs come out wrong (a direction setting is wrong);
* the robot balances clearly worse on the new settings.

Driving, steering, `Motors 0`, `AUTOTUNE 0`, a fall or a tilt error above 6° aborts and restores.
Send `SCMD SaveCfg 1` to keep a good result.

Output: `AT,BASELINE` / `AT,MEASURE` / `AT,VERIFY`, then
`AT,RESULT,<accepted>,<L>,<L from g/p1>,<L from −1/p3>,<zero>,<R²>,<wobble before>,<wobble after>`
(wobble = mean squared angle error, deg²). The web server logs these and shows them in the UI.

In the simulator (`test/`) it measures L within about 1-5% for robots of 12-45 cm (about 15% at
8 cm) and the zero within 0.2°. It rejects slipping wheels. With four times the expected IMU noise,
L comes out about 20% high, and the resulting gains still balance well.

DC motors: not supported. The wheel acceleration from the encoders isn't clean enough for the fit.

Commands
--------

`SCMD <name> <value>` (echoed back), `READ <RemoteInit|PIDParamTX|SYSParamTX|printConfig>`.

| Name | Value |
| --- | --- |
| `speedPIDKp`, `speedPIDKi`, `speedPIDKd` | gain x 10000 (UI sliders) |
| `anglePIDConKp/Ki/Kd`, `anglePIDAggKp/Ki/Kd` | gain x 100 |
| `TriggerAngleAggressive`, `calibratedZeroAngle` | deg x 100 |
| `speedPIDMoveFactor`, `speedFilter`, `speedFeedForward`, `autotuneAmplitude` (deg) | value x 100 |
| `anglePIDOutputLimit` (%), `fallAngle` (deg), `speedPIDOutputLimit` (deg), `speedRamp` (%/s), `Maxsteer` (%), `Maxthrottle` (% of full speed), `encoderCountsPerSecFull`, `encoderCountsPerRev`, `stepperMaxSpeed` (steps/s), `stepperMaxAccel` (steps/s²), `stepsPerRev` (incl. microstepping), `wheelDiameter` (mm) | plain number |
| `motorLeftMinimumSpeed`, `motorRightMinimumSpeed` | PWM 0-254 |
| `imuAxis` (1/2), `imuInvert`, `motorLeftInvert`, `motorRightInvert`, `encoderLeftInvert`, `encoderRightInvert` | 0/1 |
| `debugSampleRate` (ms), `telemetryPeriod` (ms) | integer |
| `controller` | 1 = LQR + disturbance observer (steppers), 0 = PID. Change with the motors off |
| `lqrMaxPosition` (m), `lqrMaxSpeed` (m/s), `lqrMaxAngle` (°), `lqrMaxRate` (°/s), `lqrMaxAccel` (m/s²) | LQR weights for a 0.2 m robot; applied when the motors are off |
| `lqrAccelLimit` (m/s²), `dobBandwidth` (rad/s for a 0.2 m robot, 0 = observer off) | plain number |
| `kalmanQAngle`, `kalmanQBias`, `kalmanR` | Kalman filter noise settings, plain numbers |
| `Motors`, `AUTOTUNE`, `SerialDebug` | 0/1 |
| `Throttle`, `Steer` | -100..100. Not echoed. A non-zero value must be repeated at least every second (the web server sends them at 10 Hz); otherwise both drop back to 0, so a lost connection can't leave the robot driving |
| `CalibrateZero 1`, `MotorTest <-100..100>`, `pendulumLength <mm>` (steppers: gains for that length), `SaveCfg 1`, `Load_def 1`, `E_EEPROM 1`, `resetPID 1` | actions |

Lines from the robot start with a tag: `T`/`TH` telemetry, `PID`/`PIDH`, `SYS`/`SYSH`, `EV` events,
`AT` autotune progress/result, `LQR` gain (`LQR,k_pos,k_speed,k_pitch,k_rate,L`, also with
`READ LQRParamTX`), `TEST` motor test, `***` config dump, `E` errors. Telemetry includes `balAngle`,
the disturbance observer's balance angle.

Simulator
---------

`test/run_tests.sh` (needs g++) compiles the firmware for both motor types against a model of the
robot and checks balancing, pushes, driving, fall detection and recovery, the autotuner, the
command parser, telemetry format and EEPROM handling. `SIM_TRACE=1 test/sim_dc balance` prints the
control loop as it runs.

`test/sim_stepper live` runs the simulated robot in real time and speaks the serial protocol on
stdin/stdout. The web server uses it with `npm run sim` (in `server/`), so the whole system can be
tried without hardware: a simulated person holds the robot up until it balances and picks it up
again after a fall.
