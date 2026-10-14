Bailey firmware
===============

Firmware for the Bailey two-wheeled self-balancing robot: an **ESP32-S3** (e.g. the Waveshare
ESP32-S3-Pico) does the balancing and drives two stepper motors, using the MPU-6050 on the IMU
board. A Raspberry Pi runs the Node.js server in `server/` (web UI, video) and talks to it over USB
serial. It is written in C++ (the `.ino` files are C++), with **Eigen** for the matrix maths and
**TinyMPC** for model predictive control.

Firmware 3.0 is ESP32-S3 and steppers only. Eigen and TinyMPC need a C++ standard library, which
the Arduino Mega doesn't have. The Mega and DC motor builds are in the git history (firmware 2.x).

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
| `config.h` | Build-time options: pins, IMU address, loop rate |
| `bailey.h` | `Settings` (everything tunable, stored in EEPROM), `Pid`, prototypes |
| `bailey_FW.ino` | `setup()`, `loop()`, state machine and the balance loop (500 Hz) |
| `imu.ino` | MPU-6050 read + Kalman filter (pitch and gyro bias), any mounting orientation, zero offset |
| `lqr.ino` | LQR controller (Riccati equation solved on the board with Eigen) and disturbance observer |
| `mpc.ino` | Model predictive control with TinyMPC, solved on the second core |
| `src/tinympc/` | TinyMPC solver sources (vendored, MIT; see its README for the small changes) |
| `motors.ino` | Steppers via FastAccelStepper |
| `comms.ino` | Serial commands, telemetry, PID/SYS lines |
| `autotune.ino` | Autotuner: measures pendulum length and IMU offset while balancing |
| `gains.ino` | PID gains from the pendulum length (physical units, scaled by dynamic similarity) |
| `settings.ino` | Defaults, EEPROM load/save with checksum |
| `test/` | Simulator: builds the real firmware against a physics model, for a PC or the browser (not part of the sketch) |

Control loop (500 Hz). There are three controllers, selected with `SCMD controller <n>` (motors
off) or in the web UI:
* **1 = LQR**, the default;
* **2 = MPC**;
* **0 = PID**.

```
LQR:
  gyro + accelerometer -> [Kalman] -> pitch, pitch rate (bias-corrected)
  pitch, rate, wheel acceleration -> [disturbance observer] -> balance angle
  throttle -> speed/position reference (ramped)
  a = -K · [position error, speed error, pitch - balance angle, pitch rate] -> integrate -> stepper speed

PID:
  throttle -> speed setpoint (ramped) -> [speed PID] -> pitch setpoint -> [angle PID] -(+ feedforward)-> motors
```

**The robot model.** All the controllers and the autotuner use

```
pitch'' = (g · pitch − κ · a) / L
```

with `a` the wheels' acceleration (steppers do exactly what they're told), `L` how fast the robot
falls and `κ` how hard the wheels' acceleration tips it. A cart-pole has κ = 1. A real two-wheeler
has more, because the motors' housings are bolted to the body: the wheels turn with the body and
the motors push back on it. For Bailey's geometry this gives L ≈ 0.31 m and κ ≈ 1.4; the autotuner
measures both (`pendulumLength`, `pendulumCoupling`). This was found by checking the simulator against
the **MuJoCo** physics engine (`test/mujoco_check.py`). Firmware 2.x used a plain cart-pole, which
had the robot falling 55 % too fast; worse, its autotuner required two length estimates to agree
within 25 %, and on the real robot they would differ by about 38 %, so it would always have rejected.

**LQR** (`lqr.ino`) uses that model. It weighs position, speed, pitch and pitch rate
together and finds the gain that keeps them small for the least effort. The trade-off is set by the
`lqrMax*` settings: how far each quantity may stray, given for a 0.2 m robot. The discrete Riccati
equation is solved on the board at boot, when a weight changes with the motors off, or when κ
changes, with the structure-preserving doubling algorithm; the matrix algebra (products, LU solves)
is Eigen's. The
model is discretised exactly (cosh/sinh), not by a series. The gain is then rescaled instantly to the
measured pendulum length (exact by dynamic similarity), so autotune can apply it mid-balance. The
solver matches SciPy's `solve_discrete_are` to 1e-4 (`test/`, `lqrsolver`).

The **disturbance observer** compares the gyro with what the model predicts for the commanded
acceleration. It works out the angle the robot actually balances at, and LQR regulates around that:
- a payload or a shifted battery is absorbed;
- an IMU zero error is absorbed;
- a constant push is absorbed;
- the robot holds its position instead of creeping.

In the simulator, a payload shift that made the robot creep 3 m without the observer moves it 7 mm
with it. The estimate is the telemetry field `balAngle`.

**MPC** (`mpc.ino`) uses the same model, state and observer as LQR. Every 30 ms it plans 1.2 s
ahead (40 steps of 30 ms) within the motors' hard limits: acceleration up to `lqrAccelLimit` and
wheel speed up to full speed. The plan is solved by **TinyMPC** (`src/tinympc`), an MPC solver built
for microcontrollers: ADMM on top of a cached Riccati solution, so it stays well conditioned over a
long horizon of an unstable robot in 32-bit floats. The cost is the LQR's.

Solving takes milliseconds, so it runs on the ESP32-S3's second core (core 0) while the balance loop
keeps its 500 Hz on core 1. The plan is handed over as corrections to the LQR (c = u + K·x along the
plan) and the balance loop applies `a = −K·x + c(t)`. A late or failed plan therefore falls back to
plain LQR, never to no control. With no limit in sight c is (nearly) 0 and MPC behaves like LQR.

In the simulator TinyMPC converges in 6-9 iterations per plan, about 40 µs on a PC; expect a few ms
on the ESP32-S3 (well within the 30 ms between plans).

**Honest result:** in the simulator it is never worse than LQR, but it is also no better. Both
survive the same hardest push (limited by the motors' acceleration), and both drive at full
throttle with reversals. What made fast driving safe were two reference changes that apply to
LQR and MPC alike:
* while driving, track speed rather than a position the robot has fallen behind;
* cap the speed target at `MAX_DRIVE_SPEED` (90 %) of full speed, so there is room left to tilt
  back and brake.

MPC is there to experiment with: for example with harder limits (lower `lqrAccelLimit` or
`stepperMaxSpeed`), or as a base for obstacle or slope constraints (TinyMPC also supports linear
and cone constraints).

The **Kalman filter** (`imu.ino`) estimates pitch and gyro bias together. It keeps learning the bias,
which drifts as the chip warms up, so pitch doesn't creep and the pitch rate the controllers use is
bias-free. The steppers' known acceleration is subtracted from the accelerometer first
(`imuAccelCompensation`, on by default), which makes the pitch about 4× more accurate while driving
(0.77° → 0.18° RMS in the simulator) and keeps the bias estimate intact through hard driving.
Whatever acceleration is left, the accelerometer is ignored while the total isn't close to 1 g.

**Libraries checked against the hand-written parts** (`test/`, same recorded or simulated data):

| Part | Tried | Result |
| --- | --- | --- |
| Riccati solver (LQR) | Eigen 3.4, PX4's matrix library (drones), the old hand-written code | Eigen, written with one LU solve per step: 6.3 µs, PX4 7.1 µs, old 9.6 µs (PC, SIMD off as on the ESP32-S3). All agree to 1e-5. |
| MPC | TinyMPC vs the old hand-written FISTA solver | TinyMPC (a published solver for microcontrollers), on the second core. Same results as LQR in the simulator. |
| Pitch filter | TKJ KalmanFilter, Madgwick, Mahony | TKJ is the same algorithm (GPL); Madgwick and Mahony don't estimate the gyro bias and drift off by degrees when it changes. All 0.68-0.87° RMS while driving; the acceleration compensation above gives 0.18°. |
| PID | QuickPID (successor of the Arduino PID library) | Worse: it differentiates the pitch instead of using the gyro, and oscillates at rest (0.18° vs 0.01° RMS). |
| Physics (simulator) | MuJoCo | Found the model error above; the simulator now matches MuJoCo within 0.02°. |

Signs everywhere: pitch > 0 = leaning forward, motor command > 0 = drive forward, wheel speed > 0
= moving forward. Fix the hardware to match with the `*Invert` / `imuAxis` settings, not in code.

State machine: `OFF` (Motors 0) -> `WAIT` (Motors 1, waiting until the robot is held within 5 deg
of upright for 0.3 s) -> `BAL` -> `FALL` (tilt > `fallAngle`, motors cut) -> `WAIT` again once
picked up. Telemetry field `bal` is the state: 0 OFF, 1 WAIT, 2 BAL, 3 FALL.

Board
-----

ESP32-S3: 2 × 240 MHz cores with a single-precision FPU, 3.3 V logic, settings in flash emulating
EEPROM. It talks to the Pi over its own USB port as `/dev/ttyACM0`, which needs **USB CDC On Boot =
Enabled** when compiling; `install/flash-firmware.sh` sets it. It needs no programmer: it uploads
over the same USB cable. If an upload fails (the first time, or after a crash), hold **BOOT**, press
**RESET** (or replug the cable), release BOOT and upload again.

**ESP32-S3-Pico wiring** (GPIO numbers, set in `config.h`):

| Signal | GPIO | Connect to |
| --- | --- | --- |
| Left STEP / DIR | 11 / 12 | left stepper driver STEP / DIR |
| Right STEP / DIR | 13 / 14 | right stepper driver STEP / DIR |
| Stepper ENABLE | 17 | EN of both drivers (active low) |
| I2C SDA / SCL | 15 / 16 | MPU-6050 SDA / SCL |
| 3V3, GND | | MPU-6050 VCC/GND, stepper drivers' logic VDD/GND |

Wiring notes:
* All signals are 3.3 V. DRV8825 and TMC2209 driver inputs and the MPU-6050 accept that
  directly. An A4988 does only when its logic supply (VDD) is 3.3 V: at VDD 5 V it needs 3.5 V
  for a high, so power its VDD from the board's 3V3 (on a Big Easy Driver, set its 3.3 V/5 V
  jumper to 3.3 V). Never connect a 5 V output to the board's pins.
* The motor power (VMOT) stays on the drivers' own supply, with the grounds connected together.
* These GPIOs are safe on any ESP32-S3 with quad PSRAM. The pins avoided are:
  * 0, 3, 45 and 46 (boot mode);
  * 19 and 20 (USB);
  * 26-32 (flash/PSRAM);
  * 21 (the Pico's RGB LED).
* Check the GPIO numbers against the pinout card that comes with the board. If you'd rather use
  other pins, any free GPIO works for every signal; change them in `config.h`.

Libraries
---------

Install from the Arduino Library Manager (versions the firmware was built with):

* **MPU6050** by Electronic Cats, v1.4.5. This is the maintained continuation of Jeff Rowberg's
  i2cdevlib and includes `I2Cdev`. It replaces FreeIMU, which has not been maintained since about
  2014. Balancing needs only the pitch angle and rate, which a Kalman filter on the raw
  gyro and accelerometer provides with less delay than a full AHRS. The magnetometer and
  barometer are not used.
* **FastAccelStepper** by gin66, v2.0.0. Generates step pulses from a hardware timer with
  acceleration ramps, so the steppers never get a speed step they can't follow. v2.0.0 is a
  brand-new major release; if it misbehaves, v1.4.0 has the same API for everything used here.
* **Eigen** by hideakitai, v0.3.2: Eigen 3.4.0, the standard C++ linear algebra library, packaged
  for Arduino (installs ArxTypeTraits with it). Used for the LQR's Riccati solver and by TinyMPC.
* **TinyMPC** is not a Library Manager library: its solver sources are in `src/tinympc` and compile
  with the sketch.
* Wire, EEPROM (bundled with the board core).

Board core: **esp32** by Espressif (Arduino-ESP32 3.x; board "ESP32S3 Dev Module", USB CDC On Boot
Enabled, 16 MB flash, QSPI PSRAM). `install/flash-firmware.sh` installs everything and uploads from
the Pi.

Bring-up (do it in this order)
------------------------------

Send the commands from the web UI console or a serial monitor. Settings changed
with `SCMD` are only kept over a reset after `SCMD SaveCfg 1`.

1. **Turn on debug output:** `SCMD SerialDebug 1` prints a readable status line twice a second.
2. **IMU axis:** tilt the robot forward. `pitch` must follow the tilt and go positive. `imuAxis`
   is the sensor axis the robot pitches around (0 = X, the default; 1 = Y; 2 = Z). Try the others
   if pitch doesn't follow the tilt. If it follows but goes negative, use `SCMD imuInvert 1`.
   Any mounting orientation works.
3. **Zero:** hold the robot at its balance point (where it neither falls forward nor backward)
   and send `SCMD CalibrateZero 1`.
4. **Motors:** lift the wheels off the ground, motors off. `SCMD MotorTest 30` runs
   both wheels at 30 % for one second, then prints `TEST,<left>,<right>`.
   * Both wheels must turn **forward** (the direction the robot leans to when pitch is positive).
     If a wheel turns backward, use `SCMD motorLeftInvert 1` or `SCMD motorRightInvert 1`.
   * `stepperMaxSpeed` (steps/s at 100 %, default 4000 = 2.5 rev/s at 1/8 microstepping)
     and `stepperMaxAccel` must suit your motors and microstepping. If the motors stall or skip at
     `MotorTest 100`, lower them, or raise the driver current. The default gains assume about the
     same wheel speed in m/s, so if you change the microstepping, scale `stepperMaxSpeed` and
     `stepperMaxAccel` by the same factor.
5. **Save:** `SCMD SaveCfg 1`.
6. **Balance:** `SCMD Motors 1`, hold the robot upright and let go once the wheels engage.
7. **Autotune**: `SCMD AUTOTUNE 1` while it balances (below), then `SCMD SaveCfg 1`.
   This also replaces step 3, so a rough zero is enough to get it balancing. Fine-tune with the
   sliders if you like.

Keep the robot still while it boots: the gyro bias is measured then.

If it falls the same way every time, re-check steps 2-4: a sign error makes it run away instantly.
If it oscillates fast: with LQR lower `lqrMaxAccel` (effort becomes more expensive, so the
controller is gentler); with PIDs lower `anglePIDConKp` or `anglePIDConKd`. If it drifts and has to keep
driving to stay up, redo step 3.

Default gains
-------------

**LQR (default).** The settings were chosen in the simulator over robots of 12-30 cm, IMU
offsets of ±2° and up to 4× the expected sensor noise, then checked against gyro drift, payload
shifts and robots of 7-45 cm:
* `lqrMaxPosition` 0.1 m, `lqrMaxSpeed` 0.5 m/s, `lqrMaxAngle` 4.5°, `lqrMaxRate` 10°/s,
  `lqrMaxAccel` 2.7 m/s²;
* `dobBandwidth` 2 rad/s;
* Kalman: `kalmanQAngle` 0.002, `kalmanQBias` 0.003, `kalmanR` 3.

The gain only depends on the robot through L and κ, and the autotuner measures both. Before
autotuning, the defaults (L = 0.30 m, κ = 1.35, from Bailey's geometry) balance robots from 7 cm to
45 cm in the simulator.

**PID.** The gains live in physical units in `gains.ino` and are converted using
`wheelDiameter`, `stepsPerRev` and `stepperMaxSpeed`. Changing those settings recomputes them, so
you can change microstepping or wheels without retuning. They depend on one property of the robot,
its effective pendulum length L: how high the mass sits above the axle, which sets how fast it
falls. Robots that differ only in L behave identically with time scaled by √(L/g), so gains tuned
once carry over exactly. The angle loop's Kp scales with √L, Ki stays constant and Kd scales with
L; the speed loop's Kp scales with 1/√L and Ki with 1/L. κ is divided out of the angle loop and
multiplied into the speed loop.

The reference gains were tuned in the simulator for L = 0.2 m (the speed loop's Ki retuned for the
compensated pitch filter). In the simulator they balance robots from L = 7 cm to 45 cm, and are
better once scaled to the true L. If you know L, set it with `SCMD pendulumLength <mm>`.
Otherwise let the autotuner measure it.

Geometry from `hardware/`: 94 mm wheels, 220 mm body, head with the Pi and camera on top. The
mass distribution isn't in the CAD files, which is why L is measured rather than assumed.

Autotuner
---------

`SCMD AUTOTUNE 1` while the robot balances, with throttle and steer at 0. It takes about 30 s and
**measures** the robot instead of trying gains on it. A robot on stepper wheels obeys

```
pitch'' = (g / L) · (pitch − offset) − (κ / L) · wheel acceleration
```

and everything in that equation is known except L, κ and the IMU offset. The autotuner runs in
three phases:

1. **Baseline (6 s).** It balances on the current settings, scores how much the robot wobbles,
   and measures the gyro's residual bias (balancing in place, the true average rate is 0).
2. **Measure (18 s).** It adds a gentle wobble (two sines, about ±`autotuneAmplitude` = 0.7°) to
   the pitch setpoint and a small independent one (±0.4 m/s²) to the wheels' acceleration (LQR
   and MPC), so pitch and acceleration don't just follow each other. It then fits pitch'' =
   p1·pitch + p2 + p3·acceleration by least squares: **L = g/p1, κ = −p3·L**, offset = −p2/p1.
   Before the fit:
   * pitch is integrated from the gyro, because the accelerometer is fooled by the robot's own
     acceleration;
   * the wheel acceleration is delayed by 5 ms to line up with the MPU-6050's internal filter;
   * every signal gets the same 3 Hz low-pass.
3. **Verify (6 s).** It applies gains for the measured L and κ (LQR gain re-solved, or PID gains)
   and sets the measured offset as the new zero, then scores the wobble again.

The result is **rejected and everything restored** if:
* κ is outside 0.85-2.5 (physically it is a bit above 1: wheels slipping, steps skipped);
* the model explains less than 60% of the motion;
* L is outside 5-80 cm;
* the signs come out wrong (a direction setting is wrong);
* the robot balances clearly worse on the new settings.

Driving, steering, `Motors 0`, `AUTOTUNE 0`, a fall or a tilt error above 6° aborts and restores.
Send `SCMD SaveCfg 1` to keep a good result.

Output: `AT,BASELINE` / `AT,MEASURE` / `AT,VERIFY`, then
`AT,RESULT,<accepted>,<L>,<κ>,<zero>,<R²>,<wobble before>,<wobble after>`
(wobble = mean squared angle error, deg²). The web server logs these and shows them in the UI.

In the simulator (`test/`) it measures L within about 2-7 % and κ within 7 % for robots of 25-45 cm
on Bailey's wheels (smaller robots on smaller wheels: up to 25 %), and the zero within 0.1°. It
rejects slipping wheels.

Commands
--------

`SCMD <name> <value>` (echoed back), `READ <RemoteInit|PIDParamTX|SYSParamTX|printConfig>`.

| Name | Value |
| --- | --- |
| `speedPIDKp`, `speedPIDKi`, `speedPIDKd` | gain x 10000 (UI sliders) |
| `anglePIDConKp/Ki/Kd`, `anglePIDAggKp/Ki/Kd` | gain x 100 |
| `TriggerAngleAggressive`, `calibratedZeroAngle` | deg x 100 |
| `speedPIDMoveFactor`, `speedFilter`, `speedFeedForward`, `autotuneAmplitude` (deg) | value x 100 |
| `anglePIDOutputLimit` (%), `fallAngle` (deg), `speedPIDOutputLimit` (deg), `speedRamp` (%/s), `Maxsteer` (%), `Maxthrottle` (% of full speed), `stepperMaxSpeed` (steps/s), `stepperMaxAccel` (steps/s²), `stepsPerRev` (incl. microstepping), `wheelDiameter` (mm) | plain number |
| `imuAxis` (0/1/2), `imuInvert`, `motorLeftInvert`, `motorRightInvert` | 0/1 |
| `debugSampleRate` (ms), `telemetryPeriod` (ms) | integer |
| `controller` | 1 = LQR + disturbance observer, 2 = MPC, 0 = PID. Change with the motors off |
| `lqrMaxPosition` (m), `lqrMaxSpeed` (m/s), `lqrMaxAngle` (°), `lqrMaxRate` (°/s), `lqrMaxAccel` (m/s²) | LQR weights for a 0.2 m robot; applied when the motors are off |
| `lqrAccelLimit` (m/s²), `dobBandwidth` (rad/s for a 0.2 m robot, 0 = observer off) | plain number |
| `kalmanQAngle`, `kalmanQBias`, `kalmanR` | Kalman filter noise settings, plain numbers |
| `Motors`, `AUTOTUNE`, `SerialDebug` | 0/1 |
| `Throttle`, `Steer` | -100..100. Not echoed. A non-zero value must be repeated at least every second (the web server sends them at 10 Hz); otherwise both drop back to 0, so a lost connection can't leave the robot driving |
| `pendulumCoupling` | κ × 100 (autotune measures it) |
| `imuAccelCompensation` | 0/1 |
| `CalibrateZero 1`, `MotorTest <-100..100>`, `pendulumLength <mm>` (gains for that length), `SaveCfg 1`, `Load_def 1`, `E_EEPROM 1`, `resetPID 1` | actions |

Lines from the robot start with a tag: `T`/`TH` telemetry, `PID`/`PIDH`, `SYS`/`SYSH`, `EV` events,
`AT` autotune progress/result, `LQR` gain (`LQR,k_pos,k_speed,k_pitch,k_rate,L`, also with
`READ LQRParamTX`), `TEST` motor test, `***` config dump, `E` errors. Telemetry includes `balAngle`,
the disturbance observer's balance angle.

Simulator
---------

`test/run_tests.sh` (needs g++ and Eigen: `apt install libeigen3-dev`) compiles the firmware,
including TinyMPC, against a model of the robot and checks balancing, pushes, driving, fall
detection and recovery, the autotuner, the LQR solver against SciPy, the motor limits, the command
parser, telemetry format and EEPROM handling, with each controller. `test/build.sh` only builds it;
`SIM_TRACE=1 test/sim balance` prints the control loop as it runs.

`test/sim live` runs the simulated robot in real time and speaks the serial protocol on
stdin/stdout. The web server uses it with `npm run sim` (in `server/`), so the whole system can be
tried without hardware: a simulated person holds the robot up until it balances and picks it up
again after a fall.

`test/mujoco_check.py` (needs `pip install mujoco numpy`) checks the simulator's robot model against
the MuJoCo physics engine.

`test/build_mujoco.sh` (needs `pip install mujoco`) builds `test/sim_mujoco`: the same firmware and
scenarios, with the robot simulated by MuJoCo instead of the built-in equations. It is a 3D robot
whose wheels roll on the floor with friction (and can slip), whose body can pitch, roll, turn and
fall over, and whose steppers are stiff servos following the step rate. `run_tests.sh` runs the LQR
and MPC scenarios and the autotuner on it when MuJoCo is installed. The PID controller is left out:
it balances, but rests with a small (~0.2°) wobble on this plant and misses the stopping test.
The `scaling` scenario also runs only on the built-in plant: its smallest robot (7 cm, 13 mm wheels)
doesn't balance on MuJoCo's contact model; 0.2-0.45 m robots do.

`test/build_wasm.sh` builds the same simulator as WebAssembly for `test/web/index.html`, an
interactive page with a side view of the robot, driving, pushes, payload, controller choice and the
serial port (serve the folder over HTTP to open it).
