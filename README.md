Bailey
======

A two-wheeled self-balancing robot with a camera, driven from a web browser.

```
 browser (phone/laptop) ──Wi-Fi──  Raspberry Pi  ──USB──  ESP32-S3 ── stepper drivers, MPU-6050
   web UI, joystick,              server/: web UI,       bailey_FW/: balancing at 500 Hz,
   video, tuning                  video, telemetry log   LQR, MPC, Kalman, autotune, safety
```

The microcontroller does everything time-critical, so the robot keeps balancing if the Pi or the
network hiccups. The Pi is the user interface.

| Folder | |
| --- | --- |
| [`bailey_FW/`](bailey_FW/README.md) | Firmware (ESP32-S3, C++ with Eigen and TinyMPC), plus a simulator of the robot that runs the real firmware on a PC or in the browser (`bailey_FW/test`) |
| [`server/`](server/README.md) | Raspberry Pi server and web UI (Node.js) |
| `install/` | Pi setup: `install.sh` (service), `updater.sh`, `flash-firmware.sh` (upload the firmware from the Pi), `pair-gamepad.sh` (PS4/PS5 controller) |
| `hardware/` | OpenSCAD parts, schematics, photos, links to third-party parts (`hardware/README.md`) |

Getting started
---------------

**Without hardware.** Simulate the robot and drive it in the browser:

```
sudo apt install g++ libeigen3-dev      # once (Debian/Ubuntu; Eigen is the firmware's matrix library)
bailey_FW/test/run_tests.sh             # builds the simulator and runs the firmware tests
cd server && npm install && npm run sim # then open http://localhost:54321
```

**On the robot:**

1. On the Pi: `git clone https://github.com/Cap-n-Proud/Bailey.git && cd Bailey && ./install/install.sh`
2. Wire the board as in `bailey_FW/README.md`, then flash it from the Pi:
   `./install/flash-firmware.sh`.
   You can also use the Arduino IDE; see `bailey_FW/README.md` for the board settings and libraries.
3. Open `http://<pi address>:54321`. Follow the bring-up steps in `bailey_FW/README.md`: IMU axis,
   motor directions with **Motor test**, then **Motors on** while holding the robot upright, then
   **Autotune**, then **Save to EEPROM**.
4. Optional: drive with a PS4 or PS5 controller over Bluetooth. Hold SHARE + PS until the light
   bar double-flashes, then run `./install/pair-gamepad.sh` on the Pi (once). See
   `server/README.md` for the buttons.
