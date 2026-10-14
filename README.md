Bailey
======

A two-wheeled self-balancing robot with a camera, driven from a web browser.

```
 browser (phone/laptop) ──Wi-Fi──  Raspberry Pi  ──USB──  ESP32-S3 (or Arduino Mega) ── stepper drivers, MPU-6050
   web UI, joystick,              server/: web UI,       bailey_FW/: balancing at 500 Hz (Mega: 200 Hz),
   video, tuning                  video, telemetry log   LQR, Kalman, autotune, safety
```

The microcontroller does everything time-critical, so the robot keeps balancing if the Pi or the
network hiccups. The Pi is the user interface.

| Folder | |
| --- | --- |
| [`bailey_FW/`](bailey_FW/README.md) | Firmware (ESP32-S3 or Arduino Mega), plus a PC simulator of the robot that runs the real firmware (`bailey_FW/test`) |
| [`server/`](server/README.md) | Raspberry Pi server and web UI (Node.js) |
| `install/` | Pi setup: `install.sh` (service), `updater.sh`, `flash-firmware.sh` (upload the firmware from the Pi) |
| `hardware/` | OpenSCAD parts, schematics, photos |

Getting started
---------------

**Without hardware.** Simulate the robot and drive it in the browser:

```
bailey_FW/test/run_tests.sh             # builds the simulator and runs the firmware tests
cd server && npm install && npm run sim # then open http://localhost:54321
```

**On the robot:**

1. On the Pi: `git clone https://github.com/Cap-n-Proud/Bailey.git && cd Bailey && ./install/install.sh`
2. Wire the board as in `bailey_FW/README.md`, then flash it from the Pi:
   `./install/flash-firmware.sh` (ESP32-S3, the default) or `./install/flash-firmware.sh mega`.
   You can also use the Arduino IDE; see `bailey_FW/README.md` for the board settings and libraries.
3. Open `http://<pi address>:54321`. Follow the bring-up steps in `bailey_FW/README.md`: IMU axis,
   motor directions with **Motor test**, then **Motors on** while holding the robot upright, then
   **Autotune**, then **Save to EEPROM**.
