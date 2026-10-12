Bailey server
=============

Runs on the robot's Raspberry Pi. Serves the web UI, streams the camera and talks to the Arduino
(which does the balancing) over USB serial. Node.js 20 or newer.

```
npm install
npm start                # real robot on /dev/ttyACM0
npm run sim              # simulated robot, no hardware needed (build it first: ../bailey_FW/test/run_tests.sh)
npm test                 # unit tests + end-to-end tests against the simulator (~45 s)
```

Then open `http://<pi address>:54321`. On the Pi, `install/install.sh` sets everything up as a
systemd service (`journalctl -u bailey -f` for logs), and `install/updater.sh` updates it.

The web UI
----------

* **Drive:** **Motors on** / **Stop**, and a joystick (mouse, touch or arrow keys; Space = stop).
  The robot starts balancing once it's held upright.
* **Live:** pitch and wheel speed charts (20 s), the control loop time (must stay below 5000 µs)
  and the wobble.
* **Video:** a camera stream with pitch and heading overlaid. The camera only runs while someone
  is watching.
* **Tuning:**
  * **Autotune** (about 30 s, while balancing) measures the robot and sets its gains and zero.
  * **Calibrate zero**, **Motor test**, **Save to EEPROM** and **Load defaults**.
  * **Pendulum length**, if you've measured it.
  * The gains, editable in real units.
* **Events** and a **console** showing the raw serial traffic, where you can type `SCMD` / `READ`
  commands.
* **Raspberry Pi:** temperature, load and addresses; recording telemetry to CSV (in
  `~/bailey-logs`); reboot and shutdown.

Safety: the browser repeats joystick commands 10 times a second while you drive. If they stop
arriving, the server and then the firmware stop the robot:
* the server stops it after 0.6 s (browser tab hidden, phone locked, Wi-Fi lost);
* the firmware stops it after 1 s if the Pi itself dies.

There is no login. Anyone on the robot's network can drive it, so keep it on a network you trust.

Configuration
-------------

`config.json` holds the defaults. Put per-robot changes in `config.local.json` (same structure,
not in git), for example:

```json
{ "serial": { "port": "/dev/ttyUSB0" }, "camera": { "width": 1280, "height": 720 } }
```

| Setting | Default | |
| --- | --- | --- |
| `http.port` | 54321 | |
| `serial.port` | `/dev/ttyACM0` | `sim` = simulator |
| `serial.baudRate` | 115200 | must match `SERIAL_BAUD` in the firmware |
| `camera.enabled` | true | |
| `camera.command` | `rpicam-vid` | falls back to `libcamera-vid` (older Raspberry Pi OS) |
| `camera.width/height/fps/quality` | 640×480, 15, 70 | MJPEG |
| `telemetry.logDir` | `~/bailey-logs` | |
| `system.allowShutdown` | true | reboot/shutdown buttons |

Flags: `--sim`, `--port <n>`, `--serial <device>`, `--no-camera`.

Code
----

| File | |
| --- | --- |
| `src/server.js` | entry point |
| `src/app.js` | wires everything together; socket.io API documented at the top |
| `src/protocol.js` | the firmware's serial protocol (pure functions, unit tested) |
| `src/robotLink.js` | serial port with automatic reconnect, or the simulator process |
| `src/drive.js` | joystick to Throttle/Steer, with repeat and timeouts |
| `src/camera.js` | `rpicam-vid` MJPEG to browsers |
| `src/telemetryLog.js` | CSV recording |
| `src/system.js` | Pi stats, shutdown/reboot |
| `public/` | the web UI: plain HTML/CSS/JS, no build step, works without internet |
| `test/` | unit tests and end-to-end tests against the firmware simulator |

What changed from the old server
--------------------------------

The old server (`server/app`, `server/wwwroot`) needed Node 5-era packages:
* socket.io 1, serialport 2;
* node-opencv, which doesn't build on any current Node or Raspberry Pi OS.

It also had these problems:
* It matched serial lines anywhere in the text, so the echo of `SCMD Throttle 50` or the `PIDH`
  header overwrote the telemetry.
* It ran as root from an init.d script.
* It had no reconnect when the Arduino reset.
* Nothing stopped the robot if the browser went away while driving.
