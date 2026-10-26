Bailey server
=============

Runs on the robot's Raspberry Pi. Serves the web UI, streams the camera and talks to the Arduino
(which does the balancing) over USB serial. Node.js 20 or newer.

```
npm install
npm start                # real robot on /dev/ttyACM0
npm run sim              # simulated robot, no hardware needed (build it first: ../bailey_FW/test/build.sh)
npm test                 # unit tests + end-to-end tests against the simulator (~45 s)
```

Then open `http://<pi address>:54321`. On the Pi, `install/install.sh` sets everything up as a
systemd service (`journalctl -u bailey -f` for logs), and `install/updater.sh` updates it.

The web UI
----------

* **Drive:** **Motors on** / **Stop**, and a joystick (mouse, touch or arrow keys; Space = stop).
  The robot starts balancing once it's held upright.
* **Live:** pitch, balance angle and wheel speed charts ([uPlot](https://github.com/leeoniya/uPlot):
  20 s, 1 min or 5 min of history, drag to zoom), the control loop time (must stay below the
  control period, 2000 µs),
  the wobble, and the balance angle found by the disturbance observer.
* **Video:** the camera with pitch and heading overlaid. By default it is WebRTC from
  [MediaMTX](https://github.com/bluenviron/mediamtx) (installed by `install/install.sh`,
  configured in `install/mediamtx.yml`): H.264 with a fraction of MJPEG's delay and bandwidth.
  The handshake goes through this server (`/mediamtx/cam/whep`); the video itself uses UDP port
  8189. With `camera.mode` set to `"mjpeg"` the server streams MJPEG from `rpicam-vid` instead,
  only while someone watches; stop MediaMTX first (`sudo systemctl disable --now mediamtx`), since
  only one program can use the camera.
* **Tuning:**
  * **Controller:** LQR (default), MPC or PID, with the LQR gain and pendulum length shown.
  * **Autotune** (about 33 s, while balancing) measures the robot and sets its gains and zero.
  * **Calibrate zero**, **Motor test**, **Save to EEPROM** and **Load defaults**.
  * **Pendulum length**, if you've measured it.
  * The gains, editable in real units.
* **PS4/PS5 controller** (Bluetooth, paired with `install/pair-gamepad.sh`):
  * left stick up/down drives at up to 50 %, or 100 % while R1 is held;
  * right stick steers;
  * ✕ turns the motors on; ○ or PS turns them off.

  The header shows `pad: connected` while it is connected.
* **Events** and a **console** showing the raw serial traffic, where you can type `SCMD` / `READ`
  commands.
* **Raspberry Pi:** temperature, load and addresses; recording telemetry to CSV (in
  `~/bailey-logs`); reboot and shutdown.

Every line on the serial link, both ways, ends in a CRC-8 checksum (`*HH`); lines that were cut
short or run together are dropped (they show in the console marked "bad checksum").

Safety: the browser repeats joystick commands 10 times a second while you drive. If they stop
arriving, the server and then the firmware stop the robot:
* the server stops it after 0.6 s (browser tab hidden, phone locked, Wi-Fi lost);
* the firmware stops it after 1 s if the Pi itself dies.

The controller's motion sensors report hundreds of times a second while it is connected. If they
go quiet for 0.5 s (out of range, flat battery), the server treats the controller as lost and the
robot stops. It doesn't wait for Bluetooth to notice, which can take up to 20 s. When the
controller comes back, the sticks must be moved again before the robot drives.

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
| `camera.mode` | `webrtc` | `webrtc` (MediaMTX) or `mjpeg` |
| `camera.mediamtx` | `http://127.0.0.1:8889` | MediaMTX's WebRTC address (only this server talks to it) |
| `camera.command` | `rpicam-vid` | falls back to `libcamera-vid` (older Raspberry Pi OS) |
| `camera.width/height/fps/quality` | 640×480, 15, 70 | MJPEG only (WebRTC: `install/mediamtx.yml`) |
| `telemetry.logDir` | `~/bailey-logs` | |
| `system.allowShutdown` | true | reboot/shutdown buttons |
| `gamepad.enabled` | true | PS4/PS5 controller |
| `gamepad.maxThrottle` / `maxSteer` | 50, 60 | % at full stick (R1: throttle 100 %) |
| `gamepad.deadZone` | 0.1 | fraction of the stick travel ignored around the centre |

Flags: `--sim`, `--port <n>`, `--serial <device>`, `--no-camera`, `--no-gamepad`.

Code
----

| File | |
| --- | --- |
| `src/server.js` | entry point |
| `src/app.js` | wires everything together; socket.io API documented at the top |
| `src/protocol.js` | the firmware's serial protocol (pure functions, unit tested) |
| `src/robotLink.js` | serial port with automatic reconnect, or the simulator process |
| `src/drive.js` | joystick to Throttle/Steer, with repeat and timeouts |
| `src/webrtc.js` | forwards the WebRTC handshake (WHEP) to MediaMTX |
| `src/camera.js` | `rpicam-vid` MJPEG to browsers (`camera.mode` `"mjpeg"`) |
| `src/telemetryLog.js` | CSV recording |
| `src/system.js` | Pi stats, shutdown/reboot |
| `src/gamepad.js` | PS4/PS5 controller, read from `/dev/input` (no native modules) |
| `public/` | the web UI: plain HTML/CSS/JS, no build step, works without internet |
| `test/` | unit tests and end-to-end tests against the firmware simulator |

What changed from the old server
--------------------------------

The old server (`server/app`, `server/wwwroot`, removed; still in the git history) needed Node 5-era packages:
* socket.io 1, serialport 2;
* node-opencv, which doesn't build on any current Node or Raspberry Pi OS.

It also had these problems:
* It matched serial lines anywhere in the text, so the echo of `SCMD Throttle 50` or the `PIDH`
  header overwrote the telemetry.
* It ran as root from an init.d script.
* It had no reconnect when the Arduino reset.
* Nothing stopped the robot if the browser went away while driving.
