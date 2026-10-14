// A fake Bluetooth controller for tests: a /sys/class/input-like directory
// naming the devices, and named pipes standing in for /dev/input/eventN.
// Start the Gamepad on these directories first (it opens the pipes without
// blocking), then call connect() to start writing.
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { encodeEvent, EV_SYN, EV_KEY, EV_ABS } from '../../src/gamepad.js';

export class FakeGamepad {
  constructor(name = 'Wireless Controller') {
    this.root = fs.mkdtempSync(path.join(os.tmpdir(), 'bailey-pad-'));
    this.sysDir = path.join(this.root, 'sys');
    this.inputDir = path.join(this.root, 'input');
    fs.mkdirSync(this.inputDir);
    this.add('event5', name);
    this.add('event6', `${name} Motion Sensors`);
    this.add('event2', 'Logitech USB Keyboard');   // not a controller
  }

  add(event, name) {
    fs.mkdirSync(path.join(this.sysDir, event, 'device'), { recursive: true });
    fs.writeFileSync(path.join(this.sysDir, event, 'device', 'name'), `${name}\n`);
    fs.writeFileSync(path.join(this.sysDir, event, 'device', 'uniq'), 'a4:ae:12:34:56:78\n');
    execFileSync('mkfifo', [path.join(this.inputDir, event)]);
  }

  // Needs the reader open already.
  connect() {
    const flags = fs.constants.O_WRONLY | fs.constants.O_NONBLOCK;
    this.main = fs.openSync(path.join(this.inputDir, 'event5'), flags);
    this.sensors = fs.openSync(path.join(this.inputDir, 'event6'), flags);
    // The real motion sensors report continuously (~250 Hz).
    this.heartbeat = setInterval(() => fs.writeSync(this.sensors, Buffer.concat([
      encodeEvent(EV_ABS, 0, Math.round(Math.random() * 10)), encodeEvent(EV_SYN, 0, 0)])), 10);
  }

  send(...events) {
    fs.writeSync(this.main, Buffer.concat([...events.map(([t, c, v]) => encodeEvent(t, c, v)), encodeEvent(EV_SYN, 0, 0)]));
  }

  stick(code, value) { this.send([EV_ABS, code, value]); }

  press(code) {
    this.send([EV_KEY, code, 1]);
    this.send([EV_KEY, code, 0]);
  }

  // Out of range: the reports stop, the device stays (Bluetooth hasn't noticed yet).
  silence() {
    clearInterval(this.heartbeat);
    this.heartbeat = null;
  }

  // Bluetooth noticed: the devices go away.
  remove() {
    this.silence();
    fs.rmSync(this.sysDir, { recursive: true, force: true });
  }

  close() {
    this.silence();
    for (const fd of [this.main, this.sensors]) { try { if (fd != null) fs.closeSync(fd); } catch {} }
    fs.rmSync(this.root, { recursive: true, force: true });
  }
}
