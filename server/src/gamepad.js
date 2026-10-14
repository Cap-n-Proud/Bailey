// PS4 (DualShock 4) or PS5 (DualSense) controller over Bluetooth.
//
// Once paired (install/pair-gamepad.sh), the Linux driver (hid-playstation, or
// hid-sony on older kernels) turns the controller into input devices:
//   /dev/input/eventN  "Wireless Controller"                 sticks and buttons
//   /dev/input/eventM  "Wireless Controller Motion Sensors"  accelerometer/gyro
// They are read directly (struct input_event), so no native module is needed.
//
// Controls
//   left stick up/down    throttle      (maxThrottle %, R1 held: 100 %)
//   right stick left/right steer        (maxSteer %)
//   ✕                     motors on
//   ○ or PS               motors off
//
// Safety: the motion sensors report hundreds of times a second while the
// controller is connected, even when it lies still. If they go quiet for
// HEARTBEAT_MS (out of range, battery flat) the controller counts as lost and
// the robot stops at once, instead of waiting for Bluetooth to notice, which
// can take up to 20 s. While a stick is held, drive commands are repeated so
// DriveControl keeps them alive; when the controller is lost they stop and
// DriveControl stops the robot too.
import fs from 'node:fs';
import path from 'node:path';
import { EventEmitter } from 'node:events';

const EV_SYN = 0;
const EV_KEY = 1;
const EV_ABS = 3;

export const ABS = { X: 0, Y: 1, RX: 3, RY: 4 };
export const BTN = { SOUTH: 304, EAST: 305, NORTH: 307, WEST: 308, TL: 310, TR: 311, MODE: 316 };

// What a button press does: [command, value].
const BUTTON_COMMANDS = {
  [BTN.SOUTH]: ['Motors', 1],   // ✕
  [BTN.EAST]: ['Motors', 0],    // ○
  [BTN.MODE]: ['Motors', 0],    // PS
};

const NAME = /wireless controller|dualshock|dualsense/i;
const POLL_MS = 20;
const SCAN_MS = 2000;
const HEARTBEAT_MS = 500;
const REPEAT_MS = 100;

export const DEFAULTS = {
  enabled: true,
  maxThrottle: 50,
  maxSteer: 60,
  deadZone: 0.1,
};

// struct input_event: struct timeval (two longs), u16 type, u16 code, s32 value.
export function eventSize(arch = process.arch) {
  return /64/.test(arch) ? 24 : 16;
}

export function parseEvents(buf, size = eventSize()) {
  const out = [];
  for (let o = 0; o + size <= buf.length; o += size) {
    out.push({ type: buf.readUInt16LE(o + size - 8), code: buf.readUInt16LE(o + size - 6), value: buf.readInt32LE(o + size - 4) });
  }
  return out;
}

export function encodeEvent(type, code, value, size = eventSize()) {
  const buf = Buffer.alloc(size);
  buf.writeUInt16LE(type, size - 8);
  buf.writeUInt16LE(code, size - 6);
  buf.writeInt32LE(value, size - 4);
  return buf;
}

// Stick axis 0..255 (centre 128) -> -1..1 with a dead zone, rescaled so the
// output still starts at 0 at the edge of the dead zone.
export function axis(raw, deadZone) {
  const v = Math.max(-1, Math.min(1, (raw - 128) / 127));
  const a = Math.abs(v);
  if (a <= deadZone) return 0;
  return Math.sign(v) * (a - deadZone) / (1 - deadZone);
}

// Controller state -> { throttle, steer } in -100..100 (stick up = forward).
export function driveFrom(state, options = DEFAULTS) {
  const max = state.buttons.has(BTN.TR) ? 100 : options.maxThrottle;
  const throttle = Math.round(-axis(state.axes[ABS.Y], options.deadZone) * max);
  const steer = Math.round(axis(state.axes[ABS.RX], options.deadZone) * options.maxSteer);
  return { throttle: throttle || 0, steer: steer || 0 };
}

function readText(file) {
  try { return fs.readFileSync(file, 'utf8').trim(); } catch { return ''; }
}

// Finds the controller's event devices. sysDir lists eventN entries.
export function findController(sysDir, inputDir) {
  let entries;
  try { entries = fs.readdirSync(sysDir).filter((e) => /^event\d+$/.test(e)); } catch { return null; }
  const devices = entries.map((e) => ({
    event: path.join(inputDir, e),
    name: readText(path.join(sysDir, e, 'device', 'name')),
    uniq: readText(path.join(sysDir, e, 'device', 'uniq')),
  })).filter((d) => NAME.test(d.name));
  const main = devices.find((d) => !/motion sensors|touchpad/i.test(d.name));
  if (!main) return null;
  const sensors = devices.find((d) => /motion sensors/i.test(d.name) && (!main.uniq || d.uniq === main.uniq));
  return { name: main.name, address: main.uniq, main: main.event, sensors: sensors?.event ?? null };
}

export class Gamepad extends EventEmitter {
  constructor(options = {}, { log = console, sysDir = '/sys/class/input', inputDir = '/dev/input', now = Date.now, arch } = {}) {
    super();
    this.options = { ...DEFAULTS, ...options };
    this.log = log;
    this.sysDir = sysDir;
    this.inputDir = inputDir;
    this.now = now;
    this.size = eventSize(arch);
    this.fds = [];
    this.controller = null;
    this.alive = false;
    this.status = { enabled: this.options.enabled, connected: false, name: null };
  }

  start() {
    if (!this.options.enabled || this.scanTimer) return;
    this.scan();
    this.scanTimer = setInterval(() => { if (!this.controller) this.scan(); }, SCAN_MS);
  }

  stop() {
    clearInterval(this.scanTimer);
    this.scanTimer = null;
    this.disconnect('stopped');
  }

  scan() {
    const found = findController(this.sysDir, this.inputDir);
    if (!found) return;
    const fds = [];
    try {
      for (const file of [found.main, found.sensors].filter(Boolean)) {
        fds.push(fs.openSync(file, fs.constants.O_RDONLY | fs.constants.O_NONBLOCK));
      }
    } catch (err) {
      fds.forEach((fd) => fs.closeSync(fd));
      // Usually EACCES: the user isn't in the "input" group yet.
      if (this.lastError !== err.code) this.log.warn(`gamepad: ${found.name}: ${err.message}`);
      this.lastError = err.code;
      return;
    }
    this.lastError = null;
    this.fds = fds;
    this.controller = found;
    this.alive = false;
    this.resetState();
    this.pollTimer = setInterval(() => this.poll(), POLL_MS);
    if (!found.sensors) {
      this.log.warn('gamepad: no motion sensor device, so a controller out of range is only noticed when Bluetooth drops it');
      this.setAlive(true);
    }
  }

  resetState() {
    this.state = { axes: { [ABS.X]: 128, [ABS.Y]: 128, [ABS.RX]: 128, [ABS.RY]: 128 }, buttons: new Set() };
    this.sent = { throttle: 0, steer: 0, at: 0 };
  }

  // Reports arriving (true) or gone quiet (false). While quiet the devices stay
  // open: if the controller comes back in range, it carries on.
  setAlive(alive, reason) {
    if (alive === this.alive) return;
    this.alive = alive;
    const c = this.controller;
    if (alive) {
      this.setStatus({ connected: true, name: c.name, address: c.address });
      this.log.info(`gamepad: ${c.name} connected`);
    } else {
      // Forget the sticks: they must be moved again before the robot drives.
      this.resetState();
      this.emit('lost');
      this.setStatus({ connected: false, name: null, address: null });
      this.log.info(`gamepad: lost (${reason})`);
    }
  }

  disconnect(reason) {
    clearInterval(this.pollTimer);
    this.pollTimer = null;
    for (const fd of this.fds) { try { fs.closeSync(fd); } catch {} }
    this.fds = [];
    if (!this.controller) return;
    this.setAlive(false, reason);
    this.controller = null;
  }

  setStatus(s) {
    this.status = { enabled: this.options.enabled, ...s };
    this.emit('status', this.status);
  }

  poll() {
    const buf = Buffer.alloc(this.size * 256);
    for (let i = 0; i < this.fds.length; i++) {
      for (;;) {
        let n;
        try {
          n = fs.readSync(this.fds[i], buf, 0, buf.length, null);
        } catch (err) {
          if (err.code === 'EAGAIN') break;
          this.disconnect(err.code);   // ENODEV: Bluetooth dropped it
          return;
        }
        if (n <= 0) break;
        const events = parseEvents(buf.subarray(0, n), this.size);
        if (events.length && i === 1) {
          this.lastHeartbeat = this.now();
          this.setAlive(true);
        }
        if (i === 0 && this.alive) events.forEach((e) => this.handle(e));
        if (n < buf.length) break;
      }
    }
    if (this.controller.sensors && this.alive && this.now() - this.lastHeartbeat > HEARTBEAT_MS) {
      this.setAlive(false, 'no reports');
    }
    if (this.alive) this.update();
  }

  handle(e) {
    if (e.type === EV_ABS) this.state.axes[e.code] = e.value;
    else if (e.type === EV_KEY) {
      if (e.value) this.state.buttons.add(e.code);
      else this.state.buttons.delete(e.code);
      const cmd = e.value === 1 && BUTTON_COMMANDS[e.code];
      if (cmd) this.emit('command', { name: cmd[0], value: cmd[1] });
    }
    // EV_SYN and others: nothing; the state is read after each poll.
  }

  update() {
    const d = driveFrom(this.state, this.options);
    const changed = d.throttle !== this.sent.throttle || d.steer !== this.sent.steer;
    const moving = d.throttle !== 0 || d.steer !== 0;
    if (changed || (moving && this.now() - this.sent.at >= REPEAT_MS)) {
      this.sent = { ...d, at: this.now() };
      this.emit('drive', d);
    }
  }
}

export { EV_SYN, EV_KEY, EV_ABS };
