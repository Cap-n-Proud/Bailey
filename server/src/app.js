// Wires the robot link, the web UI (socket.io), video and telemetry logging.
//
// Socket.io API
//   server -> client
//     snapshot  everything a new client needs (sent on connect)
//     telemetry robot telemetry values (+ state name)
//     gains     { raw, gains } current gains
//     firmware  firmware info (SYS line)
//     autotune  { phase, result }
//     event     { time, kind, text } robot events, errors, motor tests, echoes
//     console   raw lines to/from the robot, except telemetry
//     link      { connected, port, error }
//     camera    { status, error }
//     system    Pi temperature, load, memory, addresses (every 3 s)
//     logging   { active, file, rows }
//   client -> server (all take an ack callback: { ok, error })
//     drive     { throttle, steer }  -100..100, repeat while held (see drive.js)
//     set       { name, value }      a gain in real units or another setting
//     command   { name, value }      SCMD action: Motors, AUTOTUNE, CalibrateZero, ...
//     read      what                 READ printConfig, ...
//     raw       text                 a typed "SCMD ..." / "READ ..." line
//     logging   true | false         record telemetry to CSV
//     power     'shutdown' | 'reboot'
import http from 'node:http';
import path from 'node:path';
import express from 'express';
import { Server as SocketServer } from 'socket.io';
import { RobotProtocol, scmd, read, setGain, sanitizeRaw, PID_SCALE } from './protocol.js';
import { RobotLink } from './robotLink.js';
import { DriveControl } from './drive.js';
import { TelemetryLog } from './telemetryLog.js';
import { Camera } from './camera.js';
import * as system from './system.js';
import { SERVER_DIR } from './config.js';

const VERSION = '2.0.0';
const MAX_EVENTS = 100;
const MAX_CONSOLE = 300;

export function createApp(config, { log = console, link: linkOverride } = {}) {
  const app = express();
  const httpServer = http.createServer(app);
  const io = new SocketServer(httpServer);
  const protocol = new RobotProtocol();
  const link = linkOverride || new RobotLink(config.serial, { serverDir: SERVER_DIR, log });
  const camera = new Camera(config.camera, { log });
  const telemetryLog = new TelemetryLog(config.telemetry.logDir);

  // Latest known robot state, for clients that connect later.
  const robot = {
    telemetry: null,
    gains: null,
    rawGains: null,
    firmware: null,
    autotune: { phase: 'IDLE', result: null },
    link: { connected: false, port: config.serial.port, error: null },
  };
  const events = [];
  const consoleLines = [];

  function pushConsole(direction, text) {
    const entry = { time: Date.now(), direction, text };
    consoleLines.push(entry);
    if (consoleLines.length > MAX_CONSOLE) consoleLines.shift();
    io.emit('console', entry);
  }

  function pushEvent(kind, text) {
    const entry = { time: Date.now(), kind, text };
    events.push(entry);
    if (events.length > MAX_EVENTS) events.shift();
    io.emit('event', entry);
    log.info(`robot ${kind}: ${text}`);
  }

  function send(line) {
    if (!link.write(line)) throw new Error('robot not connected');
    if (!/^SCMD (Throttle|Steer) /.test(line)) pushConsole('out', line);
  }

  const drive = new DriveControl((line) => {
    try { send(line); } catch { /* not connected: the firmware stops by itself */ }
  });

  function loggingStatus() {
    return { active: telemetryLog.active, file: telemetryLog.file, rows: telemetryLog.rows };
  }

  // --- Robot -> clients ---
  link.on('status', (status) => {
    robot.link = status;
    io.emit('link', status);
    if (!status.connected) drive.stop();
  });

  link.on('line', (line) => {
    let msg;
    try {
      msg = protocol.parse(line);
    } catch (err) {
      log.warn(`bad line from robot: ${JSON.stringify(line)}: ${err.message}`);
      return;
    }
    if (!msg) return;
    if (msg.type !== 'telemetry') pushConsole('in', line);

    switch (msg.type) {
      case 'telemetry':
        robot.telemetry = msg.values;
        io.volatile.emit('telemetry', msg.values);
        telemetryLog.add(msg.values, robot.rawGains);
        break;
      case 'pid':
        robot.gains = msg.gains;
        robot.rawGains = msg.raw;
        io.emit('gains', { raw: msg.raw, gains: msg.gains });
        break;
      case 'system':
        robot.firmware = msg.values;
        io.emit('firmware', msg.values);
        break;
      case 'header':
        // Headers come at boot and after READ RemoteInit; ask for the values.
        if (msg.kind === 'PID') setTimeout(() => { try { send(read('PIDParamTX')); } catch {} }, 50);
        if (msg.kind === 'SYS') setTimeout(() => { try { send(read('SYSParamTX')); } catch {} }, 100);
        break;
      case 'event':
        pushEvent('event', msg.text);
        // The Arduino rebooted (e.g. reset button): headers may have changed.
        if (msg.text === 'Ready') setTimeout(() => { try { send(read('RemoteInit')); } catch {} }, 200);
        if (msg.text === 'Fallen' || msg.text === 'Drive timeout') drive.stop();
        break;
      case 'autotune':
        robot.autotune = { phase: msg.phase === 'RESULT' ? 'IDLE' : msg.phase, result: msg.result ?? robot.autotune.result };
        if (msg.phase === 'RESULT') {
          const r = msg.result;
          pushEvent('autotune', r.accepted
            ? `accepted: L = ${(r.length * 100).toFixed(1)} cm, zero ${r.zero.toFixed(2)}°, wobble ${r.before} -> ${r.after}`
            : `rejected (L ${r.lengthFromGravity} / ${r.lengthFromAccel}, R² ${r.r2})`);
        }
        io.emit('autotune', robot.autotune);
        break;
      case 'error':
        if (/^autotune aborted/.test(msg.text)) {
          robot.autotune = { ...robot.autotune, phase: 'IDLE' };
          io.emit('autotune', robot.autotune);
        }
        pushEvent('error', msg.text);
        break;
      case 'motorTest':
        pushEvent('test', `motor test: left ${msg.left} %, right ${msg.right} %`);
        break;
      case 'echo':
      case 'config':
        break;
      default:
        log.debug?.(`unhandled line: ${line}`);
    }
  });

  // --- HTTP ---
  app.disable('x-powered-by');
  app.use(express.static(path.join(SERVER_DIR, 'public')));
  app.get('/stream.mjpg', (req, res) => camera.stream(req, res));
  app.get('/snapshot.jpg', (req, res) => camera.snapshot(req, res));
  app.get('/api/status', (req, res) => {
    res.json({ version: VERSION, robot, camera: { status: camera.status, error: camera.error }, logging: loggingStatus(), system: system.stats() });
  });

  camera.on('status', (s) => io.emit('camera', s));

  // --- Clients ---
  function handler(fn) {
    return (...args) => {
      const ack = typeof args[args.length - 1] === 'function' ? args.pop() : () => {};
      try {
        const result = fn(...args);
        ack({ ok: true, ...(result || {}) });
      } catch (err) {
        ack({ ok: false, error: err.message });
      }
    };
  }

  io.on('connection', (socket) => {
    socket.emit('snapshot', {
      version: VERSION,
      robot,
      events,
      console: consoleLines,
      camera: { status: camera.status, error: camera.error, enabled: config.camera.enabled },
      logging: loggingStatus(),
      system: system.stats(),
      allowShutdown: config.system.allowShutdown,
      pidScale: PID_SCALE,
    });

    socket.on('drive', handler(({ throttle, steer } = {}) => drive.set(socket.id, throttle, steer)));

    socket.on('set', handler(({ name, value } = {}) => send(setGain(name, value))));

    socket.on('command', handler(({ name, value = 1 } = {}) => {
      if (name === 'Motors' && Number(value) === 0) drive.stop();
      send(scmd(name, value));
    }));

    socket.on('read', handler((what) => send(read(what))));

    socket.on('raw', handler((text) => send(sanitizeRaw(text))));

    socket.on('logging', handler((on) => {
      if (on) {
        const names = protocol.headers.T;
        if (!names) throw new Error('no telemetry from the robot yet');
        const file = telemetryLog.start(names, protocol.headers.PID || []);
        log.info(`telemetry: recording to ${file}`);
      } else {
        const file = telemetryLog.stop();
        if (file) log.info(`telemetry: saved ${file}`);
      }
      io.emit('logging', loggingStatus());
      return loggingStatus();
    }));

    socket.on('power', handler((action) => {
      if (!config.system.allowShutdown) throw new Error('disabled in config.json');
      if (action !== 'shutdown' && action !== 'reboot') throw new Error(`unknown action ${action}`);
      drive.stop();
      try { send(scmd('Motors', 0)); } catch {}
      pushEvent('event', `Pi ${action} requested`);
      system.power(action, log).catch((err) => pushEvent('error', `${action} failed: ${err.message}`));
    }));

    socket.on('disconnect', () => drive.release(socket.id));
  });

  const systemTimer = setInterval(() => io.emit('system', system.stats()), 3000);
  const loggingTimer = setInterval(() => { if (telemetryLog.active) io.emit('logging', loggingStatus()); }, 2000);

  return {
    app,
    httpServer,
    io,
    link,
    camera,
    robot,
    start(port = config.http.port) {
      link.start();
      return new Promise((resolve) => httpServer.listen(port, () => resolve(httpServer.address().port)));
    },
    async close() {
      clearInterval(systemTimer);
      clearInterval(loggingTimer);
      drive.stop();
      telemetryLog.stop();
      camera.stop();
      link.close();
      io.close();
      await new Promise((resolve) => httpServer.close(resolve));
    },
  };
}
